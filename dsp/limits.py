"""수업 코드가 노트북 자원을 너무 많이 쓰지 않게 막는다 (기본 80%).

`import dsp`를 하면 바로 적용된다. 그래서 수업 노트북, ROS 2 노드, 시뮬레이터 모두 같은 상한을 받는다.

1. CPU: 앞쪽 N개 코어에만 붙인다 (N = 전체 코어의 80%). 이 프로세스가 띄우는 자식 프로세스도 그대로 물려받아서,
   수업 코드를 아무리 많이 띄워도 합쳐서 N코어를 넘지 못한다. 우선순위도 조금 낮춘다 (nice 5).
2. 메모리 (systemd): Ubuntu 24.04 데스크톱이면 `icarus-dsp.slice`에 들어간다. install.sh가 만든 이 slice는
   수업 프로세스 전체 메모리를 75%에서 늦추고 80%에서 멈춘다.
3. 메모리 (감시): 1초마다 이 프로세스와 자식들의 메모리를 보고, 상한에 가까워지면 실행 중인 셀을 멈춘다.
   systemd를 못 쓸 때도 동작한다.

상한 비율은 setup/hw_profile.env (install.sh가 노트북 사양을 보고 만든 파일)에서 읽는다.
DSP_NO_LIMIT=1 이면 아무것도 하지 않는다.

Keeps the course code at ~80 % of the laptop: pins to 80 % of the cores (children inherit),
joins a memory-capped systemd slice when available, and interrupts the running cell before
memory runs out.
"""
from __future__ import annotations

import _thread
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

PROFILE = Path(__file__).resolve().parent.parent / "setup" / "hw_profile.env"
SLICE = "icarus-dsp.slice"
STATUS: dict = {}
_PAGE = os.sysconf("SC_PAGE_SIZE")


def _profile():
    prof = {}
    if PROFILE.exists():
        for line in PROFILE.read_text().splitlines():
            if "=" in line and not line.startswith("#"):
                k, v = line.split("=", 1)
                prof[k.strip()] = v.strip()
    pct = int(os.environ.get("DSP_LIMIT_PCT", prof.get("DSP_LIMIT_PCT", 80)))
    cores = len(os.sched_getaffinity(0))
    total = os.sysconf("SC_PHYS_PAGES") * _PAGE
    # 사양 파일이 없거나 비율을 바꿨으면 그 자리에서 계산한다
    cpu = int(prof["DSP_CPU_LIMIT"]) if "DSP_CPU_LIMIT" in prof and "DSP_LIMIT_PCT" not in os.environ \
        else max(1, int(int(prof.get("DSP_CORES", cores)) * pct / 100))
    return pct, cpu, int(total * pct / 100), total


def _pin_cpus(n):
    """프로세스의 모든 스레드를 앞쪽 n개 코어에 붙인다. 이후 생기는 스레드와 자식 프로세스도 물려받는다."""
    allowed = sorted(os.sched_getaffinity(0))
    if len(allowed) <= n:
        return allowed
    keep = set(allowed[:n])
    for tid in os.listdir("/proc/self/task"):
        try:
            os.sched_setaffinity(int(tid), keep)
        except OSError:
            pass
    return sorted(keep)


def _join_slice():
    """systemd 사용자 세션의 icarus-dsp.slice로 들어간다. 안 되면 False (그때는 감시만으로 막는다)."""
    unit = Path.home() / ".config/systemd/user" / SLICE
    if not unit.exists() or not Path("/sys/fs/cgroup/cgroup.controllers").exists():
        return False
    if SLICE in Path("/proc/self/cgroup").read_text():
        return True  # 부모가 이미 넣어 줬다 (예: 노트북 커널이 띄운 시뮬레이터)
    pid = os.getpid()
    cmd = ["busctl", "--user", "call", "org.freedesktop.systemd1", "/org/freedesktop/systemd1",
           "org.freedesktop.systemd1.Manager", "StartTransientUnit", "ssa(sv)a(sa(sv))",
           f"icarus-dsp-{pid}.scope", "fail", "2", "PIDs", "au", "1", str(pid), "Slice", "s", SLICE, "0"]
    try:
        subprocess.run(cmd, capture_output=True, timeout=3, check=True)
    except (OSError, subprocess.SubprocessError):
        return False
    for _ in range(20):  # systemd가 옮겨 줄 때까지 잠깐 기다린다
        if SLICE in Path("/proc/self/cgroup").read_text():
            return True
        time.sleep(0.05)
    return False


def _tree_rss(root):
    """root 프로세스와 그 자손들이 쓰는 실제 메모리(바이트)."""
    parent = {}
    for d in os.listdir("/proc"):
        if d.isdigit():
            try:
                stat = Path(f"/proc/{d}/stat").read_text()
                parent[int(d)] = int(stat[stat.rfind(")") + 2:].split()[1])
            except (OSError, ValueError, IndexError):
                pass
    tree, todo = {root}, [root]
    while todo:
        p = todo.pop()
        kids = [c for c, pp in parent.items() if pp == p and c not in tree]
        tree.update(kids)
        todo += kids
    rss = 0
    for p in tree:
        try:
            rss += int(Path(f"/proc/{p}/statm").read_text().split()[1]) * _PAGE
        except (OSError, ValueError, IndexError):
            pass
    return rss, tree - {root}


def _available():
    for line in Path("/proc/meminfo").read_text().splitlines():
        if line.startswith("MemAvailable:"):
            return int(line.split()[1]) * 1024
    return 1 << 62


def _watch(limit, total):
    pid, last = os.getpid(), 0.0
    while True:
        time.sleep(1.0)
        try:
            rss, kids = _tree_rss(pid)
            low = _available() < 0.05 * total and rss > 1 << 30
            if (rss < 0.95 * limit and not low) or time.time() - last < 10:
                continue
            why = f"수업 코드가 메모리를 {rss / 2**30:.1f} GB 쓰고 있어요 (상한 {limit / 2**30:.1f} GB)" if not low \
                else "노트북 메모리가 거의 다 찼어요"
            print(f"\n⚠️ {why}. 노트북이 멈추지 않게 실행을 중단합니다.", file=sys.stderr, flush=True)
            if last and kids:  # 한 번 멈췄는데도 그대로면 자식 프로세스(시뮬레이터 등)를 끈다
                for k in kids:
                    try:
                        os.kill(k, 15)
                    except OSError:
                        pass
            _thread.interrupt_main()
            last = time.time()
        except Exception:  # noqa: BLE001  감시가 수업 코드를 망가뜨리면 안 된다
            continue


def apply():
    """상한을 건다. 여러 번 불러도 한 번만 적용된다."""
    if STATUS or os.environ.get("DSP_NO_LIMIT") == "1" or not sys.platform.startswith("linux"):
        return STATUS
    pct, cpu, limit, total = _profile()
    # 이후 불러올 수치 라이브러리의 스레드 수도 맞춘다
    for var in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_MAX_THREADS"):
        os.environ.setdefault(var, str(cpu))
    cores = _pin_cpus(cpu)
    try:
        if os.nice(0) < 5:
            os.nice(5 - os.nice(0))
    except OSError:
        pass
    cg = _join_slice()
    threading.Thread(target=_watch, args=(limit, total), name="dsp-memory-watch", daemon=True).start()
    STATUS.update(pct=pct, cpus=len(cores), cores_total=os.cpu_count(), mem_limit=limit,
                  mem_total=total, cgroup=cg)
    return STATUS


def describe():
    from .lang import tr
    s = STATUS
    if not s:
        return tr("자원 상한 꺼짐 (DSP_NO_LIMIT=1)", "resource limits off (DSP_NO_LIMIT=1)")
    mem = f"{s['mem_limit'] / 2**30:.1f}/{s['mem_total'] / 2**30:.1f} GB{' (systemd)' if s['cgroup'] else ''}"
    return tr(f"CPU {s['cpus']}/{s['cores_total']}코어, 메모리 최대 {mem}",
              f"CPU {s['cpus']}/{s['cores_total']} cores, memory max {mem}")
