"""노트북 사양을 보고 설치 방법과 자원 상한을 정한다. 설치 전에 한 번 돌린다.

Looks at this laptop (CPU, memory, disk, GPU, battery) and decides how to install
and how much of the machine the course may use (80 % by default).

    python3 setup/hw_profile.py            # 사양과 판단 결과 보기
    python3 setup/hw_profile.py --write    # setup/hw_profile.env 에 저장 (install.sh가 부른다)

시스템 python3만으로 돈다 (설치 전이라 추가 패키지 없음).
상한 비율은 DSP_LIMIT_PCT=70 처럼 바꿀 수 있다.
"""
import os
import shutil
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
OUT = REPO / "setup" / "hw_profile.env"
GB = 1024 ** 3

# 설치에 드는 디스크 (2026-11 깨끗한 24.04에서 잰 값에 여유를 둠)
NEED_DESKTOP_GB = 9.0     # ROS 2 desktop(rviz, rqt 포함) + 기본 도구 약 4 GB, .venv 1.6 GB, 내려받기 임시 파일
NEED_BASE_GB = 6.5        # ROS 2 base(화면 도구 없음) + PlotJuggler와 Qt 약 0.4 GB
MIN_RAM_GB = 4.0


def meminfo():
    info = {}
    for line in Path("/proc/meminfo").read_text().splitlines():
        k, v = line.split(":", 1)
        info[k] = int(v.split()[0]) * 1024
    return info


def cpu_model():
    for line in Path("/proc/cpuinfo").read_text().splitlines():
        if line.startswith("model name"):
            return line.split(":", 1)[1].strip()
    return "알 수 없음"


def gpus():
    try:
        out = subprocess.run(["lspci"], capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    names = []
    for line in out.splitlines():
        if " VGA " in line or "3D controller" in line or "Display controller" in line:
            names.append(line.split(": ", 1)[-1].strip())
    return names


def power():
    """(배터리가 있는지, 충전기가 꽂혀 있는지)"""
    ps = Path("/sys/class/power_supply")
    if not ps.exists():
        return False, True
    has_bat = any(p.name.startswith("BAT") for p in ps.iterdir())
    ac = [p for p in ps.iterdir() if (p / "type").exists() and (p / "type").read_text().strip() == "Mains"]
    plugged = any((p / "online").read_text().strip() == "1" for p in ac if (p / "online").exists()) if ac else True
    return has_bat, plugged


def systemd_user_ok():
    """systemd 사용자 세션이 있고 메모리 상한(cgroup v2)을 걸 수 있는지."""
    if shutil.which("systemd-run") is None or not Path("/sys/fs/cgroup/cgroup.controllers").exists():
        return False
    uid = os.getuid()
    ctrl = Path(f"/sys/fs/cgroup/user.slice/user-{uid}.slice/user@{uid}.service/cgroup.controllers")
    return ctrl.exists() and "memory" in ctrl.read_text().split()


def analyze():
    pct = int(os.environ.get("DSP_LIMIT_PCT", "80"))
    allowed = sorted(os.sched_getaffinity(0))
    cores = len(allowed)
    mem = meminfo()
    ram = mem["MemTotal"]
    disk = shutil.disk_usage(REPO).free
    disk_root = shutil.disk_usage("/").free
    gpu = gpus()
    has_bat, plugged = power()

    cpu_limit = max(1, int(cores * pct / 100))
    mem_limit = int(ram * pct / 100)
    notes, problems = [], []

    # ROS 2: desktop(rviz2, rqt 포함)이 좋지만 디스크와 메모리가 모자라면 base로
    free_for_install = min(disk, disk_root)
    if free_for_install >= NEED_DESKTOP_GB * GB and ram >= 8 * GB:
        ros = "desktop"
    elif free_for_install >= NEED_BASE_GB * GB:
        ros = "base"
        notes.append("ROS 2는 base판으로 깐다 (rviz2/rqt 없음, 수업 진행에는 지장 없음)")
    else:
        ros = "base"
        problems.append(f"디스크 여유가 {free_for_install / GB:.1f} GB라서 부족하다. 최소 {NEED_BASE_GB:.1f} GB 비워야 한다")
    if ram < MIN_RAM_GB * GB:
        problems.append(f"메모리가 {ram / GB:.1f} GB라서 부족하다 (최소 {MIN_RAM_GB:.0f} GB)")

    # 설치 중 동시 작업 수: 코어 상한과 메모리(작업 하나에 1.5 GB쯤) 중 작은 쪽
    jobs = max(1, min(cpu_limit, int(ram / (1.5 * GB))))
    if has_bat and not plugged:
        notes.append("배터리로 돌고 있다. 설치는 20~40분 걸리니 충전기를 꽂는다")
    if any("NVIDIA" in g for g in gpu):
        notes.append("NVIDIA GPU가 있지만 수업 계산은 CPU로만 한다. 그래픽 드라이버나 CUDA는 따로 깔지 않는다")
    cg = systemd_user_ok()
    if not cg:
        notes.append("systemd 메모리 상한을 쓸 수 없어서, 코어 고정과 메모리 감시로만 제한한다")

    return dict(
        DSP_LIMIT_PCT=pct, DSP_CORES=cores, DSP_CPU_LIMIT=cpu_limit,
        DSP_CPU_LIST=",".join(map(str, allowed[:cpu_limit])),
        DSP_RAM_MB=ram // 2**20, DSP_MEM_LIMIT_MB=mem_limit // 2**20,
        DSP_ROS_VARIANT=ros, DSP_JOBS=jobs, DSP_CGROUP=int(cg),
    ), dict(cpu=cpu_model(), disk=disk, gpu=gpu, has_bat=has_bat, plugged=plugged,
            avail=mem["MemAvailable"], notes=notes, problems=problems)


def show(p, x):
    print("== 노트북 사양")
    print(f"  CPU     {x['cpu']} ({p['DSP_CORES']}코어)")
    print(f"  메모리  {p['DSP_RAM_MB'] / 1024:.1f} GB (지금 여유 {x['avail'] / GB:.1f} GB)")
    print(f"  디스크  여유 {x['disk'] / GB:.1f} GB")
    print(f"  GPU     {', '.join(x['gpu']) or '찾지 못함'}")
    print(f"  전원    {'노트북' if x['has_bat'] else '데스크톱/가상머신'}, 충전기 {'연결됨' if x['plugged'] else '빠짐'}")
    print(f"== 판단 (상한 {p['DSP_LIMIT_PCT']}%)")
    print(f"  CPU     최대 {p['DSP_CPU_LIMIT']}코어만 쓴다 (나머지는 화면, VS Code 몫)")
    print(f"  메모리  최대 {p['DSP_MEM_LIMIT_MB'] / 1024:.1f} GB. 넘기 전에 실행 중인 셀을 멈춘다")
    print("  GPU     계산에 쓰지 않는다 (물리 엔진은 CPU, 3D 창은 화면 주사율로 제한)")
    print(f"  ROS 2   {p['DSP_ROS_VARIANT']}판, 설치 동시 작업 {p['DSP_JOBS']}개")
    print(f"  상한    {'systemd(cgroup) + ' if p['DSP_CGROUP'] else ''}코어 고정 + 메모리 감시")
    for n in x["notes"]:
        print(f"  참고: {n}")
    for n in x["problems"]:
        print(f"  \033[31m문제: {n}\033[0m")


def main():
    p, x = analyze()
    show(p, x)
    if "--write" in sys.argv:
        lines = ["# setup/hw_profile.py가 만든 파일. 사양이 바뀌면 install.sh를 다시 돌리거나 이 스크립트를 --write로 실행"]
        lines += [f"{k}={v}" for k, v in p.items()]
        OUT.write_text("\n".join(lines) + "\n")
        print(f"저장: {OUT.relative_to(REPO)}")
    return 2 if x["problems"] else 0


if __name__ == "__main__":
    sys.exit(main())
