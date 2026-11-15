"""0회차 환경 확인. 모든 줄이 OK면 준비 끝이다.

Week-0 environment check. If every line says OK, you are ready.

    source setup/env.sh && python setup/check_env.py            # 3D 창 시험까지
    python setup/check_env.py --no-window                       # 화면이 없는 환경
"""
import os
import platform
import shutil
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
G, R, Y, E = "\033[32m", "\033[31m", "\033[33m", "\033[0m"
results = []


def item(name, ok, detail="", warn=False):
    tag = f"{G}OK  {E}" if ok else (f"{Y}WARN{E}" if warn else f"{R}FAIL{E}")
    print(f"  [{tag}] {name:<44s} {detail}")
    results.append(ok or warn)


def hover_controller(eng):
    """엔진이 제대로 도는지 보려고 만든 작은 호버 제어기. 수업 코드와는 상관없다.

    A tiny hover controller used only to test the engine; unrelated to the course code.
    """
    from dsp.engine import quat_wxyz_to_rpy
    J, m = eng.inertia, eng.mass

    def ctrl(s, tgt):
        acc = 6 * (tgt - s["p"]) - 4 * s["v"]
        acc[:, 2] += 9.81
        rpy = quat_wxyz_to_rpy(s["q"])
        des = __import__("numpy").c_[(-acc[:, 1] / 9.81).clip(-.4, .4), (acc[:, 0] / 9.81).clip(-.4, .4), 0 * acc[:, 0]]
        tau = J * (300 * (des - rpy) - 30 * s["w"])
        return __import__("numpy").c_[m * acc[:, 2], tau]
    return ctrl


print("\nICARUS DSP 환경 확인\n")
# 1. OS와 Python 버전
osr = Path("/etc/os-release").read_text() if Path("/etc/os-release").exists() else ""
ver = next((l.split("=")[1].strip('"') for l in osr.splitlines() if l.startswith("VERSION_ID=")), "?")
in_docker = Path("/.dockerenv").exists()
# 지원하는 Ubuntu와 ROS 2 배포판 (setup/hw_profile.py와 같은 표)
ROS_FOR_UBUNTU = {"24.04": "jazzy", "26.04": "lyrical"}
item("Ubuntu 24.04 / 26.04", ver in ROS_FOR_UBUNTU, f"VERSION_ID={ver}{' (docker)' if in_docker else ''}")
ROS = os.environ.get("ROS_DISTRO") or os.environ.get("DSP_ROS_DISTRO") or ROS_FOR_UBUNTU.get(ver, "jazzy")
ROS_ROOT = Path("/opt/ros") / ROS
# 24.04에서는 ROS 2 항목이 FAIL이면 안 된다. 26.04는 노트북 커널(3.12)이 ROS 2 Lyrical(Python 3.14)의 rclpy를
# 못 불러오므로 WARN으로만 보이고, 4~5회차는 setup/docker/(24.04 + Jazzy)로 한다
ROS_WARN = ver != "24.04"
ROS_HINT = f" · Ubuntu {ver}: 4~5회차는 setup/docker/" if ROS_WARN else ""
item("Python 3.12", sys.version_info[:2] == (3, 12), platform.python_version())
import dsp  # noqa: E402,F401  불러오는 순간 자원 상한이 걸린다
from dsp.limits import STATUS, describe  # noqa: E402
item(f"자원 상한 ({STATUS.get('pct', '-')}%)", bool(STATUS), describe())

# 2. 엔진을 불러와 1대를 10초 동안 띄워 본다
try:
    import numpy as np
    t0 = time.perf_counter()
    from dsp.engine import Engine
    eng = Engine(n=1)
    eng.reset(1, init_pos=[[0, 0, 1.0]])
    ctrl = hover_controller(eng)
    tgt = np.array([[0, 0, 1.0]])
    for _ in range(10 * eng.freq):
        eng.step(ctrl(eng.true_state(), tgt))
    err = float(np.linalg.norm(eng.true_state()["p"][0] - tgt[0]))
    item("engine (Crazyflow) import + 1 drone 10 s hover", err < 0.02, f"err {100 * err:.2f} cm, {time.perf_counter() - t0:.1f} s")
except Exception as e:
    item("engine (Crazyflow) import + 1 drone 10 s hover", False, repr(e)[:80])

# 3. ROS 2: rclpy import, ros2 topic list, 우리 패키지 빌드 여부
try:
    import rclpy  # noqa: F401
    p = subprocess.run(["ros2", "topic", "list"], capture_output=True, text=True, timeout=20)
    item(f"rclpy import + `ros2 topic list` ({ROS})", p.returncode == 0, p.stdout.strip().replace("\n", " ")[:50],
         warn=ROS_WARN)
except Exception as e:
    item(f"rclpy import + `ros2 topic list` ({ROS})", False, repr(e)[:80] + ROS_HINT, warn=ROS_WARN)
try:
    p = subprocess.run(["ros2", "pkg", "prefix", "dsp_ros"], capture_output=True, text=True, timeout=20)
    item("ROS 2 package dsp_ros built", p.returncode == 0, p.stdout.strip()[-50:] or "colcon build 필요",
         warn=ROS_WARN)
except Exception as e:
    item("ROS 2 package dsp_ros built", False, repr(e)[:80] + ROS_HINT, warn=ROS_WARN)

# 4. 3D 창
if "--no-window" in sys.argv:
    item("3D window (MuJoCo viewer)", False, "--no-window 라서 건너뜀. 창 대신 --video로 영상 저장", warn=True)
else:
    code = ("from dsp.engine import Engine; e=Engine(n=1); e.reset(1, init_pos=[[0,0,1]]);"
            "import time; [e.render(mode='human') for _ in range(20)]; e.close(); print('WINDOW_OK')")
    try:
        p = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=60, cwd=REPO)
        ok = "WINDOW_OK" in p.stdout
        item("3D window (MuJoCo viewer)", ok, "DISPLAY=" + os.environ.get("DISPLAY", "-") if ok
             else (p.stderr.strip().splitlines() or ["?"])[-1][:70], warn=not ok)
    except Exception as e:
        item("3D window (MuJoCo viewer)", False, repr(e)[:70], warn=True)

# 실시간 그래프 도구 (없어도 수업은 된다: 노트북 안 그래프로 대신)
pj = str(ROS_ROOT / "lib" / "plotjuggler" / "plotjuggler")
# Jazzy는 ROS 2 토픽까지 읽는 plotjuggler-ros, 그 플러그인이 아직 없는 Lyrical은 PlotJuggler 본체
pj_pkg = f"ros-{ROS}-plotjuggler-ros" if ROS == "jazzy" else f"ros-{ROS}-plotjuggler"
item("PlotJuggler (실시간 그래프)", os.path.exists(pj), pj if os.path.exists(pj)
     else f"없음. sudo apt install --no-install-recommends {pj_pkg}", warn=True)
rqt = shutil.which("rqt") or (ROS_ROOT / "bin" / "rqt").exists()
item("rqt, RViz (4~5회차 도구)", bool(rqt), "ROS 2 desktop판" if rqt else "base판이라 없음. PlotJuggler와 ros2 명령으로 대신",
     warn=True)

# 5. 6대를 육각형으로 띄우는 데모. 실시간보다 빨라야(RTF > 1) 수업을 따라갈 수 있다
try:
    eng = Engine(n=6, downwash=True)
    ang = np.arange(6) * np.pi / 3
    tgt = np.stack([0.8 * np.cos(ang), 0.8 * np.sin(ang), np.ones(6)], -1)
    eng.reset(6, init_pos=np.c_[tgt[:, :2], np.zeros(6)])
    ctrl = hover_controller(eng)
    t0 = time.perf_counter()
    for _ in range(5 * eng.freq):
        eng.step(ctrl(eng.true_state(), tgt))
    rtf = 5 / (time.perf_counter() - t0)
    err = float(np.abs(eng.true_state()["p"] - tgt).max())
    item("6-drone demo (take-off to hexagon, 5 s)", err < 0.05 and rtf > 1.0, f"max err {100 * err:.1f} cm, RTF {rtf:.1f}x")
except Exception as e:
    item("6-drone demo", False, repr(e)[:80])

ok = all(results)
print(f"\n{G if ok else R}{'모두 통과했습니다.' if ok else 'FAIL 항목을 먼저 고치세요.'}{E}\n")
sys.exit(0 if ok else 1)
