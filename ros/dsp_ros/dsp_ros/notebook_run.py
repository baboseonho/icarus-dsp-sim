"""노트북 안에서 ROS 2 군집을 띄우는 도구 (4·5회차 배경 코드가 쓴다).

    [sim_bridge, mocap, link]  ← 하위 프로세스 (ros2 launch, agents:=false commander:=false)
    [agent ×6, commander, 감시 노드]  ← 노트북 안, 같은 실행기 하나

    run = NotebookSwarm(lambda nm: MyAgent(nm), machine)
    ...                    # run.mon 에 실제 위치, 상태, 토픽 수신 시각이 쌓인다
    run.close()

Runs the simulator side in a subprocess and the agents, commander and a monitor in-process.
"""
import atexit
import os
import signal
import subprocess
import tempfile
import threading
from pathlib import Path

import numpy as np
import rclpy
from std_msgs.msg import String as _StringMsg

from .common import NAMES, make_executor


def ros_up():
    """rclpy를 한 번만 켠다. 노트북 안 노드의 안내 로그는 숨기고 경고와 오류만 보인다."""
    if not rclpy.ok():
        rclpy.init()
        from rclpy.logging import LoggingSeverity, set_logger_level
        for name in [f"agent_{nm}" for nm in NAMES] + ["commander", "dsp_test_monitor"]:
            set_logger_level(name, LoggingSeverity.WARN)
        # 오류로 멈춘 셀을 다시 돌리면 같은 이름의 노드가 잠깐 겹친다는 경고가 나오는데, 동작에는 상관없다
        set_logger_level("rcl.logging_rosout", LoggingSeverity.ERROR)


def String(s):
    return _StringMsg(data=s)


def rate(ts, t0, t1):
    """t0..t1 동안 받은 메시지의 평균 빈도 [Hz]."""
    ts = np.asarray(ts)
    return float(np.sum((ts >= t0) & (ts <= t1)) / max(1e-9, t1 - t0))


class NotebookSwarm:
    """시뮬레이터 쪽은 하위 프로세스로, agent와 지휘자와 감시 노드는 이 노트북 안에서 돌린다."""

    def __init__(self, make_agent, machine, init="scatter", seed=0, names=NAMES, **launch):
        from .commander_node import Commander
        from .testrun import Monitor
        ros_up()
        self.dir = Path(tempfile.mkdtemp(prefix="dsp_nb_"))
        args = ["agents:=false", "commander:=false", f"init:={init}", f"seed:={seed}"]
        args += [f"{k}:={v}" for k, v in launch.items()]          # 예: log_path=... (sim_bridge가 궤적 저장)
        self.log = open(self.dir / "launch.log", "w")
        self.proc = subprocess.Popen(["ros2", "launch", "dsp_ros", "swarm.launch.py"] + args, stdout=self.log,
                                     stderr=subprocess.STDOUT, start_new_session=True, env=os.environ.copy())
        atexit.register(self._kill_sim)        # 커널을 끄거나 다시 시작해도 시뮬레이터가 남지 않게
        self.agents, self.mon, self.cmd = [], None, None
        try:
            self.agents = [make_agent(nm) for nm in names]
            self.mon = Monitor()
            self.cmd = Commander(machine)
        except BaseException:
            # agent를 만들다 실패하면(빈칸 등) 이미 띄운 시뮬레이터를 바로 끈다
            for n in self.agents + [x for x in (self.mon, self.cmd) if x is not None]:
                n.destroy_node()
            self._kill_sim()
            self.log.close()
            raise
        self.exe = make_executor()
        for n in self.agents + [self.mon, self.cmd]:
            self.exe.add_node(n)
        self.stop = False
        self.th = threading.Thread(target=self._spin, daemon=True)
        self.th.start()

    def _kill_sim(self):
        if self.proc.poll() is None:
            try:
                os.killpg(self.proc.pid, signal.SIGINT)
                self.proc.wait(timeout=15)
            except Exception:  # noqa: BLE001
                try:
                    os.killpg(self.proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass

    def _spin(self):
        while not self.stop and rclpy.ok():
            self.exe.spin_once(timeout_sec=0.02)

    def close(self):
        self._kill_sim()
        atexit.unregister(self._kill_sim)
        self.log.close()
        self.stop = True
        self.th.join(timeout=3)
        for n in self.agents + [self.mon, self.cmd]:
            self.exe.remove_node(n)
            n.destroy_node()
