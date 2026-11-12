"""check.py 가 쓰는 ROS 2 비행 시험 도구.

swarm.launch.py 를 하위 프로세스로 띄우고, 옆에서 감시 노드를 돌리며 상태와 토픽 발행률,
실제 위치를 기록한다.

    with SwarmRun(out_dir, week=4, hold=40, center="0,0,1") as run:
        run.wait(lambda r: r.state == "FORM" and r.since_state() > 20)
        run.disable_pose("cf3")
        run.wait_done(timeout=120)
    t, p = run.truth()     # 실제 위치 기록 (시각은 이 프로세스의 time.monotonic)

Flight-test helper for the check scripts: launches the swarm in a subprocess and records
state changes, topic rates and ground truth from a monitor node.
"""
import json
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from .common import NAMES, SENSOR_QOS, LATCHED_QOS


class Monitor(Node):
    """시험 중에 토픽을 듣기만 하는 노드. 받은 시각은 모두 time.monotonic 으로 적는다."""

    def __init__(self):
        super().__init__("dsp_test_monitor")
        self.truth_t, self.truth_p = [], []
        self.state, self.state_t, self.status = "?", time.monotonic(), {}
        self.states = []
        self.counts = {}
        for nm in NAMES:
            for kind in ("pose", "setpoint"):
                topic = f"/{nm}/{kind}"
                self.counts[topic] = []
                self.create_subscription(PoseStamped, topic, lambda m, tp=topic: self.counts[tp].append(time.monotonic()),
                                         SENSOR_QOS if kind == "pose" else 10)
        self.create_subscription(PoseArray, "/sim/truth", self.on_truth, SENSOR_QOS)
        self.create_subscription(String, "/swarm/status", self.on_status, LATCHED_QOS)
        self.dis = self.create_publisher(String, "/mocap/disable", 10)

    def on_truth(self, msg):
        self.truth_t.append(time.monotonic())
        self.truth_p.append([[p.position.x, p.position.y, p.position.z] for p in msg.poses])

    def on_status(self, msg):
        # 상태나 대형이 바뀐 순간만 states 에 남긴다.
        s = json.loads(msg.data)
        self.status = s
        key = (s.get("state"), s.get("formation"))
        if key != getattr(self, "_key", None):
            self._key = key
            self.state, self.state_t = s.get("state"), time.monotonic()
            self.states.append((self.state, self.state_t, s.get("formation")))


class SwarmRun:
    """with 블록 동안 군집을 띄워 둔다. 키워드 인자는 그대로 launch 인자(k:=v)가 된다."""

    def __init__(self, out_dir, **largs):
        self.out = Path(out_dir); self.out.mkdir(parents=True, exist_ok=True)
        self.largs = {k: str(v) for k, v in largs.items()}
        self.proc = None

    def __enter__(self):
        if not rclpy.ok():
            rclpy.init()
        self.mon = Monitor()
        from .common import make_executor
        self.exe = make_executor()
        self.exe.add_node(self.mon)
        self._spin = threading.Thread(target=self._spin_forever, daemon=True)
        self._stop = False
        self._spin.start()
        cmd = ["ros2", "launch", "dsp_ros", "swarm.launch.py"] + [f"{k}:={v}" for k, v in self.largs.items()]
        self.logf = open(self.out / "launch.log", "w")
        self.proc = subprocess.Popen(cmd, stdout=self.logf, stderr=subprocess.STDOUT, start_new_session=True,
                                     env=os.environ.copy())
        self.t_start = time.monotonic()
        return self

    @property
    def state(self):
        return self.mon.state

    def since_state(self):
        return time.monotonic() - self.mon.state_t

    def _spin_forever(self):
        while not self._stop and rclpy.ok():
            self.exe.spin_once(timeout_sec=0.02)

    def spin(self, dt=0.05):
        time.sleep(dt)

    def wait(self, cond, timeout=120.0):
        """cond(self) 가 참이 될 때까지 기다린다. launch 가 먼저 끝나면 마지막으로 한 번 더 확인한다."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            self.spin()
            if cond(self):
                return True
            if self.proc.poll() is not None:
                return cond(self)
        return False

    def wait_done(self, timeout=180.0):
        ok = self.wait(lambda r: r.state == "DONE", timeout)
        # commander 가 끝나면 launch 도 스스로 내려간다. 그럴 시간을 준다.
        t0 = time.monotonic()
        while self.proc.poll() is None and time.monotonic() - t0 < 15:
            self.spin()
        return ok

    def disable_pose(self, names):
        """mocap 이 이 드론들의 pose 를 내지 않게 한다. 이름 하나나 리스트."""
        self.mon.dis.publish(String(data=",".join([names] if isinstance(names, str) else names)))

    def rate(self, topic, t0, t1):
        """t0..t1 (time.monotonic) 동안 topic 을 받은 평균 빈도 [Hz]."""
        ts = np.asarray(self.mon.counts.get(topic, []))
        n = np.sum((ts >= t0) & (ts <= t1))
        return n / max(1e-9, t1 - t0)

    def state_time(self, name):
        """상태 name 에 처음 들어간 시각. 없으면 None."""
        for s, t, *_ in self.mon.states:
            if s == name:
                return t
        return None

    def capture_graph(self, png):
        from .graph_capture import capture
        return capture(self.mon, png)

    def truth(self):
        return np.asarray(self.mon.truth_t), np.asarray(self.mon.truth_p)

    def __exit__(self, *exc):
        if self.proc and self.proc.poll() is None:
            try:
                # Ctrl-C 와 같은 SIGINT 를 프로세스 그룹 전체에 보내고, 안 내려가면 SIGKILL.
                os.killpg(self.proc.pid, signal.SIGINT)
                self.proc.wait(timeout=20)
            except Exception:  # noqa
                os.killpg(self.proc.pid, signal.SIGKILL)
        self.logf.close()
        self._stop = True
        self._spin.join(timeout=2)
        self.exe.remove_node(self.mon)
        self.mon.destroy_node()
        return False
