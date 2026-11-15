"""commander: 이륙, 대형 선택, 착륙, 비상 정지를 지휘하는 노드.

    ros2 run dsp_ros commander --scenario w4 --formation hexagon --hold 30
    ros2 run dsp_ros commander --scenario final    # 5회차 ScenarioMachine (기준 구현)

판단은 ROS 를 모르는 상태 기계 클래스가 하고, 이 노드는 관측을 모아 넘겨 주고 돌려받은
행동을 토픽으로 내보내기만 한다. 그래서 같은 상태 기계를 ROS 없는 판에서도 그대로 쓴다.
상태 기계는 step(t, obs) 가 행동 튜플의 리스트를 돌려주고 .state 를 가지면 된다.

The decisions live in a plain-Python state machine (step(t, obs) returns a list of action
tuples and it has a .state); this node only feeds it observations and publishes the actions.
The same machine runs unchanged in the pure-Python version.
"""
import argparse
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Empty, Float64, String

from dsp.grading import formation_error, min_pair_distance
from dsp.ref import w3 as W3

from .common import NAMES, SENSOR_QOS, LATCHED_QOS, CMD_QOS, now_s, pose_to_np, jdump


class W4Machine:
    """4회차용 간단한 시나리오: WAIT -> TAKEOFF -> FORM (hold 초) -> LAND -> DONE.

    Week-4 scenario: take off, hold one formation for `hold` seconds, land.
    """

    def __init__(self, formation="hexagon", hold=30.0, takeoff_h=1.0, center=None):
        self.formation, self.hold, self.h, self.center = formation, hold, takeoff_h, center
        self.state, self.t_state = "WAIT", 0.0

    def _go(self, s, t):
        # 상태를 바꾸고 그 시각을 기억한다 (dt 는 이 시각부터 잰다).
        self.state, self.t_state = s, t

    def step(self, t, obs):
        dt = t - self.t_state
        if self.state == "WAIT" and obs["ready"]:
            self._go("TAKEOFF", t)
            return [("takeoff", self.h, 2.5)]
        if self.state == "TAKEOFF" and dt > 3.5:
            self._go("FORM", t)
            return [("formation", self.formation, self.center)]
        if self.state == "FORM" and dt > self.hold:
            self._go("LAND", t)
            return [("formation", "idle", None), ("land", 4.0)]
        if self.state == "LAND" and dt > 5.0:
            self._go("DONE", t)
            return [("done",)]
        return []

    @property
    def current_formation(self):
        """지금 유지 중인 대형 이름. 대형 비행 중이 아니면 None."""
        return self.formation if self.state == "FORM" else None


class Commander(Node):
    def __init__(self, machine, names=NAMES):
        super().__init__("commander")
        self.m = machine
        self.names = list(names)
        self.P = {nm: None for nm in self.names}
        for nm in self.names:
            self.create_subscription(PoseStamped, f"/{nm}/pose", lambda m, nm=nm: self.on_pose(nm, m), SENSOR_QOS)
        self.cmd = {nm: self.create_publisher(String, f"/{nm}/command", CMD_QOS) for nm in self.names}
        self.form_pub = self.create_publisher(String, "/swarm/formation", LATCHED_QOS)
        self.status_pub = self.create_publisher(String, "/swarm/status", LATCHED_QOS)
        self.emg_pub = self.create_publisher(Empty, "/swarm/emergency", CMD_QOS)
        # PlotJuggler, rqt_plot에서 바로 그릴 수 있게 숫자 하나짜리 토픽도 낸다
        self.err_pub = self.create_publisher(Float64, "/swarm/form_err", 10)
        self.dmin_pub = self.create_publisher(Float64, "/swarm/dmin", 10)
        self.t0 = None
        self.done = False
        self.form_pub.publish(String(data=jdump(name="idle")))
        self.create_timer(0.05, self.on_timer)

    def on_pose(self, nm, msg):
        self.P[nm] = pose_to_np(msg)[0]

    def obs(self):
        """상태 기계에 넘길 관측. 모든 드론의 pose 가 한 번이라도 들어와야 ready 다."""
        ready = all(p is not None for p in self.P.values())
        P = np.array([p if p is not None else np.full(3, np.nan) for p in self.P.values()])
        f = getattr(self.m, "current_formation", None)
        fe = float(formation_error(P, W3.FORMATIONS[f](len(self.names)))) if (ready and f) else float("nan")
        return {"ready": ready, "P": P, "form_err": fe, "dmin": float(min_pair_distance(P)) if ready else float("nan"),
                "z": P[:, 2]}

    def on_timer(self):
        now = now_s(self)
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        o = self.obs()
        for act in self.m.step(t, o):
            self.execute(act)
        self.status_pub.publish(String(data=jdump(state=self.m.state, formation=getattr(self.m, "current_formation", None),
                                                  t=round(t, 2), form_err=o["form_err"], dmin=o["dmin"])))
        if o["ready"]:
            self.err_pub.publish(Float64(data=o["form_err"]))
            self.dmin_pub.publish(Float64(data=o["dmin"]))

    def execute(self, act):
        """행동 튜플 하나를 토픽으로 내보낸다.

        ("takeoff", h, dur)  ("land", dur)  ("formation", name, center)  ("emergency",)  ("done",)
        """
        kind = act[0]
        self.get_logger().info(f"행동: {act}")
        if kind == "takeoff":
            for nm in self.names:
                self.cmd[nm].publish(String(data=jdump(type="takeoff", height=act[1], duration=act[2])))
        elif kind == "land":
            for nm in self.names:
                self.cmd[nm].publish(String(data=jdump(type="land", duration=act[1])))
        elif kind == "formation":
            center = act[2] if len(act) > 2 else None
            self.form_pub.publish(String(data=jdump(name=act[1], center=center)))
        elif kind == "emergency":
            self.emg_pub.publish(Empty())
        elif kind == "done":
            self.status_pub.publish(String(data=jdump(state="DONE", done=True)))
            self.done = True


def main():
    rclpy.init()
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="w4", choices=["w4", "final"])
    ap.add_argument("--formation", default="hexagon")
    ap.add_argument("--hold", type=float, default=30.0)
    ap.add_argument("--center", type=float, nargs=3, default=None)
    args = ap.parse_args(remove_ros_args()[1:])
    if args.scenario == "w4":
        machine = W4Machine(args.formation, args.hold, center=args.center)
    else:
        from dsp.ref import w5 as W5  # 5회차 부품은 5회차에만 불러온다
        machine = W5.ScenarioMachine()
    node = Commander(machine)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        # DONE 을 몇 번 더 알리고 끝낸다. 그 사이 sim_bridge 는 칩이 모터를 끈 상태까지 기록한다.
        t_end = time.monotonic() + 2.5           # 벽시계 기준
        while time.monotonic() < t_end:
            node.status_pub.publish(String(data=jdump(state="DONE", done=True)))
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
