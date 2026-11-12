"""mocap: 시뮬레이션 속 천장 카메라.

/sim/truth 로 받은 실제 위치에 잡음, 지연, 가끔의 놓침을 섞어 /cfN/pose 로 100 Hz 발행한다.
실기체에서는 같은 토픽을 내는 OpenCV ArUco 노드가 이 자리에 들어가고, 나머지 노드는 그대로다.

시험용으로 /mocap/disable 에 "cf3" 를 보내면 cf3 의 pose 발행을 멈추고, 빈 문자열을 보내면 다시 켠다.
"cf2,cf5" 처럼 쉼표로 여러 대를 줄 수도 있다.

Simulated ceiling camera: adds noise, delay and dropouts to /sim/truth and publishes
/cfN/pose at 100 Hz. On real hardware an ArUco node publishing the same topics replaces it.
Test hook: send "cf3" on /mocap/disable to stop cf3's pose, "" to turn it back on.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from dsp.mocap import VirtualMocap

from .common import NAMES, SENSOR_QOS, np_to_pose, spin


class MocapNode(Node):
    def __init__(self):
        super().__init__("mocap")
        self.declare_parameters("", [("n", 6), ("rate", 100.0), ("noise", 0.005), ("delay", 0.020),
                                     ("drop", 0.01), ("seed", 0)])
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.names = NAMES[: int(g("n"))]
        self.mc = VirtualMocap(self.names, rate=float(g("rate")), noise=float(g("noise")), delay=float(g("delay")),
                               drop=float(g("drop")), seed=int(g("seed")))
        self.truth = None
        self.disabled = set()
        self.pubs = {nm: self.create_publisher(PoseStamped, f"/{nm}/pose", SENSOR_QOS) for nm in self.names}
        self.create_subscription(PoseArray, "/sim/truth", self.on_truth, SENSOR_QOS)
        self.create_subscription(String, "/mocap/disable", self.on_disable, 10)
        self.create_timer(1.0 / (2 * float(g("rate"))), self.on_timer)   # 지연 큐를 제때 비우려고 발행률의 2배로 돈다

    def on_truth(self, msg):
        P = np.array([[p.position.x, p.position.y, p.position.z] for p in msg.poses])
        Q = np.array([[p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z] for p in msg.poses])
        self.truth = {"p": P, "q": Q}

    def on_disable(self, msg):
        self.disabled = {s for s in msg.data.split(",") if s}
        self.get_logger().warn(f"pose 발행 중지: {sorted(self.disabled)}")

    def on_timer(self):
        if self.truth is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        self.mc.update(now, self.truth)
        for pose in self.mc.poll(now):
            if pose["id"] in self.disabled:
                continue
            self.pubs[pose["id"]].publish(np_to_pose(pose["p"], pose["q"], stamp=pose["t"]))


def main():
    rclpy.init()
    node = MocapNode()
    try:
        spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
