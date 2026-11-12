"""link: 지상국 쪽 무선. ROS 토픽을 받아 드론에 UDP 패킷으로 보낸다.

  /cfN/setpoint     ->  cmd_position (드론마다 link_hz 이하로)
  /cfN/command      ->  takeoff, go_to, land, emergency_stop
  /swarm/emergency  ->  모든 드론에 emergency_stop

ROS 2 는 노트북 안에서만 쓰고, Wi-Fi 너머로는 DDS 대신 UDP 만 오간다.
실기체로 갈 때는 UdpLink 를 EspUdpLink 로 바꾸기만 하면 된다.

Ground-side radio: turns ROS topics into UDP packets for the drones. DDS never leaves
the laptop. On real drones, swap UdpLink for EspUdpLink and nothing else changes.
"""
import json
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Empty, String

from dsp.link import UdpLink, BASE_PORT

from .common import NAMES, CMD_QOS, pose_to_np, spin


def yaw_of(q):
    """쿼터니언 (w, x, y, z) 의 yaw [rad]."""
    w, x, y, z = q
    return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


class LinkNode(Node):
    def __init__(self):
        super().__init__("link")
        self.declare_parameters("", [("n", 6), ("loss", 0.02), ("seed", 0), ("link_hz", 50.0), ("port_base", BASE_PORT)])
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.names = NAMES[: int(g("n"))]
        ports = {nm: int(g("port_base")) + i + 1 for i, nm in enumerate(self.names)}
        self.link = UdpLink(self.names, clock=time.monotonic, loss=float(g("loss")), seed=int(g("seed")), ports=ports)
        self.min_dt = 1.0 / float(g("link_hz"))
        self.last = {nm: -1.0 for nm in self.names}
        self.sent_sp = {nm: 0 for nm in self.names}
        for nm in self.names:
            self.create_subscription(PoseStamped, f"/{nm}/setpoint", lambda m, nm=nm: self.on_setpoint(nm, m), 10)
            self.create_subscription(String, f"/{nm}/command", lambda m, nm=nm: self.on_command(nm, m), CMD_QOS)
        self.create_subscription(Empty, "/swarm/emergency", self.on_emergency, CMD_QOS)

    def on_setpoint(self, nm, msg):
        now = time.monotonic()
        if now - self.last[nm] < 0.8 * self.min_dt:      # link_hz 보다 자주 보내지 않는다 (0.8 은 타이머 흔들림 여유)
            return
        self.last[nm] = now
        p, q, _ = pose_to_np(msg)
        self.link[nm].cmd_position(p, yaw_of(q))
        self.sent_sp[nm] += 1

    def on_command(self, nm, msg):
        c = json.loads(msg.data)
        cf = self.link[nm]
        typ = c.get("type")
        if typ == "takeoff":
            cf.takeoff(c["height"], c["duration"])
        elif typ == "go_to":
            cf.go_to(c["p"], c.get("yaw", 0.0), c["duration"])
        elif typ == "land":
            cf.land(c["duration"])
        elif typ == "emergency_stop":
            cf.emergency_stop()
        self.get_logger().info(f"{nm}: {typ}")

    def on_emergency(self, _):
        self.get_logger().error("비상 정지: 모든 드론 emergency_stop")
        for nm in self.names:
            self.link[nm].emergency_stop()


def main():
    rclpy.init()
    node = LinkNode()
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
