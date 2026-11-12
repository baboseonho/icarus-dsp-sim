"""sim_bridge: 시뮬레이션 엔진과 ROS 2 를 잇는 노드 (시뮬 전용).

드론 6대의 칩(2회차 기준 구현 Onboard)을 이 노드 안에서 돌린다. 칩은 link 노드가 보낸
무선 명령을 실제 UDP 포트(47101번부터)로 받는다. 엔진의 실제 위치는 /sim/truth 로
내보내고, mocap 노드가 여기에 잡음을 섞어 /cfN/pose 로 낸다. 엔진은 벽시계에 맞춰
실시간으로 돌고, 궤적과 실시간 배율을 기록했다가 log_path 에 npz 로 저장한다.

Runs the engine and the six drone chips (week-2 reference Onboard) in real time. The chips
receive radio packets over real UDP, and the true positions go out on /sim/truth for
the mocap node. On real hardware this node does not exist: the drones and the camera
take its place.
"""
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from std_msgs.msg import String

from dsp.engine import Engine
from dsp.link import DroneRadio, BASE_PORT
from dsp.ref import w2 as W2
from dsp.onboard import Chip, drone_state

from .common import NAMES, SENSOR_QOS, LATCHED_QOS, spin


class SimBridge(Node):
    def __init__(self):
        super().__init__("sim_bridge")
        self.declare_parameters("", [
            ("n", 6), ("seed", 0), ("downwash", True), ("wind", [0.0, 0.0, 0.0]), ("gust", 0.0),
            ("link_delay", 0.010), ("init", "grid"), ("log_path", ""), ("truth_hz", 100.0), ("port_base", BASE_PORT),
        ])
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.n = int(g("n"))
        self.names = NAMES[: self.n]
        self.eng = Engine(n=self.n, seed=int(g("seed")), wind=tuple(g("wind")), gust=float(g("gust")),
                          downwash=bool(g("downwash")))
        init = initial_positions(str(g("init")), self.n, int(g("seed")))
        self.eng.reset(self.n, init_pos=init)
        self.chips = [Chip(nm, W2.Onboard(self.eng.mass, self.eng.inertia)) for nm in self.names]
        self.radios = [DroneRadio(nm, int(g("port_base")) + i + 1, clock=time.monotonic, delay=float(g("link_delay")))
                       for i, nm in enumerate(self.names)]
        self.pub = self.create_publisher(PoseArray, "/sim/truth", SENSOR_QOS)
        self.create_subscription(String, "/swarm/status", self.on_status, LATCHED_QOS)
        self.log_path = str(g("log_path"))
        self.log = {"t": [], "p": [], "v": [], "thrust": [], "wall": []}
        self.t0_wall = time.monotonic()
        self.steps_per_tick_max = 25      # 많이 밀렸을 때 한 번에 따라잡는 양의 상한
        self.create_timer(1.0 / float(g("truth_hz")), self.on_tick)
        self.create_timer(10.0, self.save)
        self.lag_max = 0.0
        self.done_wall = None
        self.saved_after_done = False
        self.get_logger().info(f"드론 {self.n}대, UDP 포트 {self.radios[0].port}..{self.radios[-1].port}")

    def on_tick(self):
        try:
            self._tick()
        except KeyboardInterrupt:
            pass        # launch 가 끝나며 보낸 종료 신호가 계산 도중에 오면 조용히 멈춘다

    def _tick(self):
        # 벽시계로 지금까지 돌았어야 할 스텝 수만큼 엔진을 진행시킨다.
        target_steps = int((time.monotonic() - self.t0_wall) * self.eng.freq)
        todo = min(target_steps - self.eng.steps, self.steps_per_tick_max)
        self.lag_max = max(self.lag_max, (target_steps - self.eng.steps) / self.eng.freq)
        thrust = np.zeros(self.n)
        for _ in range(max(todo, 0)):
            st = self.eng.true_state()
            t = self.eng.t
            cmds = np.zeros((self.n, 4))
            for i, chip in enumerate(self.chips):
                ds = drone_state(st, i)
                for pkt in self.radios[i].poll():
                    chip.handle(pkt, t, ds)
                cmds[i] = chip.step(t, ds)
            self.eng.step(cmds)
            thrust = cmds[:, 0]
        st = self.eng.true_state()
        msg = PoseArray()
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(self.n):
            ps = Pose()
            ps.position.x, ps.position.y, ps.position.z = (float(v) for v in st["p"][i])
            ps.orientation.w, ps.orientation.x, ps.orientation.y, ps.orientation.z = (float(v) for v in st["q"][i])
            msg.poses.append(ps)
        self.pub.publish(msg)
        L = self.log
        L["t"].append(self.eng.t); L["p"].append(st["p"].copy()); L["v"].append(st["v"].copy())
        L["thrust"].append(thrust.copy()); L["wall"].append(time.monotonic() - self.t0_wall)
        # 끝나고 1.5 s 뒤, 칩이 모터를 끈 상태까지 담아 한 번 더 저장한다.
        if self.done_wall is not None and not self.saved_after_done and time.monotonic() - self.done_wall > 1.5:
            self.saved_after_done = True
            self.save()

    def on_status(self, msg):
        # commander 가 DONE 을 알리면 그 시점을 기억하고 바로 저장한다.
        if ('"done"' in msg.data or '"DONE"' in msg.data) and self.done_wall is None:
            self.done_wall = time.monotonic()
            self.save()

    def save(self):
        """기록을 log_path 에 npz 로 저장한다. 10 s 마다, 그리고 끝날 때 부른다."""
        if not self.log_path or not self.log["t"]:
            return
        a = {k: np.asarray(v) for k, v in self.log.items()}
        imp = np.array(self.eng.ground_impacts, float).reshape(-1, 3)
        modes = np.array([c.mode for c in self.chips])
        np.savez_compressed(self.log_path, **a, impacts=imp, modes=modes, names=np.array(self.names),
                            lag_max=self.lag_max, rtf=self.eng.t / max(1e-9, time.monotonic() - self.t0_wall))


def initial_positions(kind, n, seed):
    """바닥 위 시작 위치 (n, 3). grid 는 0.8 m 간격 2×3 격자, scatter 는 서로 0.6 m 이상 떨어진 무작위 배치."""
    if kind == "grid":
        xs = (np.arange(3) - 1) * 0.8
        pts = [[x, y, 0.0] for y in (-0.4, 0.4) for x in xs]
        return np.array(pts[:n])
    if kind == "scatter":
        rng = np.random.default_rng(100 + seed)
        pts = []
        while len(pts) < n:
            q = rng.uniform(-1.6, 1.6, 2)
            if all(np.linalg.norm(q - r) >= 0.6 for r in pts):
                pts.append(q)
        return np.c_[np.array(pts), np.zeros(n)]
    raise ValueError(kind)


def main():
    rclpy.init()
    node = SimBridge()
    try:
        spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
