"""agent 노드의 뼈대. 4회차 수업에서 부원이 이 클래스를 이어받아 세 곳을 채운다.

    /cfj/pose (이웃 j) ──▶ [agent_cfi] ──▶ /cfi/setpoint ──▶ link ──UDP──▶ 드론 i

여기 있는 것은 미리 주는 부분이다: 구독과 발행 만들기, 내 위치와 대형 명령 받기, 속도 제한과 끈.
채울 곳 세 군데는 비워 두었다.
  ① find_neighbors()            통신 그래프에서 내 이웃 이름 목록
  ② on_neighbor_pose(nb, msg)   이웃 위치가 오면 저장
  ③ on_timer()                  50 Hz로 합의 한 걸음, setpoint 발행, 소식이 끊기면 제자리

The provided skeleton of an agent node. Week 4 fills in ① the neighbour list, ② the
neighbour-pose callback and ③ the 50 Hz timer.
"""
import json

import numpy as np
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from dsp.formation import GRAPHS
from .common import NAMES, SENSOR_QOS, LATCHED_QOS, now_s, pose_to_np, np_to_pose

STALE = 0.3                        # 이보다 오래된 위치는 믿지 않는다 [s]


class AgentBase(Node):
    """agent_N: 드론 한 대를 맡는 지상국 노드의 뼈대.

    One drone's ground-station node, minus the three parts filled in during week 4.
    """

    def __init__(self, name, names=NAMES, graph="ring", k=0.02, rate=50.0, k_pin=0.01, v_max=0.5, leash=0.4):
        super().__init__(f"agent_{name}")
        self.name, self.names = name, list(names)
        self.i = self.names.index(name)
        self.A = GRAPHS[graph](len(self.names))
        self.k, self.k_pin, self.v_max, self.leash, self.dt = k, k_pin, v_max, leash, 1.0 / rate
        self.error = None          # 콜백 안에서 난 첫 오류 (노트북이 보여 준다)
        # 비행 중에도 바꿀 수 있는 값: ros2 param set /agent_cf1 k 0.05, 또는 rqt_reconfigure
        self.tunable = ["k", "k_pin", "v_max"]
        for p in self.tunable:
            self.declare_parameter(p, float(getattr(self, p)))
        self.add_on_set_parameters_callback(self._on_params)

        self.neighbors = list(self._safe(self.find_neighbors)() or [])      # ①
        self.nb = {}               # 이웃 이름 → (위치 (3,), 받은 시각)
        for nb in self.neighbors:  # 구독 하나가 통신 그래프의 화살표 하나다
            self.create_subscription(PoseStamped, f"/{nb}/pose",
                                     lambda msg, nb=nb: self._safe(self.on_neighbor_pose)(nb, msg), SENSOR_QOS)

        self.own = None            # (내 위치, 받은 시각)
        self.create_subscription(PoseStamped, f"/{name}/pose", self.on_own_pose, SENSOR_QOS)
        self.create_subscription(String, "/swarm/formation", self.on_formation, LATCHED_QOS)
        self.pub = self.create_publisher(PoseStamped, f"/{name}/setpoint", 10)
        self.create_timer(self.dt, self._safe(self.on_timer))

        self.active = False        # 대형 비행 중인지
        self.o = None              # 모두의 오프셋 (n, 3)
        self.center = None
        self.x = None              # 내 상태 x_i = r_i − o_i (기준점 − 오프셋)
        self.holding = None        # 제자리 유지 중이면 그 원인이 된 이름들
        self.get_logger().info(f"{name}: 이웃 {self.neighbors}")

    # 채울 곳 ---------------------------------------------------------------------

    def find_neighbors(self):
        """① 이웃 = A의 self.i 행에서 a_ij > 0 인 j. 번호가 아니라 이름(self.names[j])으로, 자기 자신은 빼고."""
        raise NotImplementedError("① find_neighbors")

    def on_neighbor_pose(self, nb, msg):
        """② 이웃 nb의 위치 메시지를 받으면 (위치, 받은 시각)을 self.nb[nb]에 저장한다."""
        raise NotImplementedError("② on_neighbor_pose")

    def on_timer(self):
        """③ 50 Hz. 소식이 끊겼으면 제자리, 아니면 합의 한 걸음 뒤 setpoint를 낸다."""
        raise NotImplementedError("③ on_timer")

    # 미리 주는 부분 ------------------------------------------------------------------

    def _on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name in self.tunable:
                setattr(self, p.name, float(p.value))
                self.on_param(p.name, float(p.value))
        return SetParametersResult(successful=True)

    def on_param(self, name, value):
        """파라미터가 바뀐 뒤 부른다. 5회차 agent가 유도 계산 쪽에도 값을 넘길 때 쓴다."""

    def _safe(self, fn):
        """콜백에서 난 오류를 삼키지 않고 기록해 둔다. 실행기가 멈추지 않게 첫 오류만 남긴다."""
        def run(*a, **kw):
            try:
                return fn(*a, **kw)
            except Exception as e:  # noqa: BLE001
                if self.error is None:
                    self.error = e
                    self.get_logger().error(f"{self.name}: {type(e).__name__}: {e}")
        return run

    def on_own_pose(self, msg):
        p, _, _ = pose_to_np(msg)
        self.own = (p, now_s(self))

    def on_formation(self, msg):
        """지휘자의 대형 명령. {"name": "hexagon", "center": [x, y, z] 또는 null}"""
        from dsp.ref.w3 import FORMATIONS          # 3회차가 끝난 뒤 공개되는 대형 오프셋
        c = json.loads(msg.data)
        name = c.get("name", "idle")
        if name not in FORMATIONS:                 # idle, hold, land 이면 대형 비행을 끝낸다
            self.active, self.x = False, None
            return
        o_new = FORMATIONS[name](len(self.names))
        if self.x is not None and self.o is not None:
            self.x = self.x + self.o[self.i] - o_new[self.i]      # 기준점은 제자리에 둔다
        self.o = o_new
        self.center = None if c.get("center") is None else np.asarray(c["center"], float)
        self.active = True

    def limit(self, x_new, p_me):
        """중심 끌림(pinning), 속도 제한, 끈(leash)을 차례로 적용한다."""
        if self.center is not None:
            x_new = x_new + self.k_pin * (self.center - self.x)
        step = x_new - self.x
        n = np.linalg.norm(step)
        if n > self.v_max * self.dt:
            x_new = self.x + step * self.v_max * self.dt / n
        dev = x_new + self.o[self.i] - p_me
        d = np.linalg.norm(dev)
        if d > self.leash:
            x_new = p_me - self.o[self.i] + dev * self.leash / d
        return x_new

    def publish_setpoint(self, p):
        self.pub.publish(np_to_pose(p, stamp=now_s(self)))
