"""기준 agent 노드 (4회차 Agent, 5회차 SwarmAgent).

수업 노트북에서 부원이 쓴 Agent가 없을 때, 그리고 `ros2 launch dsp_ros swarm.launch.py`로
강사가 시연할 때 이 클래스를 쓴다.

    /cfj/pose (이웃 j) ──▶ [agent_cfi] ──▶ /cfi/setpoint ──▶ link ──UDP──▶ 드론 i

Reference agent nodes: the week-4 Agent (distributed consensus) and the week-5
SwarmAgent (consensus + collision avoidance).
"""
import json

import numpy as np
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from dsp.ref import w3 as W3
from .common import NAMES, SENSOR_QOS, LATCHED_QOS, now_s, pose_to_np, np_to_pose

consensus_step = W3.consensus_step

STALE = 0.3                        # 이보다 오래된 위치는 믿지 않는다 [s]


class Agent(Node):
    """agent_N: 드론 한 대를 맡는 지상국 노드.

    One drone's ground-station node.
    """

    def __init__(self, name, names=NAMES, graph="ring", k=0.02, rate=50.0, k_pin=0.01, v_max=0.5, leash=0.4):
        super().__init__(f"agent_{name}")
        self.name, self.names = name, list(names)
        self.i = self.names.index(name)
        self.A = W3.GRAPHS[graph](len(self.names))
        self.k, self.k_pin, self.v_max, self.leash, self.dt = k, k_pin, v_max, leash, 1.0 / rate

        # 이웃 = A의 self.i 행에서 a_ij > 0 인 j. 번호가 아니라 이름(self.names[j])으로, 자기 자신은 빼고.
        # Neighbours: names j with a_ij > 0 in row self.i of A, excluding yourself.
        self.neighbors = [self.names[j] for j in np.nonzero(self.A[self.i])[0] if j != self.i]
        self.nb = {}               # 이웃 이름 → (위치 (3,), 받은 시각)
        for nb in self.neighbors:  # 구독 하나가 통신 그래프의 화살표 하나다
            self.create_subscription(PoseStamped, f"/{nb}/pose",
                                     lambda msg, nb=nb: self.on_neighbor_pose(nb, msg), SENSOR_QOS)

        self.own = None            # (내 위치, 받은 시각)
        self.create_subscription(PoseStamped, f"/{name}/pose", self.on_own_pose, SENSOR_QOS)
        self.create_subscription(String, "/swarm/formation", self.on_formation, LATCHED_QOS)
        self.pub = self.create_publisher(PoseStamped, f"/{name}/setpoint", 10)
        self.create_timer(self.dt, self.on_timer)

        self.active = False        # 대형 비행 중인지
        self.o = None              # 모두의 오프셋 (n, 3)
        self.center = None
        self.x = None              # 내 상태 x_i = r_i − o_i (기준점 − 오프셋)
        self.holding = None        # 제자리 유지 중이면 그 원인이 된 이름들
        self.get_logger().info(f"{name}: 이웃 {self.neighbors}")

    # 콜백

    def on_neighbor_pose(self, nb, msg):
        # 콜백은 저장만 한다. 위치와 함께 '받은 시각'(now_s(self), 내 시계)을 저장해야 4.3의 stale 검사가 된다.
        # Only store here: the position and the receive time now_s(self), not the message stamp.
        p, _, _ = pose_to_np(msg)
        self.nb[nb] = (p, now_s(self))

    def on_own_pose(self, msg):
        p, _, _ = pose_to_np(msg)
        self.own = (p, now_s(self))

    def on_formation(self, msg):
        """지휘자의 대형 명령. {"name": "hexagon", "center": [x, y, z] 또는 null}"""
        c = json.loads(msg.data)
        name = c.get("name", "idle")
        if name not in W3.FORMATIONS:              # idle, hold, land 이면 대형 비행을 끝낸다
            self.active, self.x = False, None
            return
        o_new = W3.FORMATIONS[name](len(self.names))
        if self.x is not None and self.o is not None:
            self.x = self.x + self.o[self.i] - o_new[self.i]      # 기준점은 제자리에 둔다
        self.o = o_new
        self.center = None if c.get("center") is None else np.asarray(c["center"], float)
        self.active = True

    # 50 Hz 타이머

    def on_timer(self):
        if not self.active or self.own is None:
            return
        now = now_s(self)
        p_me = self.own[0]
        if self.x is None:
            self.x = p_me - self.o[self.i]
        # 4.3 내 위치나 이웃 위치가 오래됐으면 기준점을 얼리고 제자리를 지킨다.
        stale = [n for n in self.neighbors if n not in self.nb or now - self.nb[n][1] > STALE]
        if now - self.own[1] > STALE:
            stale.append(self.name)
        if stale:
            if self.holding != stale:
                self.get_logger().warn(f"{self.name}: {stale} 위치가 끊겨 제자리 유지")
            self.holding = stale
            self.publish_setpoint(self.x + self.o[self.i])
            return
        self.holding = None

        # 나와 이웃만 담은 작은 문제를 만든다. X_loc = [내 x; 이웃마다 p̂_j − o_j],
        # A_loc 은 첫 행만 a_ij (나머지는 0). consensus_step 결과의 0번 행이 내 새 x다.
        # Build a small local problem: X_loc = [my x; p̂_j − o_j for each neighbour],
        # A_loc has only its first row set to a_ij; row 0 of consensus_step(...) is my new x.
        idx = [self.names.index(n) for n in self.neighbors]
        X_loc = np.vstack([self.x] + [self.nb[n][0] - self.o[j] for n, j in zip(self.neighbors, idx)])
        A_loc = np.zeros((len(X_loc), len(X_loc)))
        A_loc[0, 1:] = self.A[self.i, idx]
        x_new = consensus_step(X_loc, A_loc, self.k)[0]
        x_new = self.limit(x_new, p_me)

        # 새 x를 저장하고 내 기준점 x + o_i 를 setpoint로 낸다.
        # Keep the new x and publish my reference x + o_i as the setpoint.
        self.x = x_new
        self.publish_setpoint(self.x + self.o[self.i])

    # 여기부터는 채울 곳이 없다.

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


class SwarmAgent(Agent):
    """5회차 ROS 2 agent. 4회차 Agent에 두 가지를 더한다.
    척력 계산을 위해 이웃이 아닌 드론의 위치도 구독하고, setpoint는 SwarmGuidance로 정한다.

    Week-5 ROS 2 agent: the week-4 Agent, plus subscriptions to every other drone's
    pose (needed for repulsion), with setpoints computed by SwarmGuidance.
    """

    def __init__(self, name, graph="ring", repulsion_fn=None, apf_fn=None, **kw):
        super().__init__(name, graph=graph, **kw)
        self.others = [n for n in self.names if n != name]
        for nb in self.others:
            if nb not in self.neighbors:          # 이웃은 4회차 코드에서 이미 구독 중
                self.create_subscription(PoseStamped, f"/{nb}/pose",
                                         lambda msg, nb=nb: self.on_neighbor_pose(nb, msg), SENSOR_QOS)
        from dsp.ref import w5 as W5  # 5회차 부품은 5회차에만 불러온다
        self.g = W5.SwarmGuidance(self.i, len(self.names), self.A, k=self.k, k_pin=self.k_pin, v_max=self.v_max,
                               dt=self.dt, leash=self.leash,
                               repulsion_fn=repulsion_fn, apf_fn=apf_fn)

    def on_formation(self, msg):
        c = json.loads(msg.data)
        name = c.get("name", "idle")
        if name not in W3.FORMATIONS:
            self.active = False
            self.g.reset()
            return
        self.g.set_formation(W3.FORMATIONS[name](len(self.names)), c.get("center"))
        self.o = self.g.o
        self.active = True

    def on_timer(self):
        if not self.active or self.own is None:
            return
        now = now_s(self)
        p_me = self.own[0]
        stale = [n for n in self.neighbors if n not in self.nb or now - self.nb[n][1] > 0.3]
        if now - self.own[1] > 0.3:
            stale.append(self.name)
        if stale and self.g.x is not None:
            self.publish_setpoint(self.g.x + self.g.o[self.i])      # 소식이 끊기면 제자리 유지
            return
        P_nb = {self.names.index(n): self.nb[n][0] for n in self.neighbors}
        P_others = {self.names.index(n): self.nb[n][0] for n in self.others if n in self.nb and now - self.nb[n][1] < 0.3}
        self.publish_setpoint(self.g.update(p_me, P_nb, P_others, now))
