"""5회차 agent 노드: 4회차 Agent에 충돌 회피를 더한다.

척력 계산을 위해 이웃이 아닌 드론의 위치도 구독하고, setpoint는 SwarmGuidance(dsp/swarm.py)로 정한다.
척력과 속도 합치기 함수는 바깥에서 넣는다 (수업 노트북에서는 내가 쓴 함수, 넣지 않으면 기준 구현).
4회차 Agent(dsp/ref/w4.py)가 공개된 뒤, 즉 5회차부터 쓰므로 swarm_agent_class()를 부를 때 만든다.

Week-5 agent node: the week-4 Agent plus collision avoidance through SwarmGuidance.
"""
import json

import rclpy.parameter
from geometry_msgs.msg import PoseStamped

from dsp.swarm import SwarmGuidance

from .common import SENSOR_QOS, now_s


_CLS = None


def swarm_agent_class():
    """SwarmAgent 클래스를 돌려준다. 처음 부를 때 4회차 Agent 위에 만든다."""
    global _CLS
    if _CLS is not None:
        return _CLS
    from dsp.ref.w3 import FORMATIONS
    from dsp.ref.w4 import Agent

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
            self.g = SwarmGuidance(self.i, len(self.names), self.A, k=self.k, k_pin=self.k_pin,
                                   dt=self.dt, leash=self.leash,
                                   repulsion_fn=repulsion_fn, apf_fn=apf_fn)
            self.v_max = self.g.v_max
            self.set_parameters([rclpy.parameter.Parameter("v_max", value=float(self.v_max))])
            for p in ("r_safe", "k_rep"):                   # 척력도 비행 중에 바꿔 볼 수 있게
                self.tunable.append(p)
                self.declare_parameter(p, float(getattr(self.g, p)))

        def on_param(self, name, value):
            setattr(self.g, name, value)

        def on_formation(self, msg):
            c = json.loads(msg.data)
            name = c.get("name", "idle")
            if name not in FORMATIONS:
                self.active = False
                self.g.reset()
                return
            self.g.set_formation(FORMATIONS[name](len(self.names)), c.get("center"))
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

    _CLS = SwarmAgent
    return _CLS
