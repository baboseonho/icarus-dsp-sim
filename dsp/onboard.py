"""드론 칩 블록 `onboard` (시뮬 펌웨어).

칩이 하는 일은 세 가지다.

1. 무선 패킷을 받는다 (takeoff, go_to, cmd_position, land, emergency_stop).
2. takeoff/go_to/land를 부드러운 min-jerk 궤적으로 바꾼다. Crazyflie의
   high-level commander와 같은 방식이다.
3. 부원이 만든 다단 PID를 불러 [총추력, τx, τy, τz]를 엔진에 넘긴다.
   - position_update(state, sp, dt), 100 Hz: (thrust, [roll_d, pitch_d, yaw_d])를 돌려준다
   - attitude_update(state, att_d, dt), 500 Hz: tau (3,)를 돌려준다

실기체에서는 이 블록 전체가 펌웨어에 해당한다.

Drone-chip block (simulated firmware). It receives radio packets, turns
takeoff/go_to/land into smooth min-jerk trajectories like the Crazyflie
high-level commander, and calls your cascaded PID (position loop at 100 Hz,
attitude loop at 500 Hz) to produce thrust and torques for the engine. On a
real drone this whole block is the firmware.
"""
from __future__ import annotations

import numpy as np

from . import config as C
from . import link as L
from .engine import quat_wxyz_to_rpy

STREAM_TIMEOUT = 0.5   # 스트림이 이 시간[s] 넘게 끊기면 제자리에 멈춘다
STREAM_JUMP = 0.05     # 한 패킷에 이보다 크게[m] 움직이면 궤적이 아니라 웨이포인트로 보고 속도 앞먹임을 0으로


def min_jerk(p0, p1, T, tau):
    """p0에서 p1까지 T초 동안의 최소 저크 보간. 시각 tau에서의 (p, v, a)를 돌려준다.

    Minimum-jerk interpolation from p0 to p1 over T seconds; returns (p, v, a) at time tau.
    """
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    if T <= 1e-6:
        return p1.copy(), np.zeros_like(p1), np.zeros_like(p1)
    s = np.clip(tau / T, 0.0, 1.0)
    pos = 10 * s**3 - 15 * s**4 + 6 * s**5
    vel = (30 * s**2 - 60 * s**3 + 30 * s**4) / T
    acc = (60 * s - 180 * s**2 + 120 * s**3) / T**2
    d = p1 - p0
    return p0 + d * pos, d * vel, d * acc


class Planner:
    """Crazyflie high-level commander를 흉내 낸 궤적 생성기.

    Trajectory generator modelled on the Crazyflie high-level commander.
    """

    def __init__(self):
        self.p0 = np.zeros(3); self.p1 = np.zeros(3); self.t0 = 0.0; self.T = 0.0
        self.yaw0 = 0.0; self.yaw1 = 0.0

    def start(self, t, p_from, p_to, duration, yaw_from=0.0, yaw_to=0.0):
        self.p0, self.p1 = np.asarray(p_from, float).copy(), np.asarray(p_to, float).copy()
        self.t0, self.T = t, max(float(duration), 0.0)
        self.yaw0, self.yaw1 = yaw_from, yaw_to

    def done(self, t):
        return t - self.t0 >= self.T

    def __call__(self, t):
        p, v, a = min_jerk(self.p0, self.p1, self.T, t - self.t0)
        s = 1.0 if self.T <= 0 else min(max((t - self.t0) / self.T, 0.0), 1.0)
        return {"p": p, "v": v, "a": a, "yaw": self.yaw0 + (self.yaw1 - self.yaw0) * s}


class LevelHold:
    """기체를 수평으로 유지하는 간단한 자세 PD. 1회차에서만 쓰고, 2회차부터는 직접 만든 자세 제어로 바뀐다.

    Simple level-hold attitude PD used in week 1 only. From week 2 your own
    attitude loop replaces it.
    """

    def __init__(self, inertia, kp=200.0, kd=25.0):
        self.J, self.kp, self.kd = np.asarray(inertia, float), kp, kd

    def __call__(self, state, att_d):
        e = np.asarray(att_d) - state["rpy"]
        e[2] = (e[2] + np.pi) % (2 * np.pi) - np.pi
        return self.J * (self.kp * e - self.kd * state["w"])


def drone_state(true_state, i):
    """엔진 상태에서 드론 i의 상태만 꺼낸다. 칩의 상태 추정기 자리이고, 지금은 오차 없는 이상적인 추정이다.

    Drone i's onboard state estimate. Taken straight from the engine, so it is ideal.
    """
    q = true_state["q"][i]
    return {"t": true_state["t"], "p": true_state["p"][i].copy(), "v": true_state["v"][i].copy(),
            "q": q.copy(), "rpy": quat_wxyz_to_rpy(q), "w": true_state["w"][i].copy()}


class Chip:
    """드론 한 대의 칩.

    One drone's chip. `controller` must provide:

        position_update(state, sp, dt) -> (thrust, att_d)
        attitude_update(state, att_d, dt) -> tau
    """

    def __init__(self, name, controller, pos_hz=C.POS_HZ, att_hz=C.ATT_HZ):
        self.name = name
        self.ctrl = controller
        self.pos_dt, self.att_dt = 1.0 / pos_hz, 1.0 / att_hz
        self.mode = "idle"            # idle | hl | stream | stopped
        self.planner = Planner()
        self.sp = None
        self.landing = False
        self._last_stream = -np.inf
        self._stream_prev = None          # 직전 스트림 setpoint (stamp, p)
        self._v_ff = np.zeros(3)
        self._next_pos = 0.0
        self._next_att = 0.0
        self.thrust = 0.0
        self.att_d = np.zeros(3)
        self.tau = np.zeros(3)
        self.events = []

    # 무선 명령
    def handle(self, pkt: L.Packet, t, st):
        d = pkt.data
        here = self.sp["p"] if (self.sp is not None and self.mode in ("hl", "stream")) else st["p"]
        yaw_here = self.sp["yaw"] if (self.sp is not None and self.mode in ("hl", "stream")) else st["rpy"][2]
        if self.mode == "stopped":
            return
        if pkt.type == L.STOP:
            self.mode = "stopped"
        elif pkt.type == L.TAKEOFF:
            if self.mode == "idle" and hasattr(self.ctrl, "reset"):
                self.ctrl.reset()
            self.planner.start(t, here, [here[0], here[1], d[0]], d[1], yaw_here, yaw_here)
            self.mode, self.landing = "hl", False
        elif pkt.type == L.GOTO:
            if self.mode == "idle":
                return
            self.planner.start(t, here, d[0:3], d[4], yaw_here, d[3])
            self.mode, self.landing = "hl", False
        elif pkt.type == L.LAND:
            if self.mode == "idle":
                return
            self.planner.start(t, here, [here[0], here[1], 0.0], d[0], yaw_here, yaw_here)
            self.mode, self.landing = "hl", True
        elif pkt.type == L.POSITION:
            if self.mode == "idle" or self.landing:
                return               # 땅 위에 있거나 착륙 중에는 스트림을 무시한다
            p_new = np.array(d[0:3])
            # Crazyflie의 cmd_position은 속도를 0으로 두지만, 우리 펌웨어는 스트림 setpoint를
            # 차분해서 속도 앞먹임으로 쓴다. 움직이는 대형을 따라갈 때 뒤처짐이 줄어든다.
            if self.mode == "stream" and self._stream_prev is not None and pkt.stamp > self._stream_prev[0]:
                dp = p_new - self._stream_prev[1]
                v_raw = dp / (pkt.stamp - self._stream_prev[0]) if np.linalg.norm(dp) < STREAM_JUMP else np.zeros(3)
                self._v_ff = 0.6 * self._v_ff + 0.4 * v_raw
            else:
                self._v_ff = np.zeros(3)
            self._stream_prev = (pkt.stamp, p_new)
            self.sp = {"p": p_new, "v": self._v_ff.copy(), "a": np.zeros(3), "yaw": d[3]}
            self.mode = "stream"
            self._last_stream = t
        self.events.append((t, pkt.name))

    # 제어
    def step(self, t, st):
        """물리 스텝마다 부른다. [thrust, τx, τy, τz]를 돌려준다.

        Call every physics step; returns [thrust, τx, τy, τz].
        """
        if self.mode in ("idle", "stopped"):
            self._next_pos = self._next_att = t
            return np.zeros(4)
        if self.mode == "hl":
            self.sp = self.planner(t)
            if self.landing and self.planner.done(t) and st["p"][2] < 0.08:
                self.mode = "idle"
                self.landing = False
                return np.zeros(4)
        elif self.mode == "stream" and t - self._last_stream > STREAM_TIMEOUT:
            # 스트림이 끊겼으니 마지막 setpoint에서 제자리 비행
            self.planner.start(t, self.sp["p"], self.sp["p"], 0.0, self.sp["yaw"], self.sp["yaw"])
            self.mode = "hl"
            self._v_ff = np.zeros(3); self._stream_prev = None
            self.events.append((t, "stream_timeout_hold"))
        if t + 1e-9 >= self._next_pos:
            self._next_pos += self.pos_dt
            self.thrust, self.att_d = self.ctrl.position_update(st, self.sp, self.pos_dt)
            self.att_d = np.asarray(self.att_d, float)
        if t + 1e-9 >= self._next_att:
            self._next_att += self.att_dt
            self.tau = np.asarray(self.ctrl.attitude_update(st, self.att_d, self.att_dt), float)
        return np.array([float(self.thrust), *self.tau])
