"""2회차 배경 코드: 드론 한 대를 기울여서 옆으로 보내는 시험대.

수업 노트북(lesson.ipynb, 영어판 lesson_en.ipynb)이 `from background import *`로 불러 쓴다. 열어 볼 필요는 없다.
노트북에서 쓰는 세 부품(위치 제어기, 가속도→추력·자세, 기울기 제한)을 여기서 칩 하나로 묶고,
자세 제어기와 바람, 코스, 그래프는 여기서 맡는다. 화면 글자는 기본이 한국어이고, 영어 노트북은 첫 셀에서
set_language("en")을 불러 영어로 바꾼다.

Week-2 background: the notebook writes the position controller, the acceleration-to-attitude
conversion and the tilt limit; this file wires them into one chip and runs the flights.
Screen text is Korean by default; the English notebook calls set_language("en") in its first cell.
"""
from __future__ import annotations

from collections import deque

import numpy as np

from dsp.live import *  # noqa: F401,F403
from dsp.live import get_engine, filled, table, plt, COLORS, animate
from dsp.lang import tr as _t
from dsp.realtime import plotjuggler  # noqa: F401  노트북에서 PlotJuggler 창 열기
from dsp import config as C
from dsp.grading import step_metrics  # noqa: F401
from dsp.onboard import AttitudeController, LevelHold, drone_state
from dsp.ref.w1 import PID, AltitudeController, MY_GAINS as W1_GAINS  # 1회차에 만든 PID

G = 9.81
_eng = get_engine(1)
MASS = _eng.mass
INERTIA = _eng.inertia
MAX_TILT = np.radians(30)          # 칩이 허락하는 가장 큰 기울기
WIND = (0.010, 0.005, 0.0)         # 일정한 바람 [N], 무게의 약 3%
GUST = 0.004                       # 돌풍 세기 σ [N]
RING = np.array([[1.2 * np.cos(a), 1.2 * np.sin(a), 1.0 + 0.3 * (-1) ** k]
                 for k, a in enumerate(np.linspace(0, 2 * np.pi, 5, endpoint=False))])
CIRCLE_R, CIRCLE_T, CIRCLE_Z = 1.0, 8.0, 1.0
GOAL = {"hover": 0.05, "ring_time": 40.0, "ring_tilt": 35.0, "circle": 0.10}
DEFAULT_GAINS = dict(kp=6.0, kd=4.0, ki=2.0)


def warmup():
    """물리 엔진과 칩을 미리 한 번 돌려 둔다."""
    from dsp.limits import describe
    eng = get_engine(1)
    eng.reset(1, init_pos=[[0.0, 0.0, 1.0]])
    for _ in range(20):
        eng.step([[MASS * G, 0.0, 0.0, 0.0]])
    print(_t("시뮬레이터 준비 완료", "Simulator ready") + f" · {describe()}")


# 칩 조립 -------------------------------------------------------------------------------

class _Onboard:
    """노트북의 세 부품으로 만든 칩 제어기. dsp.onboard.Chip이 부른다."""

    def __init__(self, parts, gains, ff=True):
        self.pos = parts["PositionController"](**gains)
        self.acc_to = parts["acc_to_thrust_attitude"]
        self.limit = parts["limit_tilt"]
        self.att = AttitudeController(INERTIA)
        self.ff = ff
        self.max_tilt_cmd = 0.0

    def reset(self):
        self.pos.reset(); self.att.reset()

    def position_update(self, st, sp, dt):
        if not self.ff:
            sp = {**sp, "a": np.zeros(3)}
        acc = filled(self.pos.acceleration(st, sp, dt))
        acc = filled(self.limit(np.asarray(acc, float), MAX_TILT))
        thrust, att = filled(*self.acc_to(acc, sp["yaw"], MASS))
        return thrust, att

    def attitude_update(self, st, att_d, dt):
        return self.att.update(st, att_d, dt)


class MyChip:
    """노트북에서 만든 부품 세 개를 묶은 것. 비행 시험 함수에 넘긴다.

        chip = MyChip(PositionController, acc_to_thrust_attitude, limit_tilt)
        hover_in_wind(chip)
    """

    def __init__(self, PositionController, acc_to_thrust_attitude, limit_tilt):
        self.parts = dict(PositionController=PositionController,
                          acc_to_thrust_attitude=acc_to_thrust_attitude, limit_tilt=limit_tilt)

    def build(self, gains=None, ff=True):
        return _Onboard(self.parts, {**DEFAULT_GAINS, **(gains or {})}, ff)


def _world(onboard, seed=0, wind=WIND, gust=GUST, slot=""):
    from dsp.world import World
    eng = get_engine(1, wind=wind, gust=gust, seed=seed, slot=slot)
    return World([onboard], init_pos=[[0, 0, 0]], realism=C.Realism.ideal(), seed=seed, engine=eng)


def _tilt_deg(a):
    r, p = a["rpy"][:, 0, 0], a["rpy"][:, 0, 1]
    return np.degrees(np.arccos(np.clip(np.cos(r) * np.cos(p), -1, 1)))


# 1. 기울이면 옆으로 간다 ----------------------------------------------------------------

def tilt_test(sideways_acc, angles=(5, 10, 20), T=2.0, t_tilt=0.15):
    """1 m에 떠 있다가 앞으로 θ만큼 기울인 채 날린다. 높이는 지키도록 추력을 mg/cosθ로 준다.
    앞으로 가는 속도(실선)를 내 식의 가속도로 예상한 속도(점선)와 비교한다."""
    rows = []
    for deg in (0, 30, 45):
        th = np.radians(deg)
        got = float(filled(sideways_acc(th)))
        rows.append((f"θ = {deg}°", f"{got:.3f} m/s²", f"g·tanθ = {G * np.tan(th):.3f}", abs(got - G * np.tan(th)) < 1e-6))
    table(rows, header=_t(("기울기", "내 식", "답", ""), ("tilt", "my formula", "answer", "")))

    eng = get_engine(1)
    att = AttitudeController(INERTIA)
    fig, ax = plt.subplots(figsize=(8, 3.6))
    for k, deg in enumerate(angles):
        th = np.radians(deg)
        a_pred = float(filled(sideways_acc(th)))
        eng.reset(1, init_pos=[[0.0, 0.0, 1.0]]); att.reset()
        T_cmd = MASS * G / np.cos(th)
        ts, vs = [], []
        tau = np.zeros(3)
        for i in range(int(T * eng.freq)):
            st = drone_state(eng.true_state(), 0)
            if i % int(eng.freq / C.ATT_HZ) == 0:
                tau = att.update(st, np.array([0.0, th, 0.0]), 1 / C.ATT_HZ)
            eng.step([[T_cmd, *tau]])
            ts.append(i / eng.freq); vs.append(st["v"][0])
        ts, vs = np.array(ts), np.array(vs)
        c = COLORS[k % 6]
        ax.plot(ts, vs, color=c, lw=2.2, label=f"θ = {deg}° " + _t("시뮬레이터", "simulator"))
        ax.plot(ts, a_pred * np.maximum(ts - t_tilt, 0), color=c, lw=1.2, ls="--", label=f"θ = {deg}° " + _t("내 식", "my formula"))
    ax.set_ylim(0, None)
    ax.set_xlabel("t [s]"); ax.set_ylabel(_t("앞으로 가는 속도 vx [m/s]", "forward speed vx [m/s]"))
    ax.set_title(_t("앞으로 기울인 채 날리기 (점선: 공기 저항이 없을 때)", "Flying while tilted forward (dashed: without air drag)"))
    ax.legend(fontsize=8, ncol=3); plt.show()


# 2~4. 부품 확인 (손으로 푼 답과 비교) -----------------------------------------------------

def _st(p=(0, 0, 0), v=(0, 0, 0)):
    return {"p": np.array(p, float), "v": np.array(v, float)}


def _sp(p=(0, 0, 0), v=(0, 0, 0), a=(0, 0, 0), yaw=0.0):
    return {"p": np.array(p, float), "v": np.array(v, float), "a": np.array(a, float), "yaw": yaw}


def _fmt(v):
    return "(" + ", ".join(f"{x:+.2f}" for x in np.asarray(v, float)) + ")"


def check_acceleration(PositionController):
    """① 위치 제어기를 세 입력으로 시험한다 (ki = 0, 매번 새로 만들어서)."""
    cases = [(_t("목표가 +x로 1 m, 멈춰 있음", "target 1 m toward +x, at rest"), _st(), _sp(p=(1, 0, 0)), (6, 0, 0)),
             (_t("제자리인데 +y로 1 m/s 움직임", "on target but moving 1 m/s toward +y"), _st(v=(0, 1, 0)), _sp(), (0, -4, 0)),
             (_t("궤적이 위로 0.5 m/s² 를 알려 줌", "trajectory says 0.5 m/s² upward"), _st(), _sp(a=(0, 0, 0.5)), (0, 0, 0.5))]
    rows = []
    for name, st, sp, want in cases:
        pc = PositionController(kp=6.0, kd=4.0, ki=0.0)
        got = np.asarray(filled(pc.acceleration(st, sp, 0.01)), float)
        rows.append((name, _fmt(got), _fmt(want), bool(np.allclose(got, want, atol=1e-6))))
    table(rows, header=_t(("입력", "내 a [m/s²]", "손으로 푼 답", ""), ("input", "my a [m/s²]", "worked by hand", "")))


def _force_view(ax, h, v, up_label, h_label, flip, title):
    """한 방향에서 본 힘 그림: 버티는 몫(위), 가속하는 몫(옆), 둘을 합친 f, 그리고 그에 수직으로 누운 기체."""
    from matplotlib.patches import Arc
    lim = max(13.0, abs(h) + 4, v + 4)
    ax.axhline(0, color="#ccc", lw=0.8); ax.axvline(0, color="#ccc", lw=0.8)
    kw = dict(head_width=0.5, head_length=0.7, length_includes_head=True)
    ax.arrow(0, 0, 0, -G, color="#868e96", lw=1.5, **kw)
    ax.annotate(_t("무게 (-g)", "weight (-g)"), (0.3, -G + 0.8), color="#868e96", fontsize=9)
    ax.arrow(0, 0, 0, v, color="#2f9e44", lw=1.5, **kw)
    side = -1.0 if h >= 0 else 1.0                                 # 글자는 f 반대쪽에
    ax.annotate(up_label, (0.4 * side, v * 0.55), color="#2f9e44", fontsize=9,
                ha="right" if (side < 0) != flip else "left")
    if abs(h) > 0.05:
        ax.arrow(0, 0, h, 0, color="#e8590c", lw=1.5, **kw)
        ax.annotate(h_label, (h, -2.6), color="#e8590c", fontsize=9, ha="center")
        ax.plot([h, h], [0, v], color="#adb5bd", lw=0.8, ls="--"); ax.plot([0, h], [v, v], color="#adb5bd", lw=0.8, ls="--")
    ax.arrow(0, 0, h, v, color="#2a6fdb", lw=2.6, **kw)
    ax.annotate("f / m", (h * 1.04, v * 1.04 + 0.4), color="#2a6fdb", fontsize=11, weight="bold")
    n = np.hypot(h, v); body = np.array([v, -h]) / n * 2.6     # 기체는 f에 수직으로 눕는다
    ax.plot([-body[0], body[0]], [-body[1], body[1]], color="#212529", lw=5, solid_capstyle="round")
    ang = np.degrees(np.arctan2(h, v))
    if abs(ang) > 0.5:
        lo, hi = sorted((90.0, 90.0 - ang))
        ax.add_patch(Arc((0, 0), 14, 14, theta1=lo, theta2=hi, color="#ae3ec9", lw=1.6))
        mid = np.radians(90.0 - ang / 2)
        ax.annotate(f"{abs(ang):.0f}°", (8.3 * np.cos(mid), 8.3 * np.sin(mid)), color="#ae3ec9", fontsize=10,
                    ha="center", va="center")
    ax.set_xlim(-lim, lim); ax.set_ylim(-G - 2, lim); ax.set_aspect("equal")
    if flip:
        ax.invert_xaxis()
    ax.set_title(title, fontsize=10, linespacing=1.8)


def force_picture(acc=(5.0, 3.0, 0.0)):
    """② 그림: 원하는 가속도 acc [m/s²]를 내려면 기체를 어느 쪽으로 얼마나 눕혀야 하나 (yaw = 0).
    acc를 바꿔서 다시 실행해 보자. 화살표는 질량으로 나눈 힘(f/m, 단위 m/s²)이다."""
    a = np.asarray(acc, float)
    f = a + np.array([0.0, 0.0, G])
    pitch = np.arctan2(f[0], f[2])
    roll = np.arctan2(-f[1], np.hypot(f[0], f[2]))
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4.6))
    _force_view(ax1, f[0], f[2], _t("버티는 몫 g + a_z", "holding part g + a_z"), _t("가속하는 몫 a_x", "accelerating part a_x"), False,
                _t("옆에서 본 모습 (오른쪽이 앞 +x)", "side view (right = front, +x)") + f"\npitch = atan2(f_x, f_z) = {np.degrees(pitch):+.1f}°")
    _force_view(ax2, f[1], np.hypot(f[0], f[2]), r"$\sqrt{f_x^2 + f_z^2}$", _t("가속하는 몫 a_y", "accelerating part a_y"), True,
                _t("뒤에서 본 모습 (왼쪽이 +y)", "seen from behind (left = +y)") + f"\nroll = atan2($-f_y$, $\\sqrt{{f_x^2+f_z^2}}$) = {np.degrees(roll):+.1f}°")
    plt.tight_layout(); plt.show()
    T = MASS * np.linalg.norm(f)
    print(_t(f"a = {_fmt(a)} → 추력 T = m·|f/m| = {T:.3f} N (호버의 {T / (MASS * G):.2f}배) · ",
             f"a = {_fmt(a)} → thrust T = m·|f/m| = {T:.3f} N ({T / (MASS * G):.2f}× hover) · ")
          + f"roll {np.degrees(roll):+.1f}° · pitch {np.degrees(pitch):+.1f}°")
    if f[1] > 0.05:
        print(_t("왼쪽(+y)으로 가려면 기체 왼쪽을 내린다. x축 둘레 오른손 법칙으로는 이게 roll −라서 식에 −f_y가 들어간다.",
                 "To go left (+y) the drone lowers its left side. By the right-hand rule about x that is a negative roll, hence −f_y in the formula."))


def check_attitude(acc_to_thrust_attitude):
    """② 가속도 → 추력·자세를 네 입력으로 시험한다."""
    mg = MASS * G
    cases = [(_t("제자리 (a = 0)", "hover (a = 0)"), (0, 0, 0), 0.0, mg, (0, 0)),
             (_t("+x로 g만큼", "g toward +x"), (G, 0, 0), 0.0, np.sqrt(2) * mg, (0, 45)),
             (_t("+y로 g만큼", "g toward +y"), (0, G, 0), 0.0, np.sqrt(2) * mg, (-45, 0)),
             (_t("yaw 90°에서 +y로 g만큼", "yaw 90°, g toward +y"), (0, G, 0), np.pi / 2, np.sqrt(2) * mg, (0, 45))]
    rows = []
    for name, a, yaw, T_want, (r_want, p_want) in cases:
        T, att = filled(*acc_to_thrust_attitude(np.array(a, float), yaw, MASS))
        r, p = np.degrees(att[0]), np.degrees(att[1])
        ok = abs(T - T_want) < 1e-4 and abs(r - r_want) < 0.1 and abs(p - p_want) < 0.1
        rows.append((name, f"T {T:.3f} N · roll {r:+.1f}° · pitch {p:+.1f}°",
                     f"T {T_want:.3f} N · roll {r_want:+.0f}° · pitch {p_want:+.0f}°", ok))
    table(rows, header=_t(("입력", "내 결과", "답", ""), ("input", "my result", "answer", "")))
    T, att = acc_to_thrust_attitude(np.array([20.0, 0, 0]), 0.0, MASS)
    print(_t(f"a = (20, 0, 0)을 그대로 주면 pitch {np.degrees(att[1]):.1f}°. 이렇게 눕으면 높이를 못 지킨다 → ③",
             f"Passing a = (20, 0, 0) as is gives pitch {np.degrees(att[1]):.1f}°. Lying that flat cannot hold the height → ③"))


def check_tilt(limit_tilt, acc_to_thrust_attitude):
    """③ 기울기 제한을 시험한다."""
    big = np.asarray(filled(limit_tilt(np.array([20.0, 0.0, 0.0]), MAX_TILT)), float)
    _, att = acc_to_thrust_attitude(big, 0.0, MASS)
    small = np.asarray(filled(limit_tilt(np.array([1.0, 0.5, 0.2]), MAX_TILT)), float)
    rows = [(_t("a = (20, 0, 0) → 제한 후", "a = (20, 0, 0) → limited"), f"{_fmt(big)} · pitch {np.degrees(att[1]):.1f}°",
             _t("pitch 30°, a_z = 0 그대로", "pitch 30°, a_z = 0 unchanged"),
             abs(np.degrees(att[1]) - 30) < 0.1 and abs(big[2]) < 1e-9),
            (_t("a = (1, 0.5, 0.2)는 작으니 그대로", "a = (1, 0.5, 0.2) is small, unchanged"), _fmt(small), _fmt((1, 0.5, 0.2)), bool(np.allclose(small, [1, 0.5, 0.2])))]
    table(rows, header=_t(("입력", "내 결과", "답", ""), ("input", "my result", "answer", "")))


# 5. 첫 비행: 바람 속 호버 ---------------------------------------------------------------

def _hover(chip, gains=None, T=12.0, seed=0):
    w = _world(chip.build(gains), seed)

    def gs(w, t):
        if t < 1e-6:
            w.link["cf1"].takeoff(1.0, 2.0)
        elif t > 3.0:
            w.link["cf1"].cmd_position([0, 0, 1.0], 0.0)

    w.run(T, ground_station=gs)
    a = w.arrays(); w.close()
    m = a["t"] >= 5.0
    err = a["p"][m, 0] - np.array([0, 0, 1.0])
    return a, err, float(np.sqrt(np.mean(np.sum(err ** 2, 1))))


def hover_in_wind(chip, **cases):
    """바람(무게의 약 3%)과 돌풍 속에서 (0, 0, 1)에 떠 있기. 5초부터 위치 오차를 잰다.

        hover_in_wind(chip)                                   # 기본 게인
        hover_in_wind(chip, 기본={}, PD만={"ki": 0})           # 여러 게인 비교 (영어판: default={}, PD_only={"ki": 0})
    """
    cases = cases or {_t("기본 (kp 6, kd 4, ki 2)", "default (kp 6, kd 4, ki 2)"): {}}
    fig, (ax, az) = plt.subplots(1, 2, figsize=(10, 4.2), gridspec_kw={"width_ratios": [1, 1.5]})
    rows = []
    for k, (name, g) in enumerate(cases.items()):
        a, err, rms = _hover(chip, g)
        c = COLORS[k % 6]
        ax.plot(100 * err[:, 0], 100 * err[:, 1], color=c, lw=1, alpha=0.8, label=name)
        ax.plot(100 * err[:, 0].mean(), 100 * err[:, 1].mean(), "o", color=c, ms=7)
        az.plot(a["t"], 100 * np.linalg.norm(a["p"][:, 0, :2], axis=1), color=c, lw=1.5, label=name)
        rows.append((name, f"{100 * rms:.2f} cm", f"< {100 * GOAL['hover']:.0f} cm", rms < GOAL["hover"]))
    ax.add_patch(plt.Circle((0, 0), 100 * GOAL["hover"], color="#2f9e44", alpha=0.12))
    ax.annotate(_t("바람은 +x, +y 쪽으로 분다", "wind blows toward +x, +y"), (-7.5, 7.0), fontsize=8, color="#555")
    ax.set_xlim(-8, 8); ax.set_ylim(-8, 8); ax.set_aspect("equal")
    ax.set_xlabel(_t("x 오차 [cm]", "x error [cm]")); ax.set_ylabel(_t("y 오차 [cm]", "y error [cm]"))
    ax.set_title(_t("위에서 본 위치 오차 (초록: 5 cm)", "Position error from above (green: 5 cm)"))
    ax.legend(fontsize=8, loc="lower left")
    az.axvline(5.0, color="k", lw=0.8, ls=":"); az.axhline(100 * GOAL["hover"], color="#2f9e44", lw=1, ls="--")
    az.set_ylim(0, 15); az.set_xlabel("t [s]"); az.set_ylabel(_t("목표에서 수평으로 떨어진 거리 [cm]", "horizontal distance from target [cm]"))
    az.set_title(_t("시간에 따라 (점선 뒤부터 잰다)", "Over time (measured after the dotted line)"))
    plt.tight_layout(); plt.show()
    table(rows, header=_t(("게인", "위치 오차 RMS", "기준", ""), ("gains", "position error RMS", "limit", "")))


# 6. 웨이포인트: 링 코스 ------------------------------------------------------------------

def _ring(chip, WaypointFollower, gains=None, T=40.0, seed=0):
    w = _world(chip.build(gains), seed)
    fol = WaypointFollower(RING, radius=0.15)
    done_t = [None]

    def gs(w, t):
        cf = w.link["cf1"]
        if t < 1e-6:
            cf.takeoff(1.0, 2.0)
        elif t > 2.5:
            target = filled(fol.update(w.tracker.positions()[0]))
            cf.cmd_position(np.asarray(target, float), 0.0)
            if fol.done and done_t[0] is None:
                done_t[0] = t

    w.run(T, ground_station=gs, until=lambda w: done_t[0] is not None)
    a = w.arrays(); w.close()
    return a, done_t[0], fol


def _ring_axes(ax):
    for k, p in enumerate(RING):
        ax.add_patch(plt.Circle(p[:2], 0.15, color="#f59f00", alpha=0.25))
        ax.annotate(str(k + 1), p[:2], xytext=(6, 6), textcoords="offset points", fontsize=10)
    ax.set_xlim(-1.7, 1.7); ax.set_ylim(-1.7, 1.7); ax.set_aspect("equal")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")


def ring_course(chip, WaypointFollower, gains=None, video=True):
    """웨이포인트 5개짜리 링 코스 (도착 반경 15 cm, 바람 있음). 위에서 본 경로와 영상을 보여 준다."""
    a, t_done, fol = _ring(chip, WaypointFollower, gains)
    tilt = _tilt_deg(a)
    fig, (ax, az) = plt.subplots(1, 2, figsize=(10, 4.2), gridspec_kw={"width_ratios": [1, 1.4]})
    _ring_axes(ax)
    ax.plot(a["p"][:, 0, 0], a["p"][:, 0, 1], color=COLORS[0], lw=1.6)
    ax.set_title(_t("위에서 본 경로 (주황: 도착 반경 15 cm)", "Path from above (orange: arrival radius 15 cm)"))
    az.plot(a["t"], tilt, color=COLORS[1], lw=1.4)
    az.axhline(30, color="k", ls=":", lw=1, label=_t("명령 한계 30°", "command limit 30°"))
    az.axhline(GOAL["ring_tilt"], color="#d62728", ls="--", lw=1, label=_t("기준 35°", "limit 35°"))
    az.set_xlabel("t [s]"); az.set_ylabel(_t("기울기 [°]", "tilt [°]")); az.set_title(_t("기울기", "Tilt")); az.legend(fontsize=8)
    plt.tight_layout(); plt.show()
    ok_done = t_done is not None and t_done < GOAL["ring_time"]
    table([(_t("웨이포인트", "waypoints"), f"{min(fol.i, 5)}/5" + (_t(f" · {t_done:.1f} s에 완주", f" · done at {t_done:.1f} s") if t_done else ""),
            _t(f"5/5, {GOAL['ring_time']:.0f} s 안", f"5/5 within {GOAL['ring_time']:.0f} s"), ok_done),
           (_t("최대 기울기", "max tilt"), f"{tilt.max():.1f}°", f"≤ {GOAL['ring_tilt']:.0f}°", tilt.max() <= GOAL["ring_tilt"])])
    if video:
        from IPython.display import display
        display(animate(a["t"], a["p"], _t("링 코스", "Ring course"), targets=RING, lim=1.7, speed=1.5))


# 7. 원 궤적과 앞먹임 ----------------------------------------------------------------------

class CircleTraj:
    """칩에 올리는 원 궤적. 위치뿐 아니라 그 순간의 속도와 가속도도 함께 준다."""

    def __init__(self, t0, r=CIRCLE_R, period=CIRCLE_T, z=CIRCLE_Z, laps=2):
        self.t0, self.r, self.w, self.z, self.T = t0, r, 2 * np.pi / period, z, laps * period

    def done(self, t):
        return t - self.t0 >= self.T

    def __call__(self, t):
        s = min(max(t - self.t0, 0.0), self.T)
        c, sn = np.cos(self.w * s), np.sin(self.w * s)
        r, w = self.r, self.w
        moving = 0.0 < t - self.t0 < self.T
        return {"p": np.array([r * c, r * sn, self.z]),
                "v": np.array([-r * w * sn, r * w * c, 0.0]) * moving,
                "a": np.array([-r * w * w * c, -r * w * w * sn, 0.0]) * moving, "yaw": 0.0}


def _circle(chip, gains=None, ff=True, seed=0):
    w = _world(chip.build(gains, ff=ff), seed)
    t_start = 6.0
    traj = [None]

    def gs(w, t):
        cf = w.link["cf1"]
        if t < 1e-6:
            cf.takeoff(1.0, 2.0)
        elif abs(t - 2.5) < 1e-6:
            cf.go_to([CIRCLE_R, 0.0, CIRCLE_Z], 0.0, 3.0)
        elif t >= t_start and traj[0] is None:
            traj[0] = CircleTraj(t)
            chip_ = w.chips[0]
            chip_.planner, chip_.mode = traj[0], "hl"      # 궤적을 칩에 올린다

    w.run(t_start + 2 * CIRCLE_T + 1.0, ground_station=gs)
    a = w.arrays(); w.close()
    trj = traj[0]
    m = (a["t"] >= trj.t0 + 1.0) & (a["t"] <= trj.t0 + trj.T)
    ref = np.array([trj(t)["p"] for t in a["t"][m]])
    err = a["p"][m, 0] - ref
    return a, m, float(np.sqrt(np.mean(np.sum(err ** 2, 1))))


def circle(chip, gains=None, compare_ff=True):
    """반지름 1 m 원을 한 바퀴 8초로 두 바퀴 (바람 있음). compare_ff=True면 앞먹임을 끈 비행과 비교한다."""
    runs = [(_t("앞먹임 켬 (궤적의 a를 더함)", "feed-forward on (adds the trajectory's a)"), True)] + \
        ([(_t("앞먹임 끔 (a = 0)", "feed-forward off (a = 0)"), False)] if compare_ff else [])
    fig, ax = plt.subplots(figsize=(5, 5))
    th = np.linspace(0, 2 * np.pi, 200)
    ax.plot(CIRCLE_R * np.cos(th), CIRCLE_R * np.sin(th), "k--", lw=1, label=_t("목표 원", "target circle"))
    rows = []
    for k, (name, ff) in enumerate(runs):
        a, m, rms = _circle(chip, gains, ff)
        ax.plot(a["p"][m, 0, 0], a["p"][m, 0, 1], color=COLORS[k], lw=1.5, label=name)
        rows.append((name, f"{100 * rms:.2f} cm", f"< {100 * GOAL['circle']:.0f} cm", rms < GOAL["circle"]))
    ax.set_aspect("equal"); ax.set_xlim(-1.4, 1.4); ax.set_ylim(-1.4, 1.4)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_title(_t("원 궤적 (위에서 본 모습)", "Circle trajectory (from above)")); ax.legend(fontsize=8, loc="lower left")
    plt.show()
    table(rows, header=_t(("", "원에서 벗어난 거리 RMS", "기준", ""), ("", "distance off the circle RMS", "limit", "")))


# 8. 미션 ----------------------------------------------------------------------------

def flight_tests(chip, WaypointFollower, gains=None):
    """오늘의 미션: 세 비행 시험을 내 칩으로 통과한다 (기본 게인 kp 6, kd 4, ki 2)."""
    _, _, h = _hover(chip, gains)
    a, t_done, fol = _ring(chip, WaypointFollower, gains)
    tilt = _tilt_deg(a).max()
    _, _, c = _circle(chip, gains)
    rows = [(_t("바람 속 호버 위치 오차 RMS", "hover in wind: position error RMS"), f"{100 * h:.2f} cm", "< 5 cm", h < GOAL["hover"]),
            (_t("링 코스 완주", "ring course finished"), f"{min(fol.i, 5)}/5" + (f" · {t_done:.1f} s" if t_done else ""),
             _t("5/5, 40 s 안", "5/5 within 40 s"), t_done is not None and t_done < GOAL["ring_time"]),
            (_t("링 코스 최대 기울기", "ring course max tilt"), f"{tilt:.1f}°", "≤ 35°", tilt <= GOAL["ring_tilt"]),
            (_t("원 궤적 RMS", "circle trajectory RMS"), f"{100 * c:.2f} cm", "< 10 cm", c < GOAL["circle"])]
    table(rows)
    print(_t("미션 성공! 네 칸 모두 통과했어요.", "Mission complete! All four boxes passed.") if all(r[3] for r in rows)
          else _t("아직 빨간 칸이 있어요. 위로 올라가 해당 단계를 다시 보세요.", "Some boxes are still red. Scroll up and revisit that stage."))


def tune(chip, WaypointFollower, test="hover"):
    """슬라이더로 위치 게인을 바꾸면 고른 시험을 바로 다시 날린다. test는 "hover", "ring", "circle"."""
    import ipywidgets as W
    from IPython.display import display

    sl = dict(kp=W.FloatSlider(value=6, min=0.5, max=15, step=0.5, description="kp"),
              kd=W.FloatSlider(value=4, min=0, max=10, step=0.5, description="kd"),
              ki=W.FloatSlider(value=2, min=0, max=6, step=0.5, description="ki"))
    pick = W.ToggleButtons(options=[(_t("바람 속 호버", "hover in wind"), "hover"), (_t("링 코스", "ring course"), "ring"),
                                     (_t("원", "circle"), "circle")], value=test)
    for s in sl.values():
        s.continuous_update = False
    out = W.Output()

    def go(_=None):
        g = {k: s.value for k, s in sl.items()}
        with out:
            out.clear_output(wait=True)
            if pick.value == "hover":
                hover_in_wind(chip, **{f"kp {g['kp']}, kd {g['kd']}, ki {g['ki']}": g})
            elif pick.value == "ring":
                ring_course(chip, WaypointFollower, g, video=False)
            else:
                circle(chip, g, compare_ff=False)

    for s in list(sl.values()) + [pick]:
        s.observe(go, names="value")
    display(pick, W.HBox(list(sl.values())), out)
    go()



# 실시간 비행 ---------------------------------------------------------------------------

def live(chip, gains=None, window=False, duration=180.0):
    """1배속 실시간 비행. 비행 중에 목표 위치, 게인, 바람을 슬라이더로 바꾸면 바로 반영된다.

    "원 그리기"를 켜면 목표가 반지름 0.8 m 원을 따라 돈다 (위치만 주니 조금씩 늦게 따라온다).
    노트북 그래프와 PlotJuggler(/pos/...)에 위치, 목표, 기울기, 추력, 오차가 실시간으로 나온다.
    """
    from dsp.realtime import LiveSession, window_viewer
    g0 = {**DEFAULT_GAINS, **(gains or {})}
    ob = chip.build(g0)

    def fly(s):
        w = _world(ob, 0, wind=WIND, gust=GUST, slot="live")
        state = {"t_circle": None}

        def gs(w, t):
            v = s.values
            ob.pos.pid.kp, ob.pos.pid.ki, ob.pos.kd = v["kp"], v["ki"], v["kd"]
            w.engine.wind = np.array([v["wind"], 0.5 * v["wind"], 0.0])
            if t < 1e-6:
                w.link["cf1"].takeoff(1.0, 2.0)
                return
            target = np.array([v["x"], v["y"], v["z"]])
            if v["circle"]:
                t_c = state["t_circle"] = state["t_circle"] if state["t_circle"] is not None else t
                ang = 2 * np.pi * (t - t_c) / 6.0
                target = target + 0.8 * np.array([np.cos(ang) - 1.0, np.sin(ang), 0.0])
            else:
                state["t_circle"] = None
            if t > 2.5:
                w.link["cf1"].cmd_position(target, 0.0)
            L = w.log
            if L["t"]:
                p, rpy, th = L["p"][-1][0], L["rpy"][-1][0], float(L["thrust"][-1][0])
                sp = w.chips[0].sp["p"] if w.chips[0].sp is not None else target
                tilt = float(np.degrees(np.arccos(np.clip(np.cos(rpy[0]) * np.cos(rpy[1]), -1, 1))))
                s.emit(t, p=p, sp=sp, tilt=tilt, thrust=th, err_cm=100 * float(np.linalg.norm(p - sp)))

        try:
            w.run(duration, ground_station=gs, realtime=True, viewer=window_viewer(window), until=lambda w: s.stopped)
        finally:
            w.close()

    return LiveSession(
        fly, group="pos", duration=duration,
        sliders={"x": (0.0, -1.5, 1.5, 0.1), "y": (0.0, -1.5, 1.5, 0.1), "z": (1.0, 0.3, 2.0, 0.1),
                 "kp": (g0["kp"], 0, 20, 0.5), "kd": (g0["kd"], 0, 12, 0.5), "ki": (g0["ki"], 0, 8, 0.5),
                 "wind": (WIND[0], 0.0, 0.04, 0.002)},
        checkboxes={"circle": False},
        labels=_t({"x": "목표 x [m]", "y": "목표 y [m]", "z": "목표 z [m]", "wind": "바람 [N]", "circle": "원 그리기",
                   "p_x": "x", "p_y": "y", "p_z": "z", "sp_x": "목표 x", "sp_y": "목표 y", "sp_z": "목표 z",
                   "tilt": "기울기", "err_cm": "목표와 떨어진 거리"},
                  {"x": "target x [m]", "y": "target y [m]", "z": "target z [m]", "wind": "wind [N]", "circle": "fly a circle",
                   "p_x": "x", "p_y": "y", "p_z": "z", "sp_x": "target x", "sp_y": "target y", "sp_z": "target z",
                   "tilt": "tilt", "err_cm": "distance from target"}),
        panels=[(_t("가로 위치 (점선이 목표)", "horizontal position (dashed = target)"), ["p_x", "sp_x", "p_y", "sp_y"], "m"),
                (_t("높이", "height"), ["p_z", "sp_z"], "m"),
                (_t("기울기: 옆으로 가거나 바람을 버틸 때 기운다 (칩 제한 30°)", "tilt: leans to move sideways or to hold against wind (chip limit 30°)"), ["tilt"], "°"),
                (_t("목표와 떨어진 거리", "distance from target"), ["err_cm"], "cm")],
    ).start()


# 심화: 칼만 필터 ---------------------------------------------------------------------------

IMU_NOISE, IMU_BIAS = 0.20, 0.10           # 가속도계 잡음 σ, 바이어스 [m/s²]
MOCAP_NOISE, MOCAP_DELAY, MOCAP_HZ = 0.005, 0.020, 100


def _altitude_flight(source, kf=None, seed=0, T=10.0, z0=0.5, z1=1.0, t_step=5.0):
    """고도만 제어하는 비행. 제어기에 넣는 고도를 source("truth", "mocap", "kf")로 고른다."""
    rng = np.random.default_rng(300 + seed)
    eng = get_engine(1)
    eng.reset(1, init_pos=[[0.0, 0.0, z0]])
    ctrl = AltitudeController(MASS, **W1_GAINS)
    level = LevelHold(INERTIA)
    dt = eng.dt
    every = int(round(eng.freq / 100))
    cam_every = int(round(eng.freq / MOCAP_HZ))
    queue = deque()
    z_cam, thrust, vz_prev = z0, MASS * G, 0.0
    log = {k: [] for k in ("t", "z_true", "z_used", "z_cam", "thrust")}
    for k in range(int(T * eng.freq)):
        t = k * dt
        st = eng.true_state()
        z_true, vz = st["p"][0, 2], st["v"][0, 2]
        a_imu = (vz - vz_prev) / dt + IMU_BIAS + rng.normal(0.0, IMU_NOISE)   # 중력을 뺀 IMU 값
        vz_prev = vz
        if kf is not None and k > 0:
            kf.predict(a_imu, dt)
        if k % cam_every == 0:
            queue.append((t + MOCAP_DELAY, z_true + rng.normal(0.0, MOCAP_NOISE)))
        while queue and queue[0][0] <= t + 1e-9:
            z_cam = queue.popleft()[1]
            if kf is not None:
                kf.update(z_cam)
        if k % every == 0:
            z_used = {"truth": z_true, "mocap": z_cam, "kf": kf.z if kf is not None else z_cam}[source]
            thrust = float(ctrl.update(z0 if t < t_step else z1, z_used, every * dt))
            log["t"].append(t); log["z_true"].append(z_true); log["z_used"].append(z_used)
            log["z_cam"].append(z_cam); log["thrust"].append(min(max(thrust, 0.0), eng.max_thrust))
        tau = level(drone_state(st, 0), np.zeros(3))
        eng.step(np.array([[thrust, *tau]]))
    out = {k: np.asarray(v) for k, v in log.items()}
    out.update(z0=z0, z1=z1, t_step=t_step)
    return out


def kf_test(AltitudeKF):
    """고도를 무엇으로 재느냐에 따라 비행이 어떻게 달라지는지: 참값, 카메라 그대로, 칼만 필터."""
    runs = [(_t("참값 (시뮬레이터만 아는 값)", "truth (only the simulator knows it)"), "truth", None),
            (_t("천장 카메라 그대로", "ceiling camera as is"), "mocap", None),
            (_t("칼만 필터 (IMU + 카메라)", "Kalman filter (IMU + camera)"), "kf", "kf")]
    fig, (ax, af) = plt.subplots(1, 2, figsize=(11, 3.8))
    rows = []
    for k, (name, src, use_kf) in enumerate(runs):
        kf = AltitudeKF(z0=0.5) if use_kf else None
        a = _altitude_flight(src, kf)
        m = (a["t"] >= 2.0) & (a["t"] < a["t_step"])
        rms = float(np.sqrt(np.mean((a["z_true"][m] - a["z0"]) ** 2)))
        jitter = float(np.std(np.diff(a["thrust"][m])))
        ax.plot(a["t"], a["z_true"], color=COLORS[k], lw=1.5, label=name)
        af.plot(a["t"], a["thrust"], color=COLORS[k], lw=0.8, alpha=0.8 if src == "mocap" else 1.0,
                zorder=1 if src == "mocap" else 3, label=name)
        rows.append((name, f"{100 * rms:.2f} cm", f"{jitter:.3f} N", None))
    ax.set_xlabel("t [s]"); ax.set_ylabel(_t("실제 고도 [m]", "true altitude [m]")); ax.legend(fontsize=8); ax.set_title(_t("고도", "Altitude"))
    af.set_xlabel("t [s]"); af.set_ylabel(_t("추력 [N]", "thrust [N]")); af.set_title(_t("모터에 준 추력 (떨림을 보자)", "Thrust sent to the motors (watch the jitter)")); af.set_ylim(0.2, 0.55)
    plt.tight_layout(); plt.show()
    table(rows, header=_t(("고도를 잰 방법", "호버 고도 오차 RMS", "추력 떨림 σ", ""), ("altitude source", "hover altitude error RMS", "thrust jitter σ", "")))


def check_kf(AltitudeKF):
    """⑤⑥⑦ 칼만 필터를 손으로 푼 값과 비교한다."""
    kf = AltitudeKF(z0=0.0)
    kf.predict(1.0, 0.1)
    p_ok = np.allclose(kf.x, [0.005, 0.1])
    kf2 = AltitudeKF(z0=0.0)
    kf2.P = np.diag([1e-4, 1e-2]); kf2.r = 1e-4
    kf2.update(0.02)
    u_ok = abs(kf2.z - 0.01) < 1e-9
    table([(_t("⑤ 예측: z=0, v=0에서 a=1 m/s²로 0.1초", "⑤ predict: from z=0, v=0 with a=1 m/s² for 0.1 s"),
            f"z {kf.x[0]:.4f} m · v {kf.x[1]:.3f} m/s", "z 0.0050 · v 0.100", p_ok),
           (_t("⑥⑦ 보정: 내 믿음과 카메라 믿음이 같을 때 (P = r)", "⑥⑦ correct: when my belief and the camera's are equal (P = r)"),
            f"z {kf2.z:.4f} m", _t("가운데 0.0100 m", "halfway 0.0100 m"), u_ok)])
