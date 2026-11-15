"""1회차 배경 코드: 드론 한 대를 위아래로만 띄워 보는 시험대.

수업 노트북(lesson.ipynb, 영어판 lesson_en.ipynb)이 `from background import *`로 불러 쓴다. 열어 볼 필요는 없다.
기체가 기울지 않게 잡는 일(LevelHold)과 물리 엔진은 여기서 맡고, 노트북에서는
추력을 정하는 코드만 쓴다. 화면에 나오는 글자는 한국어가 기본이고, 영어 노트북은 첫 셀에서
set_language("en")을 불러 영어로 바꾼다.

Week-1 background: a test stand that flies one drone up and down. The notebook
only decides the thrust; levelling and physics are handled here. Text is Korean
by default; the English notebook calls set_language("en") in its first cell.
"""
from __future__ import annotations

import numpy as np

from dsp.live import *  # noqa: F401,F403
from dsp.live import get_engine, filled, table, plt, COLORS
from dsp.lang import tr as _t
from dsp.onboard import LevelHold, drone_state
from dsp.grading import step_metrics
from dsp.realtime import plotjuggler  # noqa: F401  노트북에서 PlotJuggler 창 열기

G = 9.81
_eng = get_engine(1)
MASS = _eng.mass                 # 컨트롤러가 아는 모델 질량 [kg]
MAX_THRUST = _eng.max_thrust     # 모터 넷을 끝까지 돌렸을 때 [N]
DT = 0.01                        # 고도 루프 주기 [s] (100 Hz)
Z0, Z1, T_STEP, T_END = 0.5, 1.5, 4.0, 10.0
GOAL = {"overshoot": 10.0, "settle": 2.0, "sse": 0.02}


def drone_info():
    """오늘 띄울 기체의 숫자들."""
    ratio = MAX_THRUST / (MASS * G)
    table([
        (_t("기체", "drone"), _t("cf2x_T350 (Crazyflie급)", "cf2x_T350 (Crazyflie class)"), "", None),
        (_t("질량 m", "mass m"), f"{MASS * 1000:.1f} g", "", None),
        (_t("최대 추력", "max thrust"), f"{MAX_THRUST:.2f} N", _t(f"무게의 {ratio:.1f}배", f"{ratio:.1f} × weight"), None),
        (_t("고도 루프", "altitude loop"), f"{1 / DT:.0f} Hz",
         _t("1초에 100번 추력을 새로 정한다", "thrust chosen 100 times a second"), None),
    ], header=("", _t("값", "value"), _t("메모", "note"), ""))


def _fly(thrust_fn, mass_scale=1.0, z0=Z0, z1=Z1, t_step=T_STEP, t_end=T_END):
    """z0에 떠 있다가 t_step에 목표가 z1로 바뀌는 비행. thrust_fn(z_target, z, dt) -> 추력 [N]."""
    eng = get_engine(1, mass_scale)
    eng.reset(1, init_pos=[[0.0, 0.0, z0]])
    level = LevelHold(eng.inertia)
    every = int(round(eng.freq * DT))
    n = int(round(t_end * eng.freq))
    t_log, z_log, f_log = [], [], []
    thrust = 0.0
    for k in range(n):
        st = eng.true_state()
        ds = drone_state(st, 0)
        t = k / eng.freq
        if k % every == 0:
            z_ref = z0 if t < t_step else z1
            thrust = float(filled(thrust_fn(z_ref, ds["p"][2], DT)))
            if not np.isfinite(thrust):
                raise ValueError(_t(f"추력이 {thrust} 이에요. 계산식을 확인하세요.", f"Thrust is {thrust}. Check your formula."))
            t_log.append(t); z_log.append(ds["p"][2]); f_log.append(min(max(thrust, 0.0), MAX_THRUST))
        tau = level(ds, np.zeros(3))
        eng.step([[thrust, *tau]])
    return np.array(t_log), np.array(z_log), np.array(f_log)


def _target(t, z0=Z0, z1=Z1, t_step=T_STEP):
    return np.where(t < t_step, z0, z1)


# 1. 호버 추력만으로 -----------------------------------------------------------------

def fly_open_loop(hover_thrust):
    """0.5 m에서 호버 추력만 계속 줄 때. 질량이 맞을 때, 10% 무거울 때, 10% 가벼울 때를 같이 그린다."""
    T = filled(hover_thrust(MASS))
    print(f"hover_thrust({MASS:.4f}) = {T:.4f} N")
    fig, ax = plt.subplots()
    for ms, label, c in ((1.0, _t("질량이 모델과 같다", "mass equals the model"), COLORS[0]),
                         (1.1, _t("실제가 10% 무겁다", "really 10% heavier"), COLORS[1]),
                         (0.9, _t("실제가 10% 가볍다", "really 10% lighter"), COLORS[2])):
        t, z, _ = _fly(lambda zr, z, dt: hover_thrust(MASS), ms, t_step=99, t_end=3.0)
        ax.plot(t, z, color=c, lw=2, label=label)
    ax.axhline(Z0, color="k", ls=":", lw=1)
    ax.set_xlabel("t [s]"); ax.set_ylabel(_t("고도 z [m]", "height z [m]")); ax.set_ylim(-0.05, 1.6)
    ax.set_title(_t("호버 추력만 계속 줄 때 (높이를 보지 않는다)", "Hover thrust only (never looks at the height)"))
    ax.legend(loc="upper left")
    plt.show()


# 2~5. 계단 시험 -----------------------------------------------------------------------

def step_test(pid, altitude_thrust, heavy=True, title=None):
    """0.5 m에 떠 있다가 4초에 목표를 1.5 m로 올리는 시험. heavy=True면 실제 질량이 10% 무겁다."""
    r = _run(pid, altitude_thrust, heavy)
    fig, (ax, af) = plt.subplots(1, 2, figsize=(10, 3.6), gridspec_kw={"width_ratios": [1.6, 1]})
    mine = _t("내 제어기", "my controller")
    _draw_step(ax, r, COLORS[0], mine)
    af.plot(r["t"], r["thrust"], color=COLORS[0], lw=1.5)
    af.axhline(MASS * G, color="k", ls=":", lw=1, label=_t("호버 추력 mg", "hover thrust mg"))
    af.axhline(MAX_THRUST, color="#d62728", ls="--", lw=1, label=_t("최대 추력", "max thrust"))
    af.set_xlabel("t [s]"); af.set_ylabel(_t("추력 [N]", "thrust [N]"))
    af.set_title(_t("모터에 준 추력", "Thrust given to the motors")); af.legend(fontsize=8)
    ax.set_title(title or _t("계단 시험", "Step test") + f" ({_drone(heavy)})")
    plt.tight_layout(); plt.show()
    _metrics_table([(mine, r)])


def compare(pids, altitude_thrust, heavy=True, title=None):
    """{"이름": PID(...), ...}를 같은 시험으로 날려서 한 그래프에 겹쳐 그린다."""
    runs = [(name, _run(pid, altitude_thrust, heavy)) for name, pid in pids.items()]
    fig, ax = plt.subplots(figsize=(9, 3.8))
    for k, (name, r) in enumerate(runs):
        _draw_step(ax, r, COLORS[k % 6], name, band=(k == 0))
    ax.set_title(title or _t("비교", "Comparison") + f" ({_drone(heavy)})")
    plt.show()
    _metrics_table(runs)


def mission(PID, gains, altitude_thrust):
    """오늘의 미션. 질량이 맞는 기체와 10% 무거운 기체 둘 다 기준을 넘으면 성공."""
    runs = [(_t("질량이 맞는 기체", "right mass"), _run(PID(**gains), altitude_thrust, False)),
            (_t("10% 무거운 기체", "10% heavy"), _run(PID(**gains), altitude_thrust, True))]
    fig, ax = plt.subplots(figsize=(9, 3.8))
    for k, (name, r) in enumerate(runs):
        _draw_step(ax, r, COLORS[k], name, band=(k == 0))
    ax.set_title(_t("미션: 0.5 m에서 1.5 m로, 지나치지 않고 빨리 서기   ",
                    "Mission: 0.5 m → 1.5 m, settle fast without overshooting   ") + ", ".join(f"{k}={v}" for k, v in gains.items()))
    plt.show()
    rows, ok_all = [], True
    for name, r in runs:
        m = r["m"]
        checks = [(_t("오버슈트", "overshoot"), f"{m['overshoot_pct']:.1f} %", f"< {GOAL['overshoot']:.0f} %",
                   m["overshoot_pct"] < GOAL["overshoot"]),
                  (_t("2% 정착 시간", "2% settling time"), f"{m['settle_s']:.2f} s", f"< {GOAL['settle']:.0f} s",
                   m["settle_s"] < GOAL["settle"]),
                  (_t("마지막 1초 오차", "last-second error"), f"{100 * m['ss_err_m']:.1f} cm", f"< {100 * GOAL['sse']:.0f} cm",
                   m["ss_err_m"] < GOAL["sse"])]
        rows += [(f"{name} · {c[0]}", *c[1:]) for c in checks]
        ok_all &= all(c[3] for c in checks)
    table(rows)
    print(_t("미션 성공! 여섯 칸 모두 통과했어요.", "Mission passed! All six boxes are green.") if ok_all else
          _t("아직 빨간 칸이 있어요. 게인을 바꿔 다시 돌려 보세요.", "Some boxes are still red. Change the gains and run it again."))


def tune(PID, altitude_thrust, kp=8.0, ki=0.0, kd=2.0, i_band=0.15):
    """슬라이더로 게인을 바꾸면 바로 미션을 다시 날린다 (ipywidgets 필요)."""
    import ipywidgets as W
    from IPython.display import display

    sl = dict(kp=W.FloatSlider(value=kp, min=0, max=30, step=0.5, description="kp"),
              ki=W.FloatSlider(value=ki, min=0, max=15, step=0.5, description="ki"),
              kd=W.FloatSlider(value=kd, min=0, max=12, step=0.5, description="kd"))
    for s in sl.values():
        s.continuous_update = False
    out = W.Output()

    def go(_=None):
        with out:
            out.clear_output(wait=True)
            mission(PID, dict(kp=sl["kp"].value, ki=sl["ki"].value, kd=sl["kd"].value, i_band=i_band), altitude_thrust)

    for s in sl.values():
        s.observe(go, names="value")
    display(W.HBox(list(sl.values())), out)
    go()


# 실시간 비행 ---------------------------------------------------------------------------

def live(PID, altitude_thrust, kp=8.0, ki=2.0, kd=4.0, i_band=0.15, heavy=True, window=False, duration=120.0):
    """1배속 실시간 비행. 비행 중에 목표 높이와 kp, ki, kd 슬라이더를 움직이면 바로 반영된다.

    노트북 그래프와 PlotJuggler(/alt/...)에 높이, 목표, 오차, P·I·D 각 항, 추력이 실시간으로 나온다.
    window=True면 MuJoCo 3D 창도 띄운다.
    """
    from dsp.realtime import LiveSession
    pid = PID(kp=kp, ki=ki, kd=kd, i_band=i_band)
    eng = get_engine(1, 1.1 if heavy else 1.0, slot="live")
    eng.reset(1, init_pos=[[0.0, 0.0, Z0]])
    level = LevelHold(eng.inertia)
    every = int(round(eng.freq * DT))

    def fly(s):
        k, thrust, e_prev, next_frame = 0, MASS * G, None, 0.0
        while not s.stopped:
            ds = drone_state(eng.true_state(), 0)
            t = k / eng.freq
            if k % every == 0:
                pid.kp, pid.ki, pid.kd = s.values["kp"], s.values["ki"], s.values["kd"]
                zr, z = s.values["target"], float(ds["p"][2])
                thrust = float(filled(altitude_thrust(pid, zr, z, DT)))
                if not np.isfinite(thrust):
                    raise ValueError(_t(f"추력이 {thrust} 이에요. 계산식을 확인하세요.", f"Thrust is {thrust}. Check your formula."))
                e = zr - z
                d = 0.0 if e_prev is None else (e - e_prev) / DT
                e_prev = e
                s.emit(t, z=z, target=zr, error=e, P=pid.kp * e, I=pid.ki * float(np.sum(pid.integral)),
                       D=pid.kd * d, thrust=min(max(thrust, 0.0), MAX_THRUST), hover=MASS * G)
                s.pace(t)
                if not window_off[0] and t >= next_frame:
                    try:
                        eng.render(mode="human")
                    except Exception as ex:  # noqa: BLE001  화면이 없으면 창만 끈다
                        print(_t(f"[3D 창을 열 수 없어서 끕니다: {ex}]", f"[Can't open the 3D window, turning it off: {ex}]"))
                        window_off[0] = True
                    next_frame = t + 1 / 30
            tau = level(ds, np.zeros(3))
            eng.step([[thrust, *tau]])
            k += 1

    window_off = [not window]
    return LiveSession(
        fly, group="alt", duration=duration,
        sliders={"target": (Z0, 0.2, 2.0, 0.1), "kp": (kp, 0, 30, 0.5), "ki": (ki, 0, 15, 0.5), "kd": (kd, 0, 12, 0.5)},
        labels={"target": _t("목표 높이 [m]", "target height [m]"), "z": _t("높이", "height"), "error": _t("오차", "error"),
                "thrust": _t("추력", "thrust"), "hover": _t("호버 추력 mg", "hover thrust mg")},
        panels=[(_t("높이: 목표 슬라이더를 움직여 보세요", "Height: try moving the target slider"), ["z", "target"], "m"),
                (_t("PID 출력의 세 항: P는 지금 오차, I는 쌓인 오차, D는 오차가 바뀌는 빠르기",
                    "The three PID terms: P is the error now, I the error piled up, D how fast the error changes"),
                 ["P", "I", "D"], "m/s²"),
                (_t("모터에 준 추력", "Thrust given to the motors"), ["thrust", "hover"], "N")],
    ).start()


# 내부 ---------------------------------------------------------------------------------

def _drone(heavy):
    return _t("실제가 10% 무거운 기체", "10% heavy drone") if heavy else _t("질량이 맞는 기체", "right-mass drone")


def _run(pid, altitude_thrust, heavy):
    t, z, f = _fly(lambda zr, z, dt: altitude_thrust(pid, zr, z, dt), 1.1 if heavy else 1.0)
    return {"t": t, "z": z, "thrust": f, "m": step_metrics(t, z, T_STEP, Z0, Z1)}


def _draw_step(ax, r, color, label, band=True):
    t = r["t"]
    if band:
        ax.plot(t, _target(t), color="k", lw=1, ls="--", label=_t("목표", "target"))
        ax.fill_between(t, Z1 - 0.02, Z1 + 0.02, where=t >= T_STEP, color="#2f9e44", alpha=0.15, lw=0, label="±2 cm")
        ax.axhline(Z1 + 0.1, color="#d62728", lw=0.8, ls=":", label=_t("오버슈트 10%", "overshoot 10%"))
    ax.plot(t, r["z"], color=color, lw=2, label=label)
    ax.set_xlabel("t [s]"); ax.set_ylabel(_t("고도 z [m]", "height z [m]")); ax.set_ylim(0.2, 2.0); ax.set_xlim(0, T_END)
    ax.legend(loc="lower right", fontsize=8, ncol=2)


def _metrics_table(runs):
    rows = []
    for name, r in runs:
        m = r["m"]
        os_, st, ss = f"{m['overshoot_pct']:.1f}", f"{m['settle_s']:.2f}", f"{100 * m['ss_err_m']:.1f}"
        rows.append((name, _t(f"오버슈트 {os_} % · 정착 {st} s · 마지막 오차 {ss} cm",
                              f"overshoot {os_} % · settling {st} s · last-second error {ss} cm"), "", None))
    table(rows, header=("", _t("결과", "result"), "", ""))


def warmup():
    """물리 엔진을 미리 한 번 돌려 둔다 (처음 한 번은 컴파일 때문에 몇 초 걸린다)."""
    from dsp.limits import describe
    _fly(lambda zr, z, dt: MASS * G, 1.0, t_end=0.1)
    print(_t("시뮬레이터 준비 완료", "Simulator ready") + f" · {describe()}")
