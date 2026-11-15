"""수업 노트북이 같이 쓰는 도구: 빈칸 표시, 그래프 설정, 위에서 본 애니메이션, 결과 표.

각 회차의 background.py가 `from dsp.live import *`로 가져다 쓴다.

Helpers shared by the lesson notebooks: the ___ blank marker, plot setup,
top-view animations and pass/fail tables.
"""
from __future__ import annotations

import os
import sys
import warnings

import numpy as np

warnings.filterwarnings("ignore", message="Unable to import Axes3D")
import matplotlib  # noqa: E402

if "ipykernel" in sys.modules:
    # setup/env.sh나 다른 곳에서 MPLBACKEND=Agg가 넘어와도 노트북 안에서는 그림이 보이게
    matplotlib.use("module://matplotlib_inline.backend_inline")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import animation, font_manager  # noqa: E402

from .lang import get_language, set_language, tr  # noqa: E402,F401

COLORS = ["#2a6fdb", "#e8590c", "#2f9e44", "#ae3ec9", "#f59f00", "#1098ad"]
SAFE_DIST = 0.30


# 빈칸 ----------------------------------------------------------------------

class BlankError(Exception):
    pass


class _Blank:
    """노트북의 빈칸 `___`. 채우지 않은 채로 계산에 쓰면 어디를 채워야 하는지 알려 준다.

    The `___` placeholder in the notebooks. Using it in any calculation raises a
    friendly error instead of a confusing one.
    """

    def _stop(self, *a, **k):
        raise BlankError(tr("아직 채우지 않은 빈칸(___)이 있어요. 위 셀의 ___ 를 채우고 그 셀부터 다시 실행하세요.",
                            "A blank (___) is still empty. Fill in the ___ in the cell above and run again from that cell."))

    __add__ = __radd__ = __sub__ = __rsub__ = __mul__ = __rmul__ = _stop
    __truediv__ = __rtruediv__ = __pow__ = __rpow__ = __neg__ = __matmul__ = __rmatmul__ = _stop
    __lt__ = __le__ = __gt__ = __ge__ = __bool__ = __float__ = __int__ = __index__ = _stop
    __getitem__ = __setitem__ = __iter__ = __len__ = __call__ = __array__ = _stop

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        self._stop()

    def __repr__(self):
        return "___"


___ = _Blank()


def _keep_blank_in_notebook():
    # IPython은 `___`를 "세 번 전 출력"으로 쓰면서 셀마다 덮어쓴다.
    # 셀을 실행하기 직전마다 다시 빈칸 표시로 돌려놓는다.
    try:
        ip = get_ipython()  # noqa: F821
    except NameError:
        return
    if getattr(ip, "_dsp_blank_hook", False):
        return

    def reset(*_):
        ip.user_ns["___"] = ___

    def short_message(shell, etype, value, tb, tb_offset=None):
        print(f"✍️ {value}")          # 긴 오류 추적 대신 한 줄만

    ip.events.register("pre_run_cell", reset)
    ip.set_custom_exc((BlankError,), short_message)
    ip._dsp_blank_hook = True
    reset()


_keep_blank_in_notebook()


def filled(*values):
    """값 가운데 빈칸이 남아 있으면 BlankError를 낸다."""
    for v in values:
        if isinstance(v, _Blank):
            v._stop()
    return values[0] if len(values) == 1 else values


# 그래프 설정 -----------------------------------------------------------------

def _korean_font():
    for name in ("NanumGothic", "NanumBarunGothic", "Noto Sans CJK KR", "Noto Sans KR", "UnDotum"):
        if any(f.name == name for f in font_manager.fontManager.ttflist):
            return name
    return None


def setup_plots():
    font = _korean_font()
    plt.rcParams.update({
        "figure.dpi": 100, "figure.figsize": (8, 3.6), "axes.grid": True, "grid.alpha": 0.3,
        "axes.spines.top": False, "axes.spines.right": False, "axes.unicode_minus": False,
        "legend.frameon": False, "animation.embed_limit": 40,
    })
    if font:
        plt.rcParams["font.family"] = font
    elif get_language() == "ko":
        print("한글 글꼴이 없어서 그래프 글자가 깨질 수 있어요:  sudo apt install fonts-nanum  후 커널 재시작")
    try:
        import imageio_ffmpeg
        plt.rcParams["animation.ffmpeg_path"] = imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        pass


setup_plots()


def log_cm_axis(ax, axis="y"):
    """로그 눈금을 '1 cm, 10 cm'처럼 cm로 적는다 (한글 글꼴에는 10⁻²의 수학 기호가 없다). 값의 단위는 m."""
    from matplotlib.ticker import FuncFormatter, NullFormatter
    fmt = FuncFormatter(lambda v, _: (f"{100 * v:g} cm" if v > 0 else ""))
    target = ax.yaxis if axis == "y" else ax.xaxis
    target.set_major_formatter(fmt)
    target.set_minor_formatter(NullFormatter())


# 표 -------------------------------------------------------------------------

def table(rows, header=None):
    """[(항목, 값, 기준, 통과 여부), ...]를 초록/빨강 표로 보여 준다. 통과 여부가 None이면 색을 칠하지 않는다."""
    from IPython.display import HTML, display
    head = header or tr(("항목", "결과", "기준", ""), ("item", "result", "limit", ""))
    h = "".join(f"<th style='text-align:left;padding:4px 12px'>{c}</th>" for c in head)
    body = ""
    for name, value, target, ok in rows:
        mark = "" if ok is None else ("✅" if ok else "❌")
        bg = "" if ok is None else ("#e6f4ea" if ok else "#fce8e6")
        body += (f"<tr style='background:{bg}'><td style='padding:4px 12px'>{name}</td>"
                 f"<td style='padding:4px 12px'><b>{value}</b></td><td style='padding:4px 12px'>{target}</td>"
                 f"<td style='padding:4px 12px'>{mark}</td></tr>")
    display(HTML(f"<table style='border-collapse:collapse;font-size:14px'><tr>{h}</tr>{body}</table>"))


# 위에서 본 그림 ------------------------------------------------------------------

def min_pair_distance(P):
    """P (..., n, 3)에서 가장 가까운 두 대의 거리 (...,)."""
    P = np.asarray(P, float)
    D = np.linalg.norm(P[..., :, None, :] - P[..., None, :, :], axis=-1)
    n = P.shape[-2]
    D[..., np.arange(n), np.arange(n)] = np.inf
    return D.min(axis=(-1, -2))


def top_view(P, targets=None, ax=None, title="", names=None, lim=2.2, start=True):
    """궤적 전체를 위에서 본 그림. P (T, n, 3)."""
    P = np.asarray(P)
    n = P.shape[1]
    names = names or [f"cf{i + 1}" for i in range(n)]
    ax = ax or plt.subplots(figsize=(4.6, 4.6))[1]
    for i in range(n):
        c = COLORS[i % 6]
        ax.plot(P[:, i, 0], P[:, i, 1], color=c, lw=1.2, alpha=0.8)
        if start:
            ax.plot(P[0, i, 0], P[0, i, 1], "o", mfc="white", mec=c, ms=6)
        ax.plot(P[-1, i, 0], P[-1, i, 1], "o", color=c, ms=7)
        ax.annotate(names[i], P[-1, i, :2], xytext=(5, 5), textcoords="offset points", fontsize=8)
    if targets is not None:
        T = np.asarray(targets)
        ax.plot(T[:, 0], T[:, 1], "x", color="k", ms=8, mew=1.5, label="목표")
        ax.legend(loc="upper right", fontsize=8)
    ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_aspect("equal")
    ax.set_xlabel("x [m] (동쪽)"); ax.set_ylabel("y [m] (북쪽)")
    if title:
        ax.set_title(title)
    return ax


def animate(t, P, title="", targets=None, safe=SAFE_DIST, fps=12, speed=1.0, trail=2.0, lim=2.2,
            names=None, side="z"):
    """위에서 본 애니메이션(왼쪽)과 고도 또는 최소 거리 그래프(오른쪽)를 영상으로 만들어 보여 준다.

    t (T,), P (T, n, 3). targets는 (n, 3)이거나 시간마다 바뀌면 (T, n, 3).
    side="z"면 오른쪽에 고도, "dmin"이면 가장 가까운 두 대의 거리.
    speed=2면 두 배 빠르게 재생한다.
    """
    from IPython.display import HTML
    t, P = np.asarray(t), np.asarray(P)
    n = P.shape[1]
    names = names or [f"cf{i + 1}" for i in range(n)]
    dt = float(np.median(np.diff(t)))
    step = max(1, int(round(speed / (fps * dt))))
    frames = list(range(0, len(t), step))
    if frames[-1] != len(t) - 1:
        frames.append(len(t) - 1)
    tr = max(1, int(trail / dt))
    T = None if targets is None else np.asarray(targets)

    fig, (ax, az) = plt.subplots(1, 2, figsize=(9.2, 4.2), gridspec_kw={"width_ratios": [1.1, 1]})
    ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_aspect("equal")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    trails = [ax.plot([], [], color=COLORS[i % 6], lw=1, alpha=0.6)[0] for i in range(n)]
    dots = [ax.plot([], [], "o", color=COLORS[i % 6], ms=7)[0] for i in range(n)]
    rings = [plt.Circle((0, 0), safe / 2, color=COLORS[i % 6], alpha=0.18) for i in range(n)]
    for r in rings:
        ax.add_patch(r)
    labels = [ax.text(0, 0, names[i], fontsize=8) for i in range(n)]
    tgt = ax.plot([], [], "x", color="k", ms=7, mew=1.3)[0]
    head = ax.set_title(title)

    if side == "dmin":
        y = min_pair_distance(P)
        az.plot(t, y, color="#bbb", lw=1)
        live = [az.plot([], [], "k", lw=1.6)[0]]
        az.axhline(safe, color="#d62728", ls="--", lw=1, label=f"안전 거리 {safe:.2f} m")
        az.set_ylabel("가장 가까운 두 대 거리 [m]"); az.set_ylim(0, max(1.5, float(np.nanmax(y[np.isfinite(y)])) * 1.1))
        az.legend(loc="upper right", fontsize=8)
    else:
        for i in range(n):
            az.plot(t, P[:, i, 2], color=COLORS[i % 6], lw=0.8, alpha=0.25)
        live = [az.plot([], [], color=COLORS[i % 6], lw=1.4)[0] for i in range(n)]
        az.set_ylabel("고도 z [m]"); az.set_ylim(-0.05, max(1.6, float(P[:, :, 2].max()) + 0.2))
    cursor = az.axvline(t[0], color="k", lw=0.8, alpha=0.5)
    az.set_xlim(t[0], t[-1]); az.set_xlabel("t [s]")
    fig.tight_layout()

    def draw(k):
        lo = max(0, k - tr)
        for i in range(n):
            trails[i].set_data(P[lo:k + 1, i, 0], P[lo:k + 1, i, 1])
            dots[i].set_data([P[k, i, 0]], [P[k, i, 1]])
            rings[i].center = (P[k, i, 0], P[k, i, 1])
            labels[i].set_position((P[k, i, 0] + 0.08, P[k, i, 1] + 0.08))
        if T is not None:
            Tk = T[k] if T.ndim == 3 else T
            tgt.set_data(Tk[:, 0], Tk[:, 1])
        if side == "dmin":
            live[0].set_data(t[:k + 1], y[:k + 1])
        else:
            for i in range(n):
                live[i].set_data(t[:k + 1], P[:k + 1, i, 2])
        cursor.set_xdata([t[k], t[k]])
        head.set_text(f"{title}   t = {t[k]:4.1f} s" if title else f"t = {t[k]:4.1f} s")
        return trails + dots + live

    anim = animation.FuncAnimation(fig, draw, frames=frames, interval=1000 / fps, blit=False)
    try:
        html = anim.to_html5_video()
    except Exception:
        html = anim.to_jshtml()       # ffmpeg가 없으면 그림 묶음으로
    plt.close(fig)
    return HTML(html)


_ENGINES = {}


def get_engine(n=1, mass_scale=1.0, wind=(0.0, 0.0, 0.0), gust=0.0, downwash=False, seed=0, slot=""):
    """드론 n대짜리 엔진을 돌려준다. 같은 n이면 이미 컴파일한 엔진을 다시 쓰고 조건만 바꾼다.
    slot="live"는 실시간 비행 전용 엔진이다 (뒤에서 나는 동안 다른 셀이 같은 엔진을 건드리지 않게).

    Returns an n-drone engine, reusing an already compiled one when possible.
    slot="live" gives the real-time flight its own engine.
    """
    from .engine import Engine
    eng = _ENGINES.get((n, slot))
    if eng is None:
        eng = _ENGINES[(n, slot)] = Engine(n=n, seed=seed, mass_scale=mass_scale)
    eng.set_mass_scale(mass_scale)
    eng.wind = np.asarray(wind, float)
    eng.gust = float(gust)
    eng.downwash = bool(downwash)
    eng.rng = np.random.default_rng(seed)
    return eng


def is_notebook():
    return "ipykernel" in sys.modules


def quiet_jax():
    os.environ.setdefault("JAX_PLATFORMS", "cpu")
    os.environ.setdefault("XLA_FLAGS", "--xla_cpu_multi_thread_eigen=false")
    os.environ.setdefault("JAX_COMPILATION_CACHE_DIR", os.path.expanduser("~/.cache/icarus-dsp-jax"))


quiet_jax()

__all__ = ["np", "plt", "___", "BlankError", "filled", "table", "top_view", "animate", "min_pair_distance",
           "COLORS", "SAFE_DIST", "setup_plots", "get_engine", "log_cm_axis", "set_language"]
