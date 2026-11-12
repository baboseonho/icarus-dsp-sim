"""그림, 영상, 3D 창.

Plots, videos and the 3D window.

    LiveView(fps=30)          # World.run(viewer=...)에 넘기면 MuJoCo 3D 창이 뜬다 (시뮬 속도는 그대로)
    save_video(log, path)     # 기록으로 mp4를 만든다 (위에서 본 그림 + 고도 그래프)
"""
from __future__ import annotations

import os
import time
from pathlib import Path

import numpy as np

import warnings
warnings.filterwarnings("ignore", message="Unable to import Axes3D")
os.environ.setdefault("MPLBACKEND", "Agg")
import matplotlib  # noqa: E402

matplotlib.use(os.environ.get("MPLBACKEND", "Agg"))
import matplotlib.pyplot as plt  # noqa: E402

COLORS = ["#2a6fdb", "#e8590c", "#2f9e44", "#ae3ec9", "#f59f00", "#1098ad"]


def out_dir(week_dir) -> Path:
    p = Path(week_dir) / "out"
    p.mkdir(parents=True, exist_ok=True)
    return p


class LiveView:
    """MuJoCo 3D 창. 벽시계 기준으로 초당 fps장까지만 그려서 시뮬을 늦추지 않는다.

    MuJoCo 3D window. Draws at most `fps` frames per wall-clock second so the
    simulation is not slowed down.
    """

    def __init__(self, fps=30, record=None, record_fps=30, size=(960, 540)):
        self.period = 1.0 / fps
        self._next = 0.0
        self.frames = 0
        self.record = record          # 파일 경로를 주면 창 대신 영상으로 저장
        self.record_fps = record_fps
        self.size = size
        self._writer = None
        self._next_sim = 0.0
        self.failed = None
        self.render_time = 0.0

    def __call__(self, world):
        if self.failed:
            return
        try:
            if self.record:
                if world.t + 1e-9 < self._next_sim:
                    return
                self._next_sim += 1.0 / self.record_fps
                if self._writer is None:
                    import imageio
                    self._writer = imageio.get_writer(self.record, fps=self.record_fps, codec="libx264",
                                                      quality=7, macro_block_size=8)
                img = world.engine.render(width=self.size[0], height=self.size[1], mode="rgb_array")
                self._writer.append_data(np.asarray(img))
                self.frames += 1
                return
            now = time.perf_counter()
            if now < self._next:
                return
            world.engine.render(mode="human")
            dt = time.perf_counter() - now
            self.render_time += dt
            # 그리는 데 벽시계 시간의 1/3 넘게 쓰지 않도록 간격을 벌린다
            self._next = now + max(self.period, 3.0 * dt)
            self.frames += 1
        except Exception as e:  # 화면이 없는 환경이면 창만 끄고 시뮬은 계속
            self.failed = repr(e)
            print(f"[viz] 3D 창을 열 수 없어서 끕니다: {e}")

    def close(self):
        if self._writer is not None:
            self._writer.close()
            self._writer = None


def save_video(log, path, offsets_by_time=None, fps=25, title="", names=None, stride=None, safe=0.3):
    """기록으로 mp4를 만든다. 왼쪽은 위에서 본 그림, 오른쪽은 고도와 최소 거리. CPU로 그린다.

    Render a log to mp4: top view on the left, altitude and minimum distance on
    the right. CPU only.
    """
    import imageio
    t, P = np.asarray(log["t"]), np.asarray(log["p"])
    n = P.shape[1]
    names = names or [f"cf{i + 1}" for i in range(n)]
    dt = t[1] - t[0]
    stride = stride or max(1, int(round(1.0 / (fps * dt))))
    fig, (ax, az) = plt.subplots(1, 2, figsize=(9.6, 5.04), gridspec_kw={"width_ratios": [1.15, 1]}, dpi=100)
    writer = imageio.get_writer(path, fps=fps, codec="libx264", quality=7, macro_block_size=8)
    from .grading import min_pair_distance
    dmin = min_pair_distance(P)
    for k in range(0, len(t), stride):
        ax.cla(); az.cla()
        lo = max(0, k - int(2.0 / dt))
        for i in range(n):
            ax.plot(P[lo:k + 1, i, 0], P[lo:k + 1, i, 1], color=COLORS[i % 6], lw=1, alpha=0.5)
            ax.add_patch(plt.Circle((P[k, i, 0], P[k, i, 1]), safe / 2, color=COLORS[i % 6], alpha=0.25))
            ax.plot(P[k, i, 0], P[k, i, 1], "o", color=COLORS[i % 6], ms=6)
            ax.text(P[k, i, 0] + 0.08, P[k, i, 1] + 0.08, names[i], fontsize=8)
        ax.set_xlim(-2.2, 2.2); ax.set_ylim(-2.2, 2.2); ax.set_aspect("equal"); ax.grid(alpha=0.3)
        ax.set_title(f"{title}  t = {t[k]:5.1f} s   (top view)")
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
        for i in range(n):
            az.plot(t[:k + 1], P[:k + 1, i, 2], color=COLORS[i % 6], lw=1)
        az.plot(t[:k + 1], dmin[:k + 1], "k--", lw=1, label="min distance")
        az.axhline(safe, color="r", lw=0.8, ls=":")
        az.set_xlim(t[0], t[-1]); az.set_ylim(-0.05, 2.0); az.grid(alpha=0.3)
        az.set_xlabel("t [s]"); az.set_ylabel("z [m] / min dist [m]"); az.legend(loc="upper right", fontsize=8)
        fig.canvas.draw()
        img = np.asarray(fig.canvas.buffer_rgba())[..., :3]
        writer.append_data(img)
    writer.close()
    plt.close(fig)
    return path
