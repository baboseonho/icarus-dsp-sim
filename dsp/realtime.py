"""비행 중 값을 실시간으로 보기.

세 가지를 한데 묶는다.
  - PlotJuggler로 보내기: Telemetry("alt").send(z=..., target=...) 하면 PlotJuggler에 /alt/z, /alt/target이 생긴다.
    UDP 한 줄(Influx line protocol)이라 받는 쪽이 없어도 아무 일도 없다.
  - plotjuggler("week01"): PlotJuggler 창을 이 회차 그래프 배치로 연다. 처음 뜨는 물음 두 개(Yes, OK)만 누르면 된다.
  - LiveSession: 비행을 1배속으로 뒤에서 돌리고, 노트북에는 슬라이더와 멈춤 버튼, 실시간 그래프를 띄운다.
    비행 중에 슬라이더를 움직이면 다음 제어 주기부터 바로 반영된다.

Live view of flight values: UDP telemetry to PlotJuggler, a launcher that opens
PlotJuggler with the week's layout, and LiveSession, which flies in real time in
a background thread with sliders, a stop button and a live chart in the notebook.
"""
from __future__ import annotations

import configparser
import io
import math
import os
import shutil
import socket
import subprocess
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np

PJ_PORT = 9870
LAYOUTS = Path(__file__).resolve().parent / "plotjuggler"
_PJ = {"proc": None}
_CURRENT = {"session": None}


# PlotJuggler로 보내기 -------------------------------------------------------------

class Telemetry:
    """값을 PlotJuggler UDP 수신기로 보낸다. 이름은 /<group>/<값 이름>으로 보인다.

    배열은 성분마다 나눈다: send(p=[x, y, z]) → /group/p_x, /group/p_y, /group/p_z.
    """

    def __init__(self, group, host="127.0.0.1", port=PJ_PORT):
        self.group = group
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

    @staticmethod
    def _flat(name, v, out):
        a = np.asarray(v, float)
        if a.ndim == 0:
            out[name] = float(a)
        elif a.size == 3:
            for s, x in zip("xyz", a):
                out[f"{name}_{s}"] = float(x)
        else:
            for i, x in enumerate(a.ravel()):
                out[f"{name}_{i}"] = float(x)

    def send(self, **values):
        flat = {}
        for k, v in values.items():
            if v is not None:
                self._flat(k, v, flat)
        fields = ",".join(f"{k}={v:.6g}" for k, v in flat.items() if math.isfinite(v))
        if not fields:
            return
        try:
            self.sock.sendto(f"{self.group} {fields} {time.time_ns()}".encode(), self.addr)
        except OSError:
            pass                    # 받는 쪽이 없거나 버퍼가 차면 그냥 버린다

    def close(self):
        self.sock.close()


def _ros_distro():
    """ROS 2 배포판 이름. Ubuntu 24.04는 jazzy, 26.04는 lyrical (setup/env.sh가 ROS_DISTRO를 정한다)."""
    return os.environ.get("ROS_DISTRO") or os.environ.get("DSP_ROS_DISTRO") or "jazzy"


def _pj_binary():
    cands = [shutil.which("plotjuggler")]
    cands += [f"/opt/ros/{d}/lib/plotjuggler/plotjuggler" for d in dict.fromkeys((_ros_distro(), "jazzy", "lyrical"))]
    for p in cands:
        if p and os.path.exists(p):
            return p
    return None


def _pj_settings(streamer):
    """PlotJuggler 설정 파일에 수신기 종류, 포트, 시각 표시를 미리 적어 둔다 (다른 설정은 그대로)."""
    path = Path.home() / ".config" / "PlotJuggler" / "io.plotjuggler.PlotJuggler.ini"
    path.parent.mkdir(parents=True, exist_ok=True)
    cp = configparser.ConfigParser(interpolation=None, strict=False)
    cp.optionxform = str
    if path.exists():
        try:
            cp.read(path)
        except configparser.Error:
            cp = configparser.ConfigParser(interpolation=None, strict=False)
            cp.optionxform = str
    if not cp.has_section("General"):
        cp.add_section("General")
    g = cp["General"]
    g["MainWindow.previousStreamingPlugin"] = streamer
    g["MainWindow.dateTimeDisplay"] = "true"           # 가로축을 시:분:초로
    g["MainWindow.streamingBufferValue"] = "15"        # 최근 15초를 보여 준다
    g["UDP_Server%3A%3Aaddress"] = "127.0.0.1"
    g["UDP_Server%3A%3Aport"] = str(PJ_PORT)
    g["UDP_Server%3A%3Aprotocol"] = "Influx (Line protocol)"
    with open(path, "w") as f:
        cp.write(f, space_around_delimiters=False)


def plotjuggler(layout=None):
    """PlotJuggler를 연다. layout은 "week01"처럼 회차 이름이나 xml 경로.

    창이 뜨면 "Start the previously used streaming plugin?"에 Yes, 설정 창에 OK를 누른다.
    """
    p = _PJ["proc"]
    if p is not None and p.poll() is None:
        print("PlotJuggler가 이미 열려 있어요. 그 창을 쓰면 됩니다 (다른 회차 배치는 창에서 Layout → 불러오기).")
        return p
    exe = _pj_binary()
    if exe is None:
        d = _ros_distro()
        pkg = f"ros-{d}-plotjuggler-ros" if d == "jazzy" else f"ros-{d}-plotjuggler"   # Lyrical은 아직 본체만 있다
        print(f"PlotJuggler가 설치되어 있지 않아요. 터미널에서: sudo apt install {pkg}\n"
              "PlotJuggler 없이도 아래 노트북 그래프는 그대로 실시간으로 그려집니다.")
        return None
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        print("화면(DISPLAY)이 없는 환경이라 PlotJuggler 창을 띄울 수 없어요.")
        return None
    args = [exe, "-n"]
    streamer = "UDP Server"
    if layout:
        lp = Path(layout)
        if not lp.suffix:
            lp = LAYOUTS / f"{layout}.xml"
        if lp.exists():
            args += ["-l", str(lp)]
            if "ROS2 Topic Subscriber" in lp.read_text():
                streamer = "ROS2 Topic Subscriber"
    _pj_settings(streamer)
    env = os.environ.copy()
    ros_root = "/".join(exe.split("/")[:4]) if exe.startswith("/opt/ros/") else None   # 예: /opt/ros/jazzy
    if ros_root and f"{ros_root}/lib" not in env.get("LD_LIBRARY_PATH", ""):
        env["LD_LIBRARY_PATH"] = f"{ros_root}/lib:{ros_root}/lib/x86_64-linux-gnu:" + env.get("LD_LIBRARY_PATH", "")
    _PJ["proc"] = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                   start_new_session=True, env=env)
    print("PlotJuggler를 엽니다. 물음이 뜨면 Yes, 설정 창은 OK를 누르세요. 그다음 아래 실시간 비행을 시작하면 그래프가 움직입니다.")
    return _PJ["proc"]


# 실시간 비행 ---------------------------------------------------------------------

class LiveSession:
    """비행 함수를 뒤에서 1배속으로 돌리고, 노트북에 슬라이더, 멈춤 버튼, 실시간 그래프를 띄운다.

        def fly(s):                          # s는 이 LiveSession
            while not s.stopped:
                ...한 주기 계산...
                s.emit(t, z=z, target=zr)    # 그래프와 PlotJuggler로
                s.pace(t)                    # 벽시계에 맞춰 기다림
        LiveSession(fly, sliders={"kp": (8, 0, 30, 0.5)}, panels=[("고도", ["z", "target"], "m")],
                    group="alt").start()

    s.values는 슬라이더 값 dict이고, 비행 중에도 바로 바뀐다.
    """

    def __init__(self, fly, sliders=None, panels=(), group="dsp", title="", duration=120.0, window_s=12.0,
                 chart_hz=3.0, labels=None, checkboxes=None, choices=None):
        self.fly = fly
        self.slider_spec = dict(sliders or {})
        self.check_spec = dict(checkboxes or {})
        self.choice_spec = dict(choices or {})          # {이름: ([보기, ...], 처음 값)}
        self.values = {k: v[0] for k, v in self.slider_spec.items()}
        self.values.update({k: bool(v) for k, v in self.check_spec.items()})
        self.values.update({k: v[1] for k, v in self.choice_spec.items()})
        self.panels = list(panels)
        self.labels = labels or {}
        self.title = title
        self.duration = duration
        self.window_s = window_s
        self.chart_dt = 1.0 / chart_hz
        self.tx = Telemetry(group)
        self.buf = {}
        self.lock = threading.Lock()
        self.stopped = False
        self.error = None
        self.t_sim = 0.0
        self._w0 = None
        self._s0 = 0.0
        self.th = None

    # 비행 함수가 부르는 것 ----------------------------------------------------------

    def emit(self, t, **signals):
        self.t_sim = t
        flat = {}
        for k, v in signals.items():
            if v is not None:
                Telemetry._flat(k, v, flat)
        with self.lock:
            for k, v in flat.items():
                self.buf.setdefault(k, deque(maxlen=4000)).append((t, v))
        self.tx.send(**signals)

    def pace(self, t):
        """시뮬 시각 t가 벽시계보다 앞서 있으면 기다린다 (1배속)."""
        if self._w0 is None:
            self._w0, self._s0 = time.perf_counter(), t
        lag = (t - self._s0) - (time.perf_counter() - self._w0)
        if lag > 0.002:
            time.sleep(lag)
        if t - self._s0 > self.duration:
            self.stopped = True

    # 노트북 쪽 -------------------------------------------------------------------

    def _ui(self):
        import ipywidgets as W
        sl = []
        for k, (v, lo, hi, step) in self.slider_spec.items():
            s = W.FloatSlider(value=v, min=lo, max=hi, step=step, description=self.labels.get(k, k),
                              continuous_update=True, style={"description_width": "initial"},
                              layout=W.Layout(width="320px"))
            s.observe(lambda ch, k=k: self.values.__setitem__(k, ch["new"]), names="value")
            sl.append(s)
        for k, v in self.check_spec.items():
            c = W.Checkbox(value=bool(v), description=self.labels.get(k, k), indent=False)
            c.observe(lambda ch, k=k: self.values.__setitem__(k, ch["new"]), names="value")
            sl.append(c)
        for k, (opts, v) in self.choice_spec.items():
            d = W.Dropdown(options=opts, value=v, description=self.labels.get(k, k),
                           style={"description_width": "initial"}, layout=W.Layout(width="260px"))
            d.observe(lambda ch, k=k: self.values.__setitem__(k, ch["new"]), names="value")
            sl.append(d)
        self.btn = W.Button(description="■ 멈춤", button_style="danger", layout=W.Layout(width="90px"))
        self.btn.on_click(lambda _: self.stop())
        self.status = W.HTML(value="")
        self.img = W.Image(format="png")
        rows = [W.HBox(sl[i:i + 3]) for i in range(0, len(sl), 3)]
        return W.VBox(rows + [W.HBox([self.btn, self.status]), self.img])

    def _chart_png(self):
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from .live import COLORS
        n = max(1, len(self.panels))
        fig = Figure(figsize=(9.5, 1.9 * n + 0.3), dpi=80)
        FigureCanvasAgg(fig)
        axes = fig.subplots(n, 1, sharex=True, squeeze=False)[:, 0]
        with self.lock:
            data = {k: np.array(v) for k, v in self.buf.items() if len(v)}
        t1 = self.t_sim
        t0 = max(0.0, t1 - self.window_s)
        for ax, (ptitle, names, unit) in zip(axes, self.panels):
            seen = []
            for c, name in enumerate(names):
                if name not in data:
                    continue
                d = data[name]
                m = d[:, 0] >= t0
                dashed = "target" in name or name.startswith("sp_") or name == "safe"
                base = name.replace("sp_", "p_", 1) if name.startswith("sp_") else None
                c = names.index(base) if base in names else c          # 목표는 짝이 되는 값과 같은 색, 점선
                ax.plot(d[m, 0], d[m, 1], color=COLORS[c % 6], lw=1.6, ls="--" if dashed else "-",
                        label=self.labels.get(name, name))
                seen.append(d[m, 1])
            if seen:                     # 순간적으로 튀는 값(목표가 갑자기 바뀔 때의 D항 등) 때문에 그래프가 납작해지지 않게
                v = np.concatenate(seen)
                v = v[np.isfinite(v)]
                if v.size:
                    lo, hi = np.percentile(v, [1, 99])
                    pad = max(0.12 * (hi - lo), 0.02 * max(abs(lo), abs(hi)), 1e-3)
                    ax.set_ylim(lo - pad, hi + pad)
            ax.set_ylabel(unit)
            ax.set_title(ptitle, fontsize=10, loc="left")
            ax.grid(alpha=0.3)
            ax.set_xlim(t0, max(t0 + self.window_s, t1))
            if ax.lines:
                ax.legend(loc="upper left", fontsize=8, ncol=min(4, len(ax.lines)))
        axes[-1].set_xlabel("t [s]")
        fig.tight_layout()
        bio = io.BytesIO()
        fig.savefig(bio, format="png")
        return bio.getvalue()

    def _run(self):
        try:
            self.fly(self)
        except Exception as e:  # noqa: BLE001
            self.error = e
        finally:
            self.stopped = True

    def start(self):
        from IPython.display import display
        old = _CURRENT["session"]
        if old is not None and not old.stopped:      # 앞의 실시간 비행은 멈추고 시작
            old.stop()
            old.wait(5)
        _CURRENT["session"] = self
        display(self._ui())
        self.th = threading.Thread(target=self._run, daemon=True)
        self.th.start()
        threading.Thread(target=self._watch, daemon=True).start()
        return self

    def _watch(self):
        from .live import BlankError
        while True:
            done = self.stopped
            try:
                self.img.value = self._chart_png()
            except Exception:  # noqa: BLE001
                pass
            if done:
                break
            self.status.value = f"&nbsp; 비행 중 · t = {self.t_sim:5.1f} s (최대 {self.duration:.0f} s)"
            time.sleep(self.chart_dt)
        self.th.join(timeout=5)
        e = self.error
        if e is None:
            msg = f"&nbsp; 끝 · {self.t_sim:.1f} s 비행"
        elif isinstance(e, BlankError) or "BlankError" in type(e).__name__:
            msg = "&nbsp; ✍️ 아직 채우지 않은 빈칸(___)이 있어요. 위 셀을 채우고 다시 실행하세요."
        else:
            msg = f"&nbsp; 오류로 멈춤: {type(e).__name__}: {e}"
        self.status.value = msg
        self.btn.disabled = True
        self.tx.close()

    def stop(self):
        self.stopped = True

    def wait(self, timeout=None):
        """끝날 때까지 기다린다 (노트북 검증용)."""
        if self.th is not None:
            self.th.join(timeout)
        return self


def window_viewer(enabled):
    """MuJoCo 3D 창. enabled가 False면 None (World.run(viewer=...)에 그대로 넘긴다)."""
    if not enabled:
        return None
    from .viz import LiveView
    return LiveView(fps=30)
