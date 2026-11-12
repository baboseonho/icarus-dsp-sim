"""시스템 전체를 한 프로세스에서 돌리는 순수 Python 판. 1~3회차와 최종 순수 Python 판이 쓴다.

지상국 코드도 시뮬 시각에 맞춰 돌기 때문에 시드가 같으면 결과가 똑같이 나온다.

The whole system in one Python process (weeks 1-3 and the pure-Python final).
The ground station runs on sim time, so a fixed seed gives the same run every time.

    [mocap] ─ pose ─▶ [지상국] ─ UDP ─▶ [칩 ×n] ─ 추력·토크 ─▶ [엔진]

    w = World(controllers, init_pos, realism=Realism(), seed=0)
    w.run(10.0, ground_station=my_fn)      # my_fn(w, t)를 50 Hz로 부른다
    w.link["cf1"].takeoff(1.0, 2.0)         # 지상국 명령
    w.tracker.positions()                   # 지상국이 보는 위치 (카메라 값)
    w.log                                   # 기록 (진짜 위치 등)
"""
from __future__ import annotations

import time

import numpy as np

from . import config as C
from .engine import Engine
from .link import UdpLink, DroneRadio
from .mocap import VirtualMocap, PoseTracker
from .onboard import Chip, drone_state


class World:
    """엔진, 카메라, 무선, 드론 칩을 한데 묶은 시뮬레이션.

    Engine, camera, radio and drone chips wired together.
    """

    def __init__(self, controllers, init_pos, realism: C.Realism | None = None, seed=0, mass_scale=1.0,
                 names=None, log_hz=100, engine: Engine | None = None):
        realism = realism or C.Realism()
        self.realism = realism
        n = len(controllers)
        self.n = n
        self.names = list(names) if names else C.NAMES[:n]
        self.engine = engine or Engine(n=n, seed=seed, mass_scale=mass_scale, wind=realism.wind,
                                       gust=realism.gust, downwash=realism.downwash)
        self.engine.reset(n, init_pos=init_pos)
        clock = lambda: self.engine.t  # noqa: E731
        self.radios = [DroneRadio(nm, 0, clock=clock, delay=realism.link_delay) for nm in self.names]
        self.link = UdpLink(self.names, clock=clock, loss=realism.link_loss, seed=seed,
                            ports={r.name: r.port for r in self.radios})
        self.mocap = VirtualMocap(self.names, rate=realism.mocap_hz, noise=realism.mocap_noise,
                                  delay=realism.mocap_delay, drop=realism.mocap_drop, seed=seed)
        self.tracker = PoseTracker(self.names)
        self.chips = [Chip(nm, c) for nm, c in zip(self.names, controllers)]
        self._log_every = max(1, int(round(self.engine.freq / log_hz)))
        self.log = {"t": [], "p": [], "v": [], "rpy": [], "sp": [], "thrust": []}
        self.wall_time = 0.0
        self.mocap.update(0.0, self.engine.true_state())

    @property
    def t(self):
        return self.engine.t

    def step(self):
        eng = self.engine
        st = eng.true_state()
        t = eng.t
        cmds = np.zeros((self.n, 4))
        for i, chip in enumerate(self.chips):
            ds = drone_state(st, i)
            for pkt in self.radios[i].poll():
                chip.handle(pkt, t, ds)
            cmds[i] = chip.step(t, ds)
        eng.step(cmds)
        st = eng.true_state()
        self.mocap.update(eng.t, st)
        for pose in self.mocap.poll(eng.t):
            self.tracker.add(pose)
        if eng.steps % self._log_every == 0:
            L = self.log
            L["t"].append(eng.t); L["p"].append(st["p"].copy()); L["v"].append(st["v"].copy())
            from .engine import quat_wxyz_to_rpy
            L["rpy"].append(quat_wxyz_to_rpy(st["q"]))
            L["sp"].append(np.array([c.sp["p"] if c.sp is not None else np.full(3, np.nan) for c in self.chips]))
            L["thrust"].append(np.clip(cmds[:, 0], 0.0, eng.max_thrust))   # 엔진이 실제로 쓴 추력 (포화 반영)

    def run(self, duration, ground_station=None, gs_hz=C.LINK_HZ, viewer=None, realtime=False, until=None):
        """duration초 동안 진행한다. ground_station(world, t)를 gs_hz로 부르고,
        until(world)가 True를 돌려주면 일찍 끝낸다.

        Advance `duration` seconds, calling ground_station(world, t) at gs_hz.
        Stops early when until(world) returns True.
        """
        t_end = self.t + duration
        gs_dt = 1.0 / gs_hz
        next_gs = self.t
        w0 = time.perf_counter()
        s0 = self.t
        while self.t < t_end - 1e-9:
            if ground_station is not None and self.t + 1e-9 >= next_gs:
                next_gs += gs_dt
                ground_station(self, self.t)
                if until is not None and until(self):
                    break
            self.step()
            if viewer is not None:
                viewer(self)
            if realtime:
                lag = (self.t - s0) - (time.perf_counter() - w0)
                if lag > 0.002:
                    time.sleep(lag)
        self.wall_time += time.perf_counter() - w0
        return self

    def arrays(self):
        """기록을 numpy 배열로 바꿔 돌려준다. t (T,), p (T,n,3), ...

        The log as numpy arrays: t (T,), p (T,n,3), ...
        """
        return {k: np.asarray(v) for k, v in self.log.items()}

    def close(self):
        for r in self.radios:
            r.close()
        self.link.close()
