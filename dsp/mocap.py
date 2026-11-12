"""천장 카메라 블록 `mocap`.

시뮬에서는 엔진의 실제 위치에 잡음(σ 약 5 mm), 지연(약 20 ms), 가끔 놓치는 샘플(약 1%)을
섞어 내보낸다. 실기체에서는 OpenCV ArUco 천장 카메라가 이 자리에 들어간다 (`ArucoMocap`, 아직 비어 있음).
카메라는 위치만 주고, 속도는 지상국이 차분으로 추정한다 (`PoseTracker`).

Ceiling camera block. In the sim it publishes the true position with noise,
latency and occasional misses; on real hardware an ArUco camera takes its
place. The camera gives position only, so the ground station estimates
velocity by differencing (`PoseTracker`).

포즈 형식 (모든 회차 공통):  {id, t, p:[x,y,z], q:[w,x,y,z]}
"""
from __future__ import annotations

from collections import deque

import numpy as np


class VirtualMocap:
    """시뮬용 천장 카메라. rate [Hz]마다 모든 드론의 포즈를 잡음·지연·누락과 함께 내보낸다.

    Simulated ceiling camera: publishes every drone's pose at `rate` Hz with
    noise, delay and dropped samples.
    """

    def __init__(self, names, rate=100.0, noise=0.005, delay=0.020, drop=0.01, seed=0):
        self.names = list(names)
        self.period = 1.0 / rate
        self.noise, self.delay, self.drop = float(noise), float(delay), float(drop)
        self.rng = np.random.default_rng(seed + 2000)
        self._next = 0.0
        self._queue = deque()
        self.samples = 0
        self.missed = 0

    def update(self, t, true_state):
        """물리 스텝마다 엔진의 진짜 상태를 넣어 부른다.

        Call every physics step with the engine's true state.
        """
        if t + 1e-9 < self._next:
            return
        self._next += self.period
        if self._next < t:
            self._next = t + self.period
        P, Q = true_state["p"], true_state["q"]
        for i, name in enumerate(self.names):
            self.samples += 1
            if self.rng.random() < self.drop:
                self.missed += 1
                continue
            p = P[i] + self.rng.normal(0.0, self.noise, 3) if self.noise > 0 else P[i].copy()
            self._queue.append((t + self.delay, {"id": name, "t": float(t), "p": p.tolist(), "q": Q[i].tolist()}))

    def poll(self, t):
        """시각 t까지 도착한 포즈 목록.

        Poses that have arrived by time t.
        """
        out = []
        while self._queue and self._queue[0][0] <= t + 1e-9:
            out.append(self._queue.popleft()[1])
        return out


class PoseTracker:
    """지상국이 보는 드론별 최신 위치와, 차분으로 추정한 속도.

    Latest position of each drone on the ground station, plus a velocity
    estimate from finite differences.

        tr = PoseTracker(names)
        tr.add(pose)                 # 카메라 포즈가 올 때마다
        P = tr.positions()           # (n,3)
        V = tr.velocities()          # (n,3), 차분에 저역통과를 건 값
        tr.stale(now, 0.3)           # 0.3 s 넘게 소식이 없는 드론 이름들
    """

    def __init__(self, names, alpha=0.3):
        self.names = list(names)
        self.idx = {n: i for i, n in enumerate(self.names)}
        n = len(self.names)
        self.p = np.full((n, 3), np.nan)
        self.v = np.zeros((n, 3))
        self.t = np.full(n, -np.inf)
        self.alpha = alpha

    def add(self, pose):
        i = self.idx[pose["id"]]
        p = np.asarray(pose["p"], float)
        t = float(pose["t"])
        if np.isfinite(self.t[i]) and t > self.t[i]:
            v = (p - self.p[i]) / (t - self.t[i])
            self.v[i] = (1 - self.alpha) * self.v[i] + self.alpha * v
        self.p[i], self.t[i] = p, t

    def positions(self):
        return self.p.copy()

    def velocities(self):
        return self.v.copy()

    def ready(self):
        return bool(np.all(np.isfinite(self.t)))

    def stale(self, now, timeout=0.3):
        return [n for n, ti in zip(self.names, self.t) if now - ti > timeout]


class ArucoMocap:
    """실기체용 ArUco 천장 카메라 자리. 실기체 단계에서 채운다.

    OpenCV `cv2.aruco.ArucoDetector`와 카메라 보정으로 마커마다 world 위치를 구하고,
    VirtualMocap과 같은 포즈 형식으로 내보내면 지상국 코드는 그대로 쓸 수 있다.

    Placeholder for the real ArUco ceiling camera. It should publish the same
    pose format as VirtualMocap so the ground station code does not change.
    """

    def __init__(self, *a, **kw):
        raise NotImplementedError("ArUco 카메라는 실기체 단계에서 구현한다")
