"""대형 비행에 미리 주는 부품: 삼각형 대형, 통신 그래프 모양, 중앙형 지상국.

3회차부터 배경 코드와 ROS 노드가 같이 쓴다. 부원이 채우는 부분(육각형·일렬 오프셋, 리더-팔로워,
합의 한 걸음, 연결 판정)은 여기 없고, 지상국에는 합의 함수를 바깥에서 넣는다.

Provided parts for formation flight: the triangle formation, graph shapes and the central
ground station. The parts members write live elsewhere; the consensus step is passed in.
"""
import numpy as np


def scatter(seed=0, n=6, box=1.6, min_sep=0.6):
    """바닥에 흩어 놓은 시작 위치 (n, 3). 서로 min_sep 이상 떨어지게 뽑는다."""
    rng = np.random.default_rng(100 + seed)
    pts = []
    while len(pts) < n:
        p = rng.uniform(-box, box, 2)
        if all(np.linalg.norm(p - q) >= min_sep for q in pts):
            pts.append(p)
    return np.c_[np.array(pts), np.zeros(n)]


def triangle_offsets(n=6, side=1.8):
    """정삼각형 대형: 꼭짓점 3개와 변의 중점 3개."""
    assert n == 6
    R = side / np.sqrt(3)
    c = [np.array([R * np.cos(a), R * np.sin(a), 0.0]) for a in np.pi / 2 + 2 * np.pi * np.arange(3) / 3]
    pts = [c[0], (c[0] + c[1]) / 2, c[1], (c[1] + c[2]) / 2, c[2], (c[2] + c[0]) / 2]
    return np.array(pts)



def ring_graph(n=6):
    A = np.zeros((n, n))
    for i in range(n):
        A[i, (i + 1) % n] = A[i, (i - 1) % n] = 1
    return A


def star_graph(n=6, center=0):
    A = np.zeros((n, n))
    A[center, :] = A[:, center] = 1
    A[center, center] = 0
    return A


def complete_graph(n=6):
    return np.ones((n, n)) - np.eye(n)


def split_graph(n=6):
    """끊긴 그래프. (0, 1, 2)와 (3, 4, 5) 두 무리 사이에 선이 없다."""
    A = np.zeros((n, n))
    h = n // 2
    for grp in (range(0, h), range(h, n)):
        for i in grp:
            for j in grp:
                if i != j:
                    A[i, j] = 1
    return A


GRAPHS = {"ring": ring_graph, "star": star_graph, "complete": complete_graph, "split": split_graph}



class FormationCentral:
    """중앙형 지상국.

    지상국은 드론마다 기준점 r_i를 들고, x_i = r_i − o_i (오프셋을 뺀 대형 중심 추정)에
    합의 규칙을 50 Hz로 돌린다. 드론은 자기 기준점을 따라간다 (setpoint = r_i).

    The ground station keeps a reference r_i for each drone and runs the consensus
    rule on x_i = r_i − o_i at 50 Hz. Each drone tracks its own reference.

    - x_i의 시작값은 카메라로 본 위치에서 오프셋을 뺀 값이다.
    - v_max: 기준점이 움직일 수 있는 최대 속도.
    - leash: 드론이 기준점을 이만큼 넘게 못 따라오면 기준점을 드론 쪽으로 당긴다.
    - center: 지휘자가 대형 중심을 정해 주면 모두 그쪽으로 조금씩 끌린다 (pinning).
    - consensus_step: 합의 한 걸음을 계산하는 함수. 3회차 수업에서는 부원이 쓴 함수를 넣는다.
    """

    def __init__(self, offsets, A, k=0.02, center=None, k_pin=0.01, v_max=0.5, dt=1 / 50, leash=0.4,
                 consensus_step=None):
        if consensus_step is None:                      # 3회차가 끝난 뒤에는 기준 구현을 쓴다
            from .ref.w3 import consensus_step
        self.consensus_step = consensus_step
        self.o = np.asarray(offsets, float)
        self.A = np.asarray(A, float)
        self.k, self.k_pin, self.v_max, self.dt, self.leash = k, k_pin, v_max, dt, leash
        self.center = None if center is None else np.asarray(center, float)
        self.X = None

    def set_offsets(self, offsets):
        """대형을 바꾼다. 기준점 위치는 그대로 두고 오프셋만 바꾼다."""
        offsets = np.asarray(offsets, float)
        if self.X is not None:
            self.X = self.X + self.o - offsets
        self.o = offsets

    def setpoints(self, P):
        P = np.asarray(P, float)
        if self.X is None:
            self.X = P - self.o
        Xn = self.consensus_step(self.X, self.A, self.k)
        if self.center is not None:
            Xn = Xn + self.k_pin * (self.center - self.X)
        step = Xn - self.X
        norm = np.linalg.norm(step, axis=1, keepdims=True)
        step = step * np.minimum(1.0, self.v_max * self.dt / np.maximum(norm, 1e-9))
        X = self.X + step
        dev = X + self.o - P                                   # 기준점이 드론보다 얼마나 앞서 있나
        dn = np.linalg.norm(dev, axis=1, keepdims=True)
        X = np.where(dn > self.leash, P - self.o + dev * self.leash / np.maximum(dn, 1e-9), X)
        self.X = X
        return X + self.o
