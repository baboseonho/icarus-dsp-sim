"""물리 엔진 감싸기.

부원은 이 파일을 열어 볼 필요가 없다. 주고받는 것은 numpy 배열뿐이다.
지금 쓰는 엔진은 Crazyflow(JAX 물리, first-principles 모터 모델)이고,
엔진을 바꾸게 되면 이 파일만 고치면 된다.

Wraps the physics engine so the rest of the course only sees numpy arrays.
You never need to read this file. The backend is currently Crazyflow (JAX
physics with a first-principles motor model); swapping engines means editing
only this file.

    eng = Engine(n=6, seed=0)
    eng.reset(init_pos)                       # (n,3)
    eng.step(cmds)                            # (n,4) = [총추력 N, τx, τy, τz Nm], body frame
    s = eng.true_state()                      # dict: t, p(n,3), q(n,4 wxyz), v(n,3), w(n,3 body)
"""
from __future__ import annotations

import os
import warnings

import numpy as np

from . import config as C

os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("XLA_FLAGS", "--xla_cpu_multi_thread_eigen=false")
warnings.filterwarnings("ignore", message=".*empty prefix.*")
warnings.filterwarnings("ignore", message=".*warp.*")


def _quiet_import():
    # JAX와 Crazyflow는 import할 때 경고를 잔뜩 찍어서 출력을 잠시 막아 둔다.
    import logging, contextlib, io
    logging.getLogger("jax").setLevel(logging.ERROR)
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        import jax  # noqa
        import jax.numpy as jnp  # noqa
        from crazyflow.sim import Sim  # noqa
        from crazyflow.control import Control  # noqa
        from crazyflow.dynamics import Dynamics  # noqa
        from crazyflow.drones import Drone  # noqa
    return jax, jnp, Sim, Control, Dynamics, Drone


def quat_wxyz_to_rpy(q):
    """쿼터니언 (w, x, y, z)를 (roll, pitch, yaw) [rad]로 바꾼다. (..., 4) 배열도 된다.

    Quaternion (w, x, y, z) to (roll, pitch, yaw) in radians. Works on (..., 4) arrays.
    """
    q = np.asarray(q, float)
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.stack([roll, pitch, yaw], -1)


def quat_wxyz_to_rotmat(q):
    """쿼터니언 (w, x, y, z)를 body에서 world로 가는 회전행렬 (..., 3, 3)로 바꾼다.

    Quaternion (w, x, y, z) to the body-to-world rotation matrix, shape (..., 3, 3).
    """
    q = np.asarray(q, float)
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    R = np.empty(q.shape[:-1] + (3, 3))
    R[..., 0, 0] = 1 - 2 * (y * y + z * z); R[..., 0, 1] = 2 * (x * y - w * z); R[..., 0, 2] = 2 * (x * z + w * y)
    R[..., 1, 0] = 2 * (x * y + w * z); R[..., 1, 1] = 1 - 2 * (x * x + z * z); R[..., 1, 2] = 2 * (y * z - w * x)
    R[..., 2, 0] = 2 * (x * z - w * y); R[..., 2, 1] = 2 * (y * z + w * x); R[..., 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def yaw_to_quat_wxyz(yaw):
    yaw = np.asarray(yaw, float)
    return np.stack([np.cos(yaw / 2), np.zeros_like(yaw), np.zeros_like(yaw), np.sin(yaw / 2)], -1)


class Engine:
    """드론 n대를 굴리는 물리 엔진.

    mass_scale은 실제 질량을 모델 질량으로 나눈 값이다. 1.1로 두면 "모델 질량이 10%
    틀린" 상황을 시험할 수 있다. wind는 일정한 바람 힘 [N], gust는 돌풍 세기 σ [N]이고,
    downwash를 켜면 위 드론의 하강 기류가 아래 드론을 누른다.

    Physics engine for n drones.

    Args:
        n: number of drones
        seed: RNG seed for the gusts
        mass_scale: true mass divided by model mass (1.1 = model is 10% too light)
        wind: constant wind force [N], shape (3,)
        gust: gust strength σ [N]
        downwash: let upper drones push lower ones down
    """

    def __init__(self, n=1, seed=0, mass_scale=1.0, wind=(0, 0, 0), gust=0.0, downwash=False,
                 drone=C.DRONE_MODEL, freq=C.PHYS_HZ):
        self.jax, self.jnp, self._Sim, self._Control, self._Dynamics, self._Drone = _quiet_import()
        self.freq = freq
        self.dt = 1.0 / freq
        self.drone = drone
        self.mass_scale = mass_scale
        self.wind = np.asarray(wind, float)
        self.gust = float(gust)
        self.downwash = bool(downwash)
        self.rng = np.random.default_rng(seed)
        self.n = 0
        self.sim = None
        self.reset(n)

    # 기체 정보

    @property
    def mass(self) -> float:
        """제어기가 알고 있는 모델 질량 [kg]. mass_scale은 반영되지 않는다.

        Nominal model mass the controller assumes (mass_scale not applied).
        """
        return self._mass_nominal

    @property
    def inertia(self) -> np.ndarray:
        """관성 모멘트 대각 성분 [kg m^2], (3,).

        Diagonal of the inertia matrix.
        """
        return self._J.copy()

    @property
    def max_thrust(self) -> float:
        return self._tmax

    @property
    def arm(self) -> float:
        return self._L

    # 초기화, 한 스텝, 상태 읽기

    def reset(self, n=None, init_pos=None, init_yaw=None):
        """드론 n대를 init_pos (n, 3)에 멈춘 상태로 놓는다.

        Place n drones at rest at init_pos (n, 3).
        """
        n = int(n if n is not None else (len(init_pos) if init_pos is not None else max(self.n, 1)))
        Sim, Control, Dynamics, Drone = self._Sim, self._Control, self._Dynamics, self._Drone
        if self.sim is None or n != self.n:
            if self.sim is not None:
                self.sim.close()
            self.sim = Sim(n_worlds=1, n_drones=n, drone=Drone(self.drone), control=Control.force_torque,
                           dynamics=Dynamics.first_principles, freq=self.freq, force_torque_freq=self.freq)
            self.n = n
            p = self.sim.data.params
            self._mass_nominal = float(np.asarray(p.mass).ravel()[0])
            self._J = np.diag(np.asarray(p.J).reshape(-1, 3, 3)[0]).copy()
            self._L = float(np.asarray(p.L).ravel()[0])
            import crazyflow.sim.sim as S
            lo, hi = S.rotor_vel_limits(self.sim.dynamics, self.sim.drone)
            k = np.asarray(p.rpm2thrust).reshape(-1)[:3]
            self._tmax = float(4 * (k[0] + k[1] * hi + k[2] * hi ** 2))
            self._tmin = float(4 * (k[0] + k[1] * lo + k[2] * lo ** 2))
            self._build_fused()
            self._mass_param = p.mass
            self.set_mass_scale(self.mass_scale)
        self.sim.reset()
        if init_pos is None:
            init_pos = np.zeros((n, 3))
            init_pos[:, 0] = np.arange(n) * 0.5
        init_pos = np.asarray(init_pos, float).reshape(n, 3)
        yaw = np.zeros(n) if init_yaw is None else np.asarray(init_yaw, float).reshape(n)
        q_wxyz = yaw_to_quat_wxyz(yaw)
        q_xyzw = q_wxyz[:, [1, 2, 3, 0]]
        st = self.sim.data.states
        jnp = self.jnp
        # 공중에서 시작하는 드론은 모터가 이미 호버 회전수로 돌고 있는 상태로 둔다.
        # 회전수 0에서 시작하면 모터가 따라 올라오는 동안 떨어진다.
        rv = np.zeros((n, 4))
        rv[init_pos[:, 2] > 0.02] = self._hover_rotor_vel()
        self.sim.data = self.sim.data.replace(states=st.replace(
            pos=jnp.asarray(init_pos[None], dtype=st.pos.dtype),
            quat=jnp.asarray(q_xyzw[None], dtype=st.quat.dtype),
            rotor_vel=jnp.asarray(rv[None], dtype=st.rotor_vel.dtype)))
        self.t = 0.0
        self.steps = 0
        self._gust_state = np.zeros((n, 3))
        self._last_thrust = np.zeros(n)
        self.ground_impacts = []          # (t, 드론 번호, 접지 속도 vz)
        self._on_ground = init_pos[:, 2] <= 0.02
        self._cache = None
        self._ext = np.zeros((n, 3))
        return self.true_state()

    def _hover_rotor_vel(self):
        """모델 질량으로 떠 있을 때 모터 하나의 회전수."""
        k = np.asarray(self.sim.data.params.rpm2thrust).reshape(-1)[:3]
        f = self._mass_nominal * C.G / 4
        a, b, c = k[2], k[1], k[0] - f
        return float((-b + np.sqrt(b * b - 4 * a * c)) / (2 * a))

    def set_mass_scale(self, mass_scale):
        """실제 질량 = 모델 질량 × mass_scale로 바꾼다. 다시 컴파일하지 않으므로 엔진을 새로 만드는 것보다 빠르다.

        Change the true mass without recompiling (cheaper than building a new engine).
        """
        self.mass_scale = float(mass_scale)
        m = self._mass_param * self.mass_scale
        self.sim.data = self.sim.data.replace(params=self.sim.data.params.replace(mass=m))
        self.sim.default_data = self.sim.default_data.replace(params=self.sim.default_data.params.replace(mass=m))

    def _build_fused(self):
        # 제어 입력, 외력, 물리 한 스텝, 상태 읽기를 jit 함수 하나로 묶는다.
        # 따로 부르면 스텝마다 JAX 호출이 여러 번 생겨 꽤 느려진다.
        jax = self.jax
        from crazyflow.sim import functional as F
        step_fn = self.sim._step

        def fused(data, cmd, force):
            data = data.replace(states=data.states.replace(force=force))
            data = F.force_torque_control(data, cmd)
            data = step_fn(data, n_steps=1)
            s = data.states
            return data, (s.pos[0], s.quat[0], s.vel[0], s.ang_vel[0])

        self._fused = jax.jit(fused)

    def _set_cache(self, pos, quat, vel, w):
        self._cache = dict(t=self.t, p=np.array(pos, float), q=np.array(quat, float)[:, [3, 0, 1, 2]],
                           v=np.array(vel, float), w=np.array(w, float))

    def true_state(self) -> dict:
        """엔진이 가진 진짜 상태. 지상국은 이 값을 볼 수 없고 카메라 값만 본다.

        Ground-truth state. The ground station never sees this, only the camera.
        """
        if self._cache is None:
            s = self.sim.data.states
            pos, quat, vel, w = self.jax.device_get((s.pos[0], s.quat[0], s.vel[0], s.ang_vel[0]))
            self._cache = dict(t=self.t, p=np.array(pos, float), q=np.array(quat, float)[:, [3, 0, 1, 2]],
                               v=np.array(vel, float), w=np.array(w, float))
        return self._cache

    def _external_force(self, st) -> np.ndarray:
        n = self.n
        f = np.tile(self.wind, (n, 1))
        if self.gust > 0:   # Ornstein-Uhlenbeck 돌풍, 시간 상수 1 s
            a = self.dt / 1.0
            self._gust_state += -a * self._gust_state + np.sqrt(2 * a) * self.gust * self.rng.standard_normal((n, 3))
            f += self._gust_state
        if self.downwash and n > 1:
            f[:, 2] += downwash_force(st["p"], self._last_thrust)
        return f

    def step(self, cmds) -> None:
        """물리를 한 스텝(1/freq 초) 진행한다.

        cmds: (n, 4) [총추력 N, τx, τy, τz Nm], body frame.

        Advance one physics step. cmds is (n, 4): collective thrust [N] and
        body torques [Nm].
        """
        cmds = np.asarray(cmds, float).reshape(self.n, 4)
        st = self.true_state()
        thrust = np.clip(cmds[:, 0], 0.0, self._tmax)
        self._last_thrust = thrust
        ext = self._external_force(st)
        # 바닥에 앉아 있고 추력이 무게보다 작으면 바람에 밀려 다니지 않게 외력을 끈다
        ext[self._on_ground & (thrust < self._mass_nominal * self.mass_scale * C.G)] = 0.0
        cmd = np.concatenate([thrust[:, None], cmds[:, 1:]], 1)[None].astype(np.float32)
        self.sim.data, out = self._fused(self.sim.data, cmd, ext[None].astype(np.float32))
        self.steps += 1
        self.t = self.steps * self.dt
        self._set_cache(*self.jax.device_get(out))
        self._ground_contact(thrust)

    def _ground_contact(self, thrust):
        # Crazyflow 드론에는 충돌체가 없어서 바닥을 여기서 직접 만든다.
        # z < 0으로 내려가거나, 바닥에 앉은 채 추력이 무게보다 작으면 z=0에 세워 둔다.
        st = self.true_state()
        below = st["p"][:, 2] < 0.0
        weight = self._mass_nominal * self.mass_scale * C.G
        resting = self._on_ground & (thrust < weight)
        fix = below | resting
        if not np.any(fix):
            self._on_ground[:] = False
            return
        for i in np.where(below & ~self._on_ground)[0]:
            if st["v"][i, 2] < -0.1:     # 살짝 닿은 것은 빼고 실제 접지만 기록
                self.ground_impacts.append((self.t, int(i), float(st["v"][i, 2])))
        p, v, q, w = st["p"].copy(), st["v"].copy(), st["q"].copy(), st["w"].copy()
        p[fix, 2] = 0.0
        v[fix] = 0.0
        w[fix] = 0.0
        yaw = quat_wxyz_to_rpy(q[fix])[:, 2]
        q[fix] = yaw_to_quat_wxyz(yaw)
        self._on_ground = fix.copy()
        jnp = self.jnp
        s = self.sim.data.states
        self.sim.data = self.sim.data.replace(states=s.replace(
            pos=jnp.asarray(p[None], dtype=s.pos.dtype), vel=jnp.asarray(v[None], dtype=s.vel.dtype),
            quat=jnp.asarray(q[:, [1, 2, 3, 0]][None], dtype=s.quat.dtype),
            ang_vel=jnp.asarray(w[None], dtype=s.ang_vel.dtype)))
        self._cache = None

    # 화면

    def render(self, width=960, height=540, mode="rgb_array"):
        """MuJoCo로 그린다. mode="human"이면 3D 창을 띄운다.

        Render with MuJoCo; mode="human" opens a 3D window.
        """
        cam = {"distance": 3.6, "elevation": -35.0, "azimuth": 135.0, "lookat": np.array([0.0, 0.0, 0.8])}
        return self.sim.render(mode=mode, width=width, height=height, cam_config=cam)

    def close(self):
        if self.sim is not None:
            self.sim.close()


def downwash_force(p, thrust, k=0.35, r0=0.06, spread=0.12, z_decay=0.6):
    """하강 기류 모델. 단순한 경험식이라 실기체로 보정해야 한다.

    위에 있는 드론 j가 아래 드론 i를 수직으로 누른다. 세기는 수평 거리 r에 대해
    가우시안이고(높이 차 dz가 클수록 넓어진다), dz에 대해서는 지수적으로 줄어든다.
    반환값은 드론별 z 방향 힘 (n,) [N]이며 항상 0 이하다.

    Simple empirical downwash model, still to be calibrated on real drones.
    An upper drone j pushes drone i straight down. The push is Gaussian in the
    horizontal distance r (wider for larger height gap dz) and decays
    exponentially with dz. Returns the z-force per drone, shape (n,), always <= 0.
    """
    p = np.asarray(p, float)
    d = p[None, :, :] - p[:, None, :]            # d[i,j] = p_j - p_i
    dz = d[..., 2]
    r = np.hypot(d[..., 0], d[..., 1])
    sig = r0 + spread * np.clip(dz, 0, None)
    mask = (dz > 0.0) & (dz < 2.0)
    contrib = k * np.asarray(thrust)[None, :] * np.exp(-0.5 * (r / sig) ** 2) * np.exp(-np.clip(dz, 0, None) / z_decay)
    return -np.sum(np.where(mask, contrib, 0.0), axis=1)
