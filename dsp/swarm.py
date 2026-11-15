"""5회차에 미리 주는 부품: 시나리오 상태 기계의 뼈대와 드론 한 대의 유도 계산(SwarmGuidance).

부원이 채우는 부분(가장 가까운 쌍, 척력, 속도 합치기, 상태 전이표)은 여기 없다.
SwarmGuidance에는 척력과 속도 합치기 함수를 바깥에서 넣는다.

Provided week-5 parts: the scenario state-machine skeleton and per-drone guidance.
The repulsion and velocity-combining functions are passed in.
"""
import numpy as np


class ScenarioBase:
    """최종 시나리오의 상태 기계.

        WAIT → TAKEOFF → FORM(line) → FORM(hexagon) → FORM(triangle) → LAND → DONE
        (비행 중 위험하면 EMERGENCY)

    여기는 미리 주는 부분이다(대형 순서, 정착 판정, 비상 정지). 상태 전이표는 transitions()에 채운다.
    지휘자(commander)가 step(t, obs)를 20 Hz로 부르고, 돌려받은 행동 목록을 그대로 실행한다.
    obs = {"ready", "form_err", "dmin", "z", "P"}. 행동은 다음 중 하나다.
        ("takeoff", h, dur)   ("formation", name, center)   ("land", dur)   ("emergency",)   ("done",)

    Mission state machine. The commander calls step(t, obs) at 20 Hz and executes
    the list of actions it returns. Each transition changes the state with
    self.go(...) and returns the matching actions.
    """

    def __init__(self, sequence=("line", "hexagon", "triangle"), takeoff_h=1.0, center=(0.0, 0.0, 1.0),
                 settle_err=0.05, settle_time=2.0, min_hold=4.0, form_timeout=45.0, d_emergency=0.12,
                 geofence=(2.5, 2.5, 2.5)):
        self.seq = list(sequence)
        self.h, self.center = takeoff_h, list(center)
        self.settle_err, self.settle_time, self.min_hold = settle_err, settle_time, min_hold
        self.form_timeout, self.d_emergency, self.fence = form_timeout, d_emergency, np.asarray(geofence)
        self.state, self.t_state, self.k = "WAIT", 0.0, 0
        self.t_good = None                 # 대형 오차가 기준 아래로 내려온 시각
        self.log = []                      # (t, 상태, 이유)

    @property
    def current_formation(self):
        return self.seq[self.k] if self.state == "FORM" else None

    def go(self, state, t, why=""):
        self.state, self.t_state, self.t_good = state, t, None
        self.log.append((round(t, 2), state if state != "FORM" else f"FORM:{self.seq[self.k]}", why))

    def settled(self, t, obs):
        """대형 오차가 settle_err보다 작은 상태로 settle_time 동안 유지됐는가.

        True once the formation error has stayed below settle_err for settle_time seconds.
        """
        if obs["form_err"] < self.settle_err:
            if self.t_good is None:
                self.t_good = t
            return t - self.t_good >= self.settle_time
        self.t_good = None
        return False

    def step(self, t, obs):
        dt = t - self.t_state
        flying = self.state in ("TAKEOFF", "FORM")
        # 비상 정지는 다른 어떤 전이보다 먼저 본다: 너무 가깝거나 울타리 밖이면 바로 멈춘다.
        if flying and obs["ready"]:
            out = np.any(np.abs(obs["P"][:, :2]) > self.fence[:2]) or np.any(obs["P"][:, 2] > self.fence[2])
            if obs["dmin"] < self.d_emergency or out:
                self.go("EMERGENCY", t, f"dmin={obs['dmin']:.2f}" if not out else "geofence")
                return [("emergency",)]
        if self.state == "EMERGENCY" and dt > 3.0:
            self.go("DONE", t, "after emergency")
            return [("done",)]
        return self.transitions(t, obs, dt)

    def transitions(self, t, obs, dt):
        """④ 상태 전이표. 5회차 수업에서 채운다. 행동 리스트를 돌려준다."""
        raise NotImplementedError("④ transitions")


class SwarmGuidance:
    """드론 한 대를 맡는 지상국 쪽 유도 계산. 3회차 합의, 대형 중심 끌림, 5회차 APF를 합친다.
    ROS 2 agent와 순수 Python 판이 같은 클래스를 쓴다.

    기준점 r_i = x_i + o_i를 50 Hz로 움직인다.
      v_att: 합의와 중심 끌림으로 정한 x의 변화량 / dt (인력)
      v_rep: 지금 위치로 본 척력과 tau초 뒤를 예측한 척력 중 가장 큰 것, 여기에 회전 성분을 더한다
      v    : apf_velocity(v_att, v_rep, v_lim)를 가속도 a_max로 부드럽게 한 뒤 x ← x + v·dt
    예측을 쓰는 이유: 다가오는 드론은 관성 때문에 바로 서지 못한다. 다른 드론의 속도는 위치 차분으로 추정한다.
    회전 성분 k_tan·(ẑ × v_rep): 정면으로 마주 오면 둘 다 오른쪽으로 비켜서 서로 막혀 멈추는 일(지역 최소)을 피한다.

    Ground-station guidance for one drone: week-3 consensus, a weak pull toward the
    formation centre, and the week-5 APF. The ROS 2 agent and the pure-Python version
    share this class. Repulsion is also evaluated tau seconds ahead, because an
    approaching drone cannot stop instantly, and a tangential term makes head-on
    pairs both sidestep to the right instead of stalling in a local minimum.
    """

    def __init__(self, i, n, A, k=0.02, k_pin=0.01, v_max=0.3, a_max=1.0, dt=1 / 50, leash=0.4,
                 r_safe=0.75, k_rep=0.12, k_tan=1.0, tau=1.0, use_apf=True, slow=True,
                 repulsion_fn=None, apf_fn=None):
        self.i, self.n, self.A = i, n, np.asarray(A, float)
        self.k, self.k_pin, self.v_max, self.a_max, self.dt, self.leash = k, k_pin, v_max, a_max, dt, leash
        self.r_safe, self.k_rep, self.k_tan, self.tau, self.use_apf = r_safe, k_rep, k_tan, tau, use_apf
        self.slow = slow and use_apf
        if repulsion_fn is None or apf_fn is None:       # 5회차가 끝난 뒤에는 기준 구현을 쓴다
            from .ref import w5 as _w5
            repulsion_fn, apf_fn = repulsion_fn or _w5.repulsion, apf_fn or _w5.apf_velocity
        from .ref.w3 import consensus_step              # 3회차 합의 한 걸음
        self.consensus_step = consensus_step
        self.repulsion = repulsion_fn                    # 수업 노트북에서 직접 쓴 함수를 넣는다
        self.apf_velocity = apf_fn
        self.o = None
        self.center = None
        self.x = None
        self.v = np.zeros(3)
        self.neighbors = [j for j in np.nonzero(self.A[i])[0] if j != i]
        self._seen = {}                       # j -> (t, p, v), 다른 드론의 속도 추정용

    def set_formation(self, offsets, center=None):
        offsets = np.asarray(offsets, float)
        if self.x is not None and self.o is not None:
            self.x = self.x + self.o[self.i] - offsets[self.i]
        self.o = offsets
        self.center = None if center is None else np.asarray(center, float)

    def reset(self):
        self.x = None
        self.v = np.zeros(3)

    def _velocity(self, j, p, t):
        prev = self._seen.get(j)
        v = np.zeros(3)
        if prev is not None and t > prev[0] + 1e-6:
            v = 0.7 * prev[2] + 0.3 * (p - prev[1]) / (t - prev[0])
        self._seen[j] = (t, p, v)
        return v

    def update(self, p_me, P_nb, P_others, t):
        """이번 틱의 내 setpoint (3,)를 돌려준다.

        p_me는 내 위치, P_nb는 {j: 이웃 위치}, P_others는 {j: 나를 뺀 모든 드론 위치}, t는 시각.

        Returns this tick's setpoint for drone i. P_nb holds neighbour positions and
        P_others holds every other drone's position, both keyed by drone index.
        """
        o = self.o
        p_me = np.asarray(p_me, float)
        if self.x is None:
            self.x = p_me - o[self.i]
        js = [j for j in self.neighbors if j in P_nb]
        X_loc = np.vstack([self.x] + [P_nb[j] - o[j] for j in js])
        A_loc = np.zeros((len(X_loc), len(X_loc)))
        A_loc[0, 1:] = self.A[self.i, js]
        x_c = self.consensus_step(X_loc, A_loc, self.k)[0]
        if self.center is not None:
            x_c = x_c + self.k_pin * (self.center - self.x)
        v_att = (x_c - self.x) / self.dt
        r = self.x + o[self.i]
        v_rep = np.zeros(3)
        if self.use_apf and P_others:
            ids = list(P_others)
            Q = np.array([P_others[j] for j in ids])
            V = np.array([self._velocity(j, Q[m], t) for m, j in enumerate(ids)])
            v_me = self._velocity(-1, p_me, t)           # 카메라 위치를 차분한 내 실제 속도
            cands = [self.repulsion(r, Q, self.r_safe, self.k_rep),                                   # 지금 기준점
                     self.repulsion(r + self.v * self.tau, Q + V * self.tau, self.r_safe, self.k_rep),  # tau초 뒤 기준점
                     self.repulsion(p_me + v_me * self.tau, Q + V * self.tau, self.r_safe, self.k_rep)]  # tau초 뒤 실제 위치
            v_rep = max(cands, key=np.linalg.norm)
            v_rep = v_rep + self.k_tan * np.cross([0.0, 0.0, 1.0], v_rep)
            v_rep[2] *= 0.3                       # downwash 때문에 위아래로는 덜 비킨다
        v_lim = self.v_max
        if self.slow and P_others:                 # 가까운 드론이 있으면 천천히 움직인다
            d_near = min(np.linalg.norm(np.array(list(P_others.values())) - p_me, axis=1).min(),
                         np.linalg.norm(np.array(list(P_others.values())) - r, axis=1).min())
            v_lim = self.v_max * float(np.clip((d_near - 0.25) / 0.5, 0.3, 1.0))
        v = self.apf_velocity(v_att, v_rep, v_lim)
        dv = v - self.v                           # 기준점의 가속도를 a_max로 제한
        n_dv = np.linalg.norm(dv)
        if n_dv > self.a_max * self.dt:
            v = self.v + dv * self.a_max * self.dt / n_dv
        x_new = self.x + v * self.dt
        dev = x_new + o[self.i] - p_me
        d = np.linalg.norm(dev)
        if d > self.leash:
            x_new = p_me - o[self.i] + dev * self.leash / d
        self.v = (x_new - self.x) / self.dt
        self.x = x_new
        return x_new + o[self.i]
