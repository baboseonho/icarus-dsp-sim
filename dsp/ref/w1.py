"""1회차 기준 구현: 호버 추력, PID, 고도 제어기.

2회차부터는 이 파일의 PID를 배경 코드가 가져다 쓴다.
Week-1 reference: hover thrust, PID and the altitude controller.
"""
import numpy as np

G = 9.81  # 중력가속도 [m/s^2]


def hover_thrust(mass, g=G):
    """제자리에 떠 있는 데 필요한 총추력 [N].

    Collective thrust needed to hover [N].
    """
    return mass * g


class PID:
    """이산 PID 제어기.

    error는 float여도 되고 numpy 배열(예: 3축)이어도 된다. 배열이면 축마다 따로 계산된다.

    Discrete PID controller. `error` may be a float or a numpy array (e.g. three axes);
    arrays are handled element-wise.
    """

    def __init__(self, kp, ki=0.0, kd=0.0, i_limit=None, i_band=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit = i_limit        # |∫e dt| ≤ i_limit, None이면 자르지 않음
        self.i_band = i_band          # |e| < i_band 일 때만 적분, None이면 항상 적분
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_error = None        # 미분에 쓸 직전 오차

    def update(self, error, dt):
        """오차를 받아 이번 스텝의 제어 출력을 돌려준다.

        Takes the current error and returns this step's control output.
        """
        # 오차가 i_band 이상인 축은 이번 스텝에 적분하지 않는다.
        # Do not integrate on axes where |error| >= i_band.
        e_int = error
        if self.i_band is not None:
            e_int = np.where(np.abs(error) < self.i_band, error, 0.0)

        # 적분: 매 스텝 e_int·dt 를 더한다 (사각형 근사).
        # Integral: add e_int·dt every step (rectangle rule).
        self.integral = self.integral + e_int * dt

        # 적분값을 [-i_limit, i_limit] 로 자른다. i_limit 이 None 이면 그대로.
        # Clip the integral to [-i_limit, i_limit]; leave it alone when i_limit is None.
        if self.i_limit is not None:
            self.integral = np.clip(self.integral, -self.i_limit, self.i_limit)

        # 미분: (error − 직전 오차) / dt. 첫 호출이라 직전 오차가 없으면 0.
        # Derivative: (error − previous error) / dt, and 0 on the first call.
        if self.prev_error is None:
            derivative = 0.0 * error
        else:
            derivative = (error - self.prev_error) / dt
        self.prev_error = error

        # 출력 = P + I + D
        # Output = P + I + D
        return self.kp * error + self.ki * self.integral + self.kd * derivative


class AltitudeController:
    """고도 제어기. 추력 = 호버 추력 + m · PID(오차).

    Altitude controller: thrust = hover thrust + m · PID(error).
    """

    def __init__(self, mass, kp, ki=0.0, kd=0.0, i_limit=None, i_band=None):
        self.mass = mass              # 컨트롤러가 알고 있는 '모델' 질량. 실제 질량과 다를 수 있다.
        self.pid = PID(kp, ki, kd, i_limit, i_band)

    def reset(self):
        self.pid.reset()

    def update(self, z_target, z, dt):
        # 오차의 부호: 위로 가야 할 때 양수다.
        # Error sign: it must be positive when the drone has to go up.
        error = z_target - z
        acc = self.pid.update(error, dt)          # 원하는 위쪽 가속도 [m/s^2]
        return hover_thrust(self.mass) + self.mass * acc


# 1회차 수업에서 고른 게인. 실제 질량이 모델보다 10 % 무거워도 오버슈트 < 10 %, 정착 < 2 s, 처짐 < 2 cm.
MY_GAINS = dict(kp=10.0, ki=4.0, kd=5.0, i_limit=0.3, i_band=0.15)
