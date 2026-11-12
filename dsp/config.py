"""모든 회차가 함께 쓰는 상수와 설정값.

좌표계는 world ENU, 단위는 미터·초·라디안이다. 드론 이름은 cf1부터 cf6까지.

Shared constants and settings used by every week. World frame is ENU,
units are SI (metres, seconds, radians). Drones are named cf1 to cf6.
"""
from __future__ import annotations

from dataclasses import dataclass, field

G = 9.81                       # 중력가속도 [m/s^2]
NAMES = [f"cf{i}" for i in range(1, 7)]

# 루프 주기 [Hz]
PHYS_HZ = 500                  # 물리 엔진
ATT_HZ = 500                   # 칩의 자세 루프
POS_HZ = 100                   # 칩의 위치 루프
LINK_HZ = 50                   # 지상국에서 드론으로 보내는 setpoint
MOCAP_HZ = 100                 # 천장 카메라

DRONE_MODEL = "cf2x_T350"      # Crazyflow 기체 (37.9 g, 추력/무게 약 1.9). ESP32-S3 마이크로 드론과 비슷한 급
SAFE_DIST = 0.30               # 기체 사이 최소 거리 기준 [m]
ARENA = (-2.5, 2.5, -2.5, 2.5, 0.0, 2.5)   # 실내 비행 공간 (xmin, xmax, ymin, ymax, zmin, zmax)


@dataclass
class Realism:
    """현실 조건 스위치. 기본값은 6장에서 정한 최종 기준이다.

    Realism switches (noise, delay, loss, wind). Defaults are the final
    settings from chapter 6.
    """
    mocap_noise: float = 0.005     # 위치 잡음 σ [m]
    mocap_delay: float = 0.020     # [s]
    mocap_drop: float = 0.01       # 샘플을 놓칠 확률
    mocap_hz: float = MOCAP_HZ
    link_hz: float = LINK_HZ
    link_delay: float = 0.010      # 무선 지연 [s]
    link_loss: float = 0.02        # 패킷 손실률
    downwash: bool = True
    wind: tuple = (0.0, 0.0, 0.0)  # 일정한 바람 힘 [N]
    gust: float = 0.0              # 돌풍 세기 σ [N]

    @classmethod
    def ideal(cls) -> "Realism":
        """잡음·지연·손실이 없는 설정 (1~2회차용).

        No noise, delay or loss (used in weeks 1 and 2).
        """
        return cls(0.0, 0.0, 0.0, MOCAP_HZ, LINK_HZ, 0.0, 0.0, False)


@dataclass
class ScenarioSpec:
    takeoff_h: float = 1.0
    sequence: list = field(default_factory=lambda: ["line", "hexagon", "triangle"])
    center: tuple = (0.0, 0.0, 1.0)
