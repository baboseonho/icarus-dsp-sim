[English](README.en.md)

# dsp_ros

4·5회차에서 쓰는 ROS 2 패키지. 시뮬레이터, 가상 천장 카메라, 무선 링크, 드론별 agent, 지휘자(commander)가 각각 노드로 돈다.

```
sim_bridge ──/sim/truth──▶ mocap ──/cfN/pose──┬──▶ agent_cfN ──/cfN/setpoint──▶ link ──UDP──▶ 드론 칩 (sim_bridge 안)
                                              └──▶ commander ──/cfN/command───▶ link
                                                       └──/swarm/formation──▶ agent_cfN
```

## 노드

| 노드 | 하는 일 |
|---|---|
| `sim_bridge` | 엔진과 드론 6대의 칩(2회차 `Onboard`)을 실시간으로 돌린다. 명령은 진짜 UDP 로 받고, 실제 위치를 `/sim/truth` 로 낸다. `log_path` 를 주면 궤적을 npz 로 저장한다. |
| `mocap` | `/sim/truth` 에 잡음, 지연, 놓침을 섞어 `/cfN/pose` 로 낸다. 천장 카메라 역할. |
| `link` | 지상국 쪽 무선. `/cfN/setpoint`, `/cfN/command`, `/swarm/emergency` 를 받아 UDP 패킷으로 보낸다. |
| `agent_cfN` | 드론 한 대의 분산 제어. 부원이 `weekNN/exercise.py` 에 채운 `Agent` 클래스를 그대로 불러온다 (`DSP_SRC` 로 정답이나 제출본 선택). |
| `commander` | 이륙, 대형, 착륙, 비상 정지를 정한다. 4회차는 `W4Machine`, 최종 시나리오는 5회차 `ScenarioMachine`. 끝나면 launch 전체가 내려간다. |

## 토픽

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/cfN/pose` | `PoseStamped` | 드론 위치, 100 Hz |
| `/cfN/setpoint` | `PoseStamped` | agent 가 내는 목표점, 50 Hz |
| `/cfN/command` | `String` (JSON) | `{"type": "takeoff" \| "go_to" \| "land" \| "emergency_stop", ...}` |
| `/swarm/formation` | `String` (JSON) | `{"name": "hexagon", "center": [x, y, z] 또는 null}` |
| `/swarm/emergency` | `Empty` | 모든 드론 비상 정지 |
| `/swarm/status` | `String` (JSON) | commander 상태 (`state`, `formation`, `t`, `form_err`, `dmin`) |
| `/sim/truth` | `PoseArray` | 시뮬 전용. 엔진의 실제 위치 |
| `/mocap/disable` | `String` | 시험용. `"cf3"` 을 보내면 cf3 의 pose 가 끊기고, `""` 이면 다시 나온다 |

`N` 은 1~6. ROS 2 는 노트북 안에서만 쓰고 드론과는 UDP 로만 이야기한다.

## 실행

```bash
ros2 launch dsp_ros swarm.launch.py                          # 4회차: 육각형 분산 합의
ros2 launch dsp_ros swarm.launch.py week:=5 scenario:=final  # 최종 시나리오
```

보통은 `week04/run.py`, `week05/run.py --mode ros` 가 알맞은 인자를 붙여 대신 띄워 준다.

| 인자 | 기본값 | 뜻 |
|---|---|---|
| `week` | `4` | agent 를 몇 회차 `exercise.py` 에서 불러올지 |
| `graph` | `ring` | 통신 그래프: `ring`, `star`, `complete`, `split` |
| `scenario` | `w4` | `w4` 또는 `final` |
| `formation` | `hexagon` | `w4` 에서 만들 대형: `line`, `hexagon`, `triangle` |
| `hold` | `30.0` | `w4` 에서 대형을 유지하는 시간 [s] |
| `center` | (없음) | `w4` 대형 중심 `"x,y,z"` |
| `seed` | `0` | 시뮬, mocap, link 가 같이 쓰는 난수 시드 |
| `init` | `grid` | 시작 배치: `grid` (2×3 격자) 또는 `scatter` |
| `loss` | `0.02` | UDP 패킷 손실률 |
| `noise` | `0.005` | mocap 위치 잡음 표준편차 [m] |
| `delay` | `0.020` | mocap 지연 [s] |
| `drop` | `0.01` | mocap 이 한 프레임을 놓칠 확률 |
| `link_delay` | `0.010` | 무선 지연 [s] |
| `downwash` | `true` | 위 드론의 하강 기류가 아래 드론을 누르는 효과 |
| `bag` | (없음) | 주면 그 이름으로 `ros2 bag record` |
| `log_path` | (없음) | 주면 sim_bridge 가 궤적을 이 npz 에 저장 |

## 빌드

저장소 루트에서 venv 를 켠 상태로 빌드한다.

```bash
source setup/env.sh
cd ros
python -m colcon build --symlink-install
```

`colcon` 을 venv 의 `python` 으로 돌리는 이유: 이때 쓴 파이썬이 노드 실행 파일의 shebang 에 들어간다. 시스템 `python3` 로 빌드하면 노드가 crazyflow 같은 venv 패키지가 없는 파이썬으로 떠서 import 에서 바로 죽는다. `--symlink-install` 이라 `.py` 를 고치면 다시 빌드할 필요가 없고, `setup.py` 의 실행 파일 목록을 바꿨을 때만 다시 빌드하면 된다. 처음 빌드한 뒤에는 `source setup/env.sh` 를 한 번 더 해야 `ros2` 가 패키지를 찾는다.

## 실기체로 갈 때

| 시뮬 | 실기체 |
|---|---|
| `sim_bridge` | 없어진다. 진짜 드론과 카메라가 그 자리를 채운다. |
| `mocap` | 같은 `/cfN/pose` 를 내는 OpenCV ArUco 노드 |
| `link` 안의 `UdpLink` | `EspUdpLink` |
| `agent_cfN`, `commander` | 그대로 |
