# ICARUS DSP 시뮬레이션 수업

[English](README.en.md)

ICARUS 드론 군집(Drone Swarm Project) 수업의 시뮬레이션 환경과 수업 자료다.
수업은 Ubuntu 24.04 노트북의 VS Code에서 한다. 설명을 들으면서 노트북의 빈칸(`___`)을 같이 채우거나
숫자를 바꿔 실행하고, 바로 나오는 그래프로 결과를 본다. 다섯 번의 수업이 끝나면 드론 6대가 대형을 바꾸며
서로 부딪히지 않고 날아다닌다.

| 회차 | 내용 | 상태 |
|---|---|---|
| 1 | 한 대를 띄우는 피드백 (PID) | 수업 당일 공개 |
| 2 | 다단 제어와 웨이포인트 | 수업 당일 공개 |
| 3 | 대형과 합의 (중앙형) | 수업 당일 공개 |
| 4 | ROS 2로 옮긴 분산 합의 | 수업 당일 공개 |
| 5 | 충돌 회피와 통합 챌린지 | 수업 당일 공개 |

설치(`setup/`)는 다섯 회차에 필요한 것을 처음에 전부 깐다. 회차 폴더(`week01/` 등)는 **수업 당일**에 올라오니,
그날 저장소 폴더에서 `git pull`로 받는다. 설치는 수업 전에 미리 끝내 둔다.

## 0. Claude에게 설치 맡기기 (선택)

VS Code에 Claude Code 확장이 있으면 아래 1~3절을 Claude가 대신 진행한다.
Claude가 이 폴더의 [`CLAUDE.md`](CLAUDE.md)를 읽고 설치 순서와 주의할 점을 알아서 따른다.
Claude 계정(Pro 이상 구독 또는 API 키)이 필요하다.

1. VS Code가 없으면 터미널에서 `sudo snap install code --classic`
2. VS Code 왼쪽 확장 탭(`Ctrl+Shift+X`)에서 **Claude Code**(Anthropic)를 설치하고 로그인
3. `파일 → 폴더 열기`로 이 폴더(`icarus-dsp-sim`)를 연다
4. Claude 창에 아래를 그대로 붙여 넣는다

```text
이 폴더는 ICARUS DSP 드론 수업 자료야. CLAUDE.md와 README.md를 읽고 내 노트북(Ubuntu 24.04)에 수업 환경을 설치해 줘.
먼저 setup/hw_profile.py로 내 노트북 사양을 보고 어떻게 설치할지 알려 줘.
비밀번호(sudo)가 필요한 명령은 내가 VS Code 터미널에서 직접 실행할 수 있게 알려 주고, 끝나면 setup/install.log를 읽어서 잘 됐는지 확인해 줘.
마지막으로 setup/check_env.py로 확인해 줘.
```

- 비밀번호는 Claude 창이 아니라 **터미널에만** 입력한다. Claude도 채팅으로 비밀번호를 묻지 않게 되어 있다.
- Claude가 명령을 실행해도 되냐고 물으면 내용을 읽어 보고 허락한다.
- 수업 중에도 Claude에게 오류를 물어볼 수 있다. 다만 빈칸(`___`)은 직접 채우는 칸이라서, 따로 부탁하지 않으면 Claude는 정답 대신 힌트를 준다.

## 1. 설치 (우분투 24.04를 막 깔았을 때 한 번)

터미널에서 이 저장소를 홈 폴더에 받고 설치 스크립트를 실행한다:

```bash
sudo apt install -y git
git clone https://github.com/baboseonho/icarus-dsp-sim.git ~/icarus-dsp-sim
cd ~/icarus-dsp-sim
bash setup/install.sh
```

git을 쓰기 어려우면 GitHub 페이지의 `Code → Download ZIP`으로 받아 홈 폴더에 풀어도 된다 (이때는 `git pull` 대신 새 회차마다 다시 받는다).

20~40분쯤 걸리고 대부분 내려받는 시간이다. 중간에 비밀번호(sudo)를 묻는다.
끊기면 같은 명령을 다시 실행하면 된다. 실행 기록은 `setup/install.log`에 남는다.

| 단계 | 내용 |
|---|---|
| 1 사양 확인 | CPU, 메모리, 디스크, GPU를 보고 설치 방법과 자원 상한을 정한다 (아래 "자원 상한") |
| 2 자원 상한 | 수업 코드가 노트북의 80%까지만 쓰도록 설정 |
| 3 기본 도구 | git, curl, 한글 글꼴(나눔), ffmpeg, graphviz, 3D 화면용 라이브러리 |
| 4 ROS 2 Jazzy | 4~5회차용. 디스크와 메모리가 넉넉하면 desktop판(rviz2, rqt 포함), 아니면 base판 |
| 5 Python | uv, `.venv`(Python 3.12), 물리 엔진 Crazyflow, numpy, matplotlib, Jupyter |
| 6 ROS 2 패키지 | `ros/dsp_ros` 빌드 (colcon) |
| 7 VS Code | Python, Jupyter 확장 포함. 이미 있으면 건너뛴다 (`--no-vscode`로 끌 수 있다). 별칭 `dsp` |
| 8 확인 | 그래프 글꼴 준비, 엔진 1대·6대와 ROS 2 시험 |

설치 전에 사양만 먼저 보고 싶으면 `python3 setup/hw_profile.py`. 디스크는 최소 6.5 GB, 넉넉하게는 9 GB 이상 비워 둔다.

## 2. 설치 확인

새 터미널을 열고:

```bash
dsp
python setup/check_env.py
```

모든 줄이 `OK`면 준비 끝이다 (3D 창 항목은 `WARN`이어도 수업에는 지장 없다).

## 3. 수업 당일

```bash
dsp
git pull       # 오늘 회차 폴더 받기 (예: week01/)
code .
```

VS Code에서 오늘 회차의 `lesson.ipynb`(예: `week01/lesson.ipynb`)를 연다. 오른쪽 위 커널이 `.venv (Python 3.12)`인지 확인한다.
첫 셀을 `Shift+Enter`로 실행해서 "시뮬레이터 준비 완료 · CPU …코어, 메모리 최대 … GB"가 나오면 된다.
수업 흐름은 그 회차 폴더의 `README.md`에 있다.

`git pull`에서 충돌이 나면, 지난 회차에 채운 노트북을 다른 이름으로 저장해 두고 다시 받는다.

`code .`를 꼭 `dsp`를 친 터미널에서 실행한다. 그래야 ROS 2 환경까지 VS Code가 물려받는다.

## 실시간으로 보기 (1회차부터)

회차마다 노트북 끝에 **실시간 비행** 셀(`live(...)`)이 있다. 드론을 1배속으로 계속 띄우고, 날고 있는 동안
슬라이더로 게인, 목표, 바람, 척력 값을 바꾸면 노트북 그래프가 바로 달라진다. ■ 멈춤으로 끝낸다. `window=True`를 넣으면 MuJoCo 3D 창도 뜬다.

같은 값을 **PlotJuggler**로도 볼 수 있다. 실제 드론 팀이 비행 기록을 볼 때 쓰는 그래프 도구다.

| 도구 | 여는 법 | 쓰는 회차 |
|---|---|---|
| 노트북 실시간 그래프와 슬라이더 | `live(...)` 셀 | 1~5 |
| PlotJuggler | `plotjuggler("week01")` 셀 → 창에서 **Yes → OK** | 1~5 (4~5회차는 ROS 2 토픽을 바로 그린다) |
| MuJoCo 3D 창 | `live(..., window=True)` | 1~5 |
| `ros2 topic echo/hz`, `ros2 param set` | `dsp`를 친 터미널 | 4~5 |
| rqt_reconfigure, rqt_graph, RViz2 | `dsp`를 친 터미널 (ROS 2 desktop판만) | 4~5 |
| `ros2 bag record` → PlotJuggler로 다시 보기 | `dsp`를 친 터미널 | 4~5 |

PlotJuggler, rqt, RViz는 노트북과 따로 도는 프로그램이라 아래 자원 상한 밖에서 돈다. 노트북이 버거우면 창을 하나씩만 연다.

## 자원 상한 (노트북의 80%)

수업 코드가 노트북을 다 잡아먹어서 화면이 멈추는 일이 없게, 시뮬레이터를 불러오는 순간 상한이 걸린다 ([`dsp/limits.py`](dsp/limits.py)).

- **CPU**: 전체 코어의 80%에만 붙는다 (예: 8코어면 6코어). 수업 코드가 띄우는 다른 프로세스(시뮬레이터, ROS 2 노드)도 같은 코어를 나눠 쓴다.
- **메모리**: 80%를 넘기 전에 실행 중인 셀을 멈추고 `⚠️ … 실행을 중단합니다`를 띄운다. 우분투 데스크톱에서는 systemd(`icarus-dsp.slice`)로 한 번 더 막는다.
- **GPU**: 계산에 쓰지 않는다. 물리 엔진은 CPU로 돌고, 3D 창은 화면 주사율까지만 그린다. 그래픽 드라이버나 CUDA를 따로 깔 필요 없다.
- **설치 중**: 내려받기와 빌드도 같은 코어 수, 낮은 우선순위로 돈다.

상한 비율은 설치 때 노트북 사양을 보고 `setup/hw_profile.env`에 적힌다. 노트북이 버거우면 낮출 수 있다:

```bash
DSP_LIMIT_PCT=70 bash setup/install.sh --no-sudo
```

## 폴더

```
icarus-dsp-sim/
├─ README.md         이 파일
├─ CLAUDE.md         Claude가 읽는 설치·수업 안내 (0절)
├─ setup/            ★ 시뮬레이션 환경 구축 (1~5회차 전부)
│  ├─ install.sh       설치
│  ├─ hw_profile.py    노트북 사양 분석, 설치 방법과 자원 상한 결정
│  ├─ env.sh           환경 불러오기 (`dsp`가 이걸 부른다)
│  └─ check_env.py     설치 확인
├─ weekNN/           회차 폴더 (수업 당일 공개): lesson.ipynb(수업 노트북) + background.py(배경 코드)
├─ dsp/              시뮬레이터 본체: 물리 엔진, 드론 칩, 카메라, 무선, 그래프 도구, 자원 상한
│  └─ ref/           끝난 회차의 기준 구현. 회차가 끝날 때마다 추가되고, 다음 회차 배경 코드가 부품으로 쓴다
├─ ros/dsp_ros/      ROS 2 패키지 (4~5회차)
├─ .vscode/          VS Code 설정 (.venv를 기본 Python으로)
└─ docs/             시스템 설계 개요, 물리 엔진 비교
│
```

기체는 Crazyflie급 `cf2x_T350`(37.9 g)이다. 실제로 쓸 25 cm급 기체의 질량과 모터가 정해지면 모델을 바꿔 다시 맞춘다.
시스템 구조와 ROS 2 토픽 약속은 [`docs/시스템_구조.md`](docs/시스템_구조.md)에 있다.

## 문제가 생기면

- 설치가 중간에 멈췄다: `setup/install.log` 끝부분을 보거나 Claude에게 "setup/install.log 읽고 원인 알려 줘"라고 한다.
- `사양이 부족해서 멈춥니다`: `python3 setup/hw_profile.py`의 `문제:` 줄을 본다. 대개 디스크 공간이다.
- 셀이 `⚠️ … 실행을 중단합니다`로 멈췄다: 메모리 상한에 걸렸다. 브라우저 탭 등 다른 프로그램을 닫고 커널을 재시작한다.
- 그래프 글자가 네모로 나온다: `sudo apt install fonts-nanum` 후 VS Code에서 커널 재시작.
- 커널 목록에 `.venv`가 없다: `dsp` 터미널에서 `code .`로 다시 연다.
- 우분투 24.04가 아니다: `setup/docker/`로 같은 환경을 컨테이너로 만든다.
