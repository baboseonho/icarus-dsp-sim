# ICARUS DSP 수업 저장소 (Claude용 안내)

이 저장소는 ICARUS 동아리의 드론 군집 수업 자료다. 부원은 Ubuntu 24.04 노트북의 VS Code에서
`weekNN/lesson.ipynb`를 열고, 수업을 들으면서 빈칸(`___`)을 채우거나 숫자를 바꿔 실행해 그래프를 본다.
부원 대부분은 리눅스와 제어 이론이 처음이다. 설명은 짧고 쉬운 한국어로, 명령은 복사해서 바로 붙여 넣을 수 있게 준다.

## 설치를 부탁받으면

목표: `setup/install.sh`가 끝까지 돌고, `setup/check_env.py`가 전부 OK인 것.
회차 폴더(`week01/` 등)는 수업 당일에 올라오므로 설치 때는 없어도 정상이다.

1. **현재 상태와 노트북 사양 확인**
   - `python3 setup/hw_profile.py`로 사양(CPU, 메모리, 디스크, GPU)과 판단 결과를 보고 부원에게 짧게 알려 준다.
     `문제:` 줄이 있으면(디스크·메모리 부족) 설치 전에 해결 방법을 같이 찾는다.
   - `cat /etc/os-release`로 Ubuntu 24.04인지 본다. 다른 버전이면 설치하지 말고 부원에게 알린다 (`setup/docker/`가 대안이지만 강사와 먼저 상의).
   - 이미 설치된 것이 있는지 본다: `/opt/ros/jazzy/setup.bash`, `.venv/bin/python`, `ros/install/`, `setup/install.log`.
   - `setup/install.log`가 있으면 끝부분부터 읽는다. `##### 설치 완료`가 마지막이면 4번으로 간다.

2. **설치 실행 (sudo 비밀번호가 필요하다)**
   - 먼저 `sudo -n true`로 비밀번호 없이 sudo가 되는지 본다.
   - 된다면: `bash setup/install.sh`를 직접 실행한다. 20~40분 걸리므로 백그라운드로 돌리고 `setup/install.log`를 보며 기다린다.
   - 안 된다면 (보통 이쪽이다): 부원에게 VS Code 터미널(`` Ctrl+` ``)에서 아래 한 줄을 실행하고 비밀번호를 입력해 달라고 한다. 끝나면 알려 달라고 한다.
     ```bash
     bash setup/install.sh
     ```
   - **비밀번호를 채팅으로 받지 않는다.** `echo 비밀번호 | sudo -S` 같은 방법도 쓰지 않는다.

3. **실패했을 때**
   - `setup/install.log`에서 `##### 설치 실패` 위쪽의 오류를 읽고 원인을 설명한다.
   - sudo가 필요 없는 단계(uv, `.venv`, pip, colcon 빌드, 별칭)만 다시 돌릴 때는 직접 `bash setup/install.sh --no-sudo`를 실행해도 된다.
   - 자주 나오는 원인:

     | 증상 | 조치 |
     |---|---|
     | `사양이 부족해서 멈춥니다` | `setup/hw_profile.py`의 `문제:` 줄을 본다. 디스크면 휴지통·다운로드 폴더 정리 |
     | `Could not get lock /var/lib/dpkg/lock` | 자동 업데이트가 도는 중이다. 몇 분 기다렸다가 같은 명령 다시 실행 |
     | `Temporary failure resolving`, curl 실패 | 인터넷 연결 확인 (학교 와이파이 인증 페이지 등) |
     | `colcon 빌드 실패` | `/tmp/dsp_colcon.log` 확인. 보통 ROS 설치가 덜 됐을 때다 |
     | `snap이 없어서 VS Code를 못 깔았습니다` | VS Code가 이미 열려 있다면 무시해도 된다 |
     | 커널 목록에 `.venv`가 없음 | VS Code 확장 Python, Jupyter 설치 확인 후 창 다시 불러오기 |

4. **설치 확인**
   - Bash 호출마다 셸이 새로 뜨므로 환경을 같은 줄에서 불러온다:
     ```bash
     source setup/env.sh && python setup/check_env.py
     ```
   - 10초 안팎이면 끝난다. 첫 줄 근처의 `자원 상한` 항목이 OK인지도 본다. 3D 창 시험이 화면 문제로만 실패하면 `--no-window`로 다시 확인하고, 그 사실을 부원에게 알린다.

5. **수업 준비 안내**
   - 수업 당일에는 `dsp` → `git pull`로 그날 회차 폴더를 받고, 그 폴더의 `lesson.ipynb`를 연다.
     오른쪽 위 커널을 `.venv (Python 3.12)`로 고르고, 첫 셀이 "시뮬레이터 준비 완료"를 내면 된다.
   - 회차 폴더가 아직 없으면 정상이다. 미리 받을 방법을 찾지 않는다.
   - 4~5회차(ROS 2)는 터미널에서 `dsp`를 친 뒤 `code .`로 VS Code를 연 창에서 해야 한다고 알려 준다.
     `dsp` 별칭은 새 터미널부터 적용된다.

## 자원 상한 (노트북의 80%)

- `import dsp`를 하는 모든 수업 코드는 `dsp/limits.py`가 CPU 코어 수와 메모리를 상한(기본 80%) 안으로 묶는다.
  비율은 설치 때 `setup/hw_profile.py`가 노트북 사양을 보고 `setup/hw_profile.env`에 적는다.
- 상한을 끄지 않는다 (`DSP_NO_LIMIT=1`, `taskset` 해제, slice 파일 삭제 등). 느리다는 말을 들어도 상한은 그대로 둔다.
  노트북이 버거우면 `DSP_LIMIT_PCT=70 bash setup/install.sh --no-sudo`처럼 오히려 낮추는 쪽만 제안한다.
- 셀 실행 중 `⚠️ ... 실행을 중단합니다`가 나오면 메모리 상한에 걸린 것이다. 다른 프로그램(브라우저 탭 등)을 닫고,
  VS Code에서 커널을 재시작한 뒤 다시 실행하라고 안내한다.
- GPU는 계산에 쓰지 않는다. NVIDIA 드라이버, CUDA, GPU용 JAX를 설치하지 않는다.

## 지키는 것

- 이 노트북 안에서만 작업한다. 설치 범위는 `setup/install.sh`가 하는 것까지. 시스템 설정, SSH, 방화벽, 사용자 계정은 건드리지 않는다.
- conda, 다른 Python, `pip install --user` 등 다른 방식으로 따로 설치하지 않는다. 모든 패키지는 `.venv` 안에 둔다.
- `dsp/`, `ros/`, `weekNN/background.py`는 수업 공용 코드다. 설치 문제로 고칠 일이 생기면 고치지 말고 무엇이 문제인지 부원에게 설명한다. 강사에게 전달할 수 있게 오류 메시지를 정리해 준다.

## 수업 중에

- 수업 시간에 노트북의 `___`는 부원이 직접 채우는 칸이다. 부원이 "답 채워줘"라고 분명히 말하기 전에는 정답을 써 넣지 않는다.
  막혔다고 하면 무엇을 계산하는 칸인지, 어떤 변수를 쓰면 되는지 힌트부터 준다.
- 이 저장소에는 정답 파일이 없다. 막히면 그래프를 보고 원인을 같이 생각하게 돕는다.
- 그래프가 이상하면 노트북의 숫자(게인 등)를 바꿔서 다시 실행해 보라고 권한다. 수업은 "바꿔 보고 바로 보는" 방식이다.

## 저장소 구조

- `setup/` 설치: `install.sh`(전체 설치), `hw_profile.py`(사양 분석, 상한 결정), `env.sh`(환경 불러오기), `check_env.py`(확인), `requirements.txt`, `docker/`
- `weekNN/` 회차별 수업: `lesson.ipynb`(빈칸 노트북), `background.py`(배경 코드, 노트북이 `from background import *`로 불러옴)
- `dsp/` 시뮬레이터 라이브러리 (물리 엔진 Crazyflow 래퍼, 그래프 도구 `live.py`, 자원 상한 `limits.py`, 끝난 회차의 기준 구현 `ref/`)
- `ros/dsp_ros/` ROS 2 Jazzy 패키지 (4~5회차)
- `docs/` 설계 개요, 물리 엔진 비교
- 새 회차는 `git pull`로 받는다. 부원이 노트북에 채운 내용과 충돌하면, 채운 노트북을 다른 이름으로 저장해 두고 받게 안내한다.
