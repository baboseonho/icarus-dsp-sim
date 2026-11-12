#!/usr/bin/env bash
# ICARUS DSP 설치 스크립트. 우분투 24.04를 막 설치한 노트북에서 한 번 실행한다.
# 1~5회차 수업에 필요한 것을 전부 깐다: ROS 2 Jazzy, Python 3.12 가상환경, 물리 엔진, Jupyter, VS Code.
# Run once on a fresh Ubuntu 24.04 install. Installs everything weeks 1-5 need.
#
#   bash setup/install.sh                 # 전체 설치 (20~40분, 대부분 내려받는 시간)
#   bash setup/install.sh --no-vscode     # VS Code는 이미 있거나 따로 깔 때
#   bash setup/install.sh --no-sudo       # sudo가 필요한 단계(apt, ROS, VS Code)는 빼고 나머지만
#   DSP_LIMIT_PCT=70 bash setup/install.sh  # 노트북 자원 상한을 80% 대신 70%로
#
# 처음에 노트북 사양(CPU, 메모리, 디스크, GPU)을 보고 설치 방법과 자원 상한을 정한다 (setup/hw_profile.py).
# 설치 중에도, 설치 뒤 수업 코드도 CPU와 메모리를 상한(기본 80%)까지만 쓴다.
# 중간에 끊겨도 같은 명령을 다시 실행하면 된다. 이미 끝난 단계는 건너뛴다.
# 실행 기록은 setup/install.log에 쌓인다. 막히면 이 파일을 보여 주면 된다.
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO"
WITH_VSCODE=1; NO_SUDO=0
for a in "$@"; do
  case "$a" in
    --no-vscode) WITH_VSCODE=0 ;;
    --no-sudo) NO_SUDO=1 ;;
  esac
done
LOG="$REPO/setup/install.log"
exec > >(tee -a "$LOG") 2>&1
echo "##### install.sh $* 시작: $(date '+%F %T')"
trap 'echo "##### 설치 실패 (install.sh ${LINENO}번째 줄). 위의 오류 메시지를 확인하세요."' ERR
say() { printf '\n\033[1;36m== %s\033[0m\n' "$*"; }
SUDO=""; [ "$(id -u)" -ne 0 ] && SUDO="sudo"

say "1/8 노트북 사양 확인"
. /etc/os-release
echo "OS: $PRETTY_NAME"
if [ "${VERSION_ID:-}" != "24.04" ]; then
  echo "경고: Ubuntu 24.04가 아닙니다. Docker로 설치하는 편이 안전합니다 (README 참고)."
fi
if ! /usr/bin/python3 setup/hw_profile.py --write; then
  if [ "${FORCE:-0}" != "1" ]; then
    echo "사양이 부족해서 멈춥니다. 위의 '문제'를 해결하고 다시 실행하세요 (무시하려면 FORCE=1)."
    exit 1
  fi
fi
set -a; . setup/hw_profile.env; set +a

# 무거운 작업을 상한 안에서 돌린다: 정해진 코어에만, 낮은 우선순위로, (가능하면) 메모리 상한이 있는 slice 안에서
lim() {
  if [ "$DSP_CGROUP" = "1" ] && [ -f "$HOME/.config/systemd/user/icarus-dsp.slice" ]; then
    systemd-run --user --scope --quiet --slice=icarus-dsp.slice \
      nice -n 10 ionice -c 2 -n 7 taskset -c "$DSP_CPU_LIST" "$@"
  else
    nice -n 10 ionice -c 2 -n 7 taskset -c "$DSP_CPU_LIST" "$@"
  fi
}
export UV_CONCURRENT_DOWNLOADS=4 UV_CONCURRENT_BUILDS="$DSP_JOBS" UV_CONCURRENT_INSTALLS="$DSP_JOBS"
export MAKEFLAGS="-j$DSP_JOBS"

say "2/8 자원 상한 설정 (CPU ${DSP_CPU_LIMIT}/${DSP_CORES}코어, 메모리 ${DSP_LIMIT_PCT}%)"
# 수업 프로세스 전체를 묶는 systemd slice. 75%에서 메모리를 늦추고 80%(상한)에서 멈춘다.
# systemd를 못 쓰는 환경이면 dsp/limits.py가 코어 고정과 메모리 감시만으로 막는다.
if [ "$DSP_CGROUP" = "1" ]; then
  mkdir -p "$HOME/.config/systemd/user"
  cat > "$HOME/.config/systemd/user/icarus-dsp.slice" <<EOF
# ICARUS DSP 수업 코드가 쓰는 자원 상한. setup/install.sh가 노트북 사양을 보고 만들었다.
[Unit]
Description=ICARUS DSP course (up to ${DSP_LIMIT_PCT}% of this laptop)

[Slice]
CPUQuota=$((DSP_CPU_LIMIT * 100))%
MemoryHigh=$((DSP_LIMIT_PCT - 5))%
MemoryMax=${DSP_LIMIT_PCT}%
EOF
  systemctl --user daemon-reload
  echo "systemd slice: ~/.config/systemd/user/icarus-dsp.slice"
else
  echo "systemd 상한 없음: 코어 고정 + 메모리 감시로 제한"
fi

say "3/8 기본 도구 (git, 한글 글꼴, 영상, 그래프, 3D 화면용 라이브러리)"
if [ "$NO_SUDO" = "1" ]; then
  echo "--no-sudo 라서 건너뜀"
else
  $SUDO apt-get update
  $SUDO nice -n 10 apt-get install -y --no-install-recommends \
    git curl ca-certificates build-essential software-properties-common \
    fonts-nanum ffmpeg graphviz libgl1 libegl1 libglfw3 libosmesa6
fi

say "4/8 ROS 2 Jazzy (${DSP_ROS_VARIANT}판, 4~5회차)"
if [ -f /opt/ros/jazzy/setup.bash ]; then
  echo "이미 설치되어 있음: /opt/ros/jazzy"
elif [ "${SKIP_ROS:-0}" = "1" ] || [ "$NO_SUDO" = "1" ]; then
  echo "SKIP_ROS=1 또는 --no-sudo 라서 건너뜀"
else
  # 공식 문서의 deb 설치 절차 (docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
  $SUDO add-apt-repository -y universe
  ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
  curl -L -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME:-${VERSION_CODENAME}}_all.deb"
  $SUDO dpkg -i /tmp/ros2-apt-source.deb
  $SUDO apt-get update
  # desktop: rviz2, rqt 포함 (디스크·메모리 넉넉할 때) / base: 화면 도구 없이 수업에 필요한 것만
  if [ "$DSP_ROS_VARIANT" = "desktop" ]; then ROS_PKG=ros-jazzy-desktop; else ROS_PKG=ros-jazzy-ros-base; fi
  $SUDO nice -n 10 apt-get install -y "$ROS_PKG" ros-dev-tools
fi

say "5/8 uv와 Python 가상환경 (.venv, Python 3.12), 물리 엔진, Jupyter"
export PATH="$HOME/.local/bin:$PATH"
if ! command -v uv >/dev/null 2>&1; then
  curl -LsSf https://astral.sh/uv/install.sh | sh
fi
uv --version
# --system-site-packages: apt로 설치한 rclpy를 가상환경 안에서도 import하기 위해
if [ ! -x .venv/bin/python ]; then
  uv venv --python /usr/bin/python3.12 --system-site-packages .venv
fi
VIRTUAL_ENV="$REPO/.venv" lim uv pip install -r setup/requirements.txt
VIRTUAL_ENV="$REPO/.venv" lim uv pip install -e .
.venv/bin/python -m ipykernel install --user --name icarus-dsp --display-name "ICARUS DSP (py3.12)" >/dev/null
# 그래프 한글 글꼴 목록을 새로 만든다 (fonts-nanum을 방금 깔았을 수 있어서)
rm -rf "$HOME/.cache/matplotlib"

say "6/8 ROS 2 패키지 빌드 (ros/dsp_ros)"
if [ -f /opt/ros/jazzy/setup.bash ]; then
  set +u; source /opt/ros/jazzy/setup.bash; set -u
  ( cd ros && lim "$REPO/.venv/bin/python" -m colcon build --symlink-install --parallel-workers "$DSP_JOBS" \
      >/tmp/dsp_colcon.log 2>&1 && echo "colcon OK" ) \
    || { echo "colcon 빌드 실패. 전체 로그: /tmp/dsp_colcon.log"; tail -20 /tmp/dsp_colcon.log; exit 1; }
else
  echo "ROS 2가 없어서 건너뜀 (4~5회차 전에 SKIP_ROS 없이 다시 실행)"
fi

say "7/8 VS Code와 편의 설정"
if [ "$WITH_VSCODE" = "1" ]; then
  if command -v code >/dev/null 2>&1; then
    echo "VS Code 이미 설치됨"
  elif [ "$NO_SUDO" = "1" ]; then
    echo "--no-sudo 라서 VS Code 설치는 건너뜀"
  elif command -v snap >/dev/null 2>&1; then
    $SUDO snap install code --classic
  else
    echo "snap이 없어서 VS Code를 못 깔았습니다. https://code.visualstudio.com 에서 .deb를 받아 설치하세요."
  fi
  # 노트북을 VS Code에서 열고 실행하는 데 필요한 확장 (Python, Jupyter)
  if command -v code >/dev/null 2>&1; then
    code --install-extension ms-python.python --install-extension ms-toolsai.jupyter >/dev/null || true
  fi
fi
# 터미널을 열 때 `dsp` 한 단어로 환경을 불러오게 한다
LINE="alias dsp='cd \"$REPO\" && source setup/env.sh'"
grep -qxF "$LINE" "$HOME/.bashrc" 2>/dev/null || echo "$LINE" >> "$HOME/.bashrc"
echo "~/.bashrc 에 별칭 dsp 를 넣었습니다"

say "8/8 첫 실행 준비 + 설치 확인 (1~2분)"
# 그래프 한글 글꼴 목록을 미리 만들어 두고(수업 첫 셀이 느려지지 않게), 엔진 1대·6대와 ROS 2를 시험한다
set +u; source setup/env.sh; set -u
lim python -c "import matplotlib.font_manager as f; print('그래프 글꼴:', f.FontProperties(fname=f.findfont('NanumGothic')).get_name())"
WIN=""; [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ] && WIN="--no-window"
CHECK="모두 통과"
lim python setup/check_env.py $WIN || CHECK="FAIL 있음, 위 표 확인"

trap - ERR
echo "##### 설치 완료 (확인: $CHECK): $(date '+%F %T')"
cat <<EOF

설치가 끝났습니다. 새 터미널을 열고:
  dsp                            # = cd $REPO && source setup/env.sh
  python setup/check_env.py      # 다시 확인하고 싶을 때
  code .                         # VS Code로 저장소 열기 → week01/lesson.ipynb
EOF
