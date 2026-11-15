#!/usr/bin/env bash
# ICARUS DSP 설치 스크립트. 우분투 24.04 또는 26.04를 막 설치한 노트북에서 한 번 실행한다.
# 1~5회차 수업에 필요한 것을 전부 깐다: ROS 2, Python 3.12 가상환경, 물리 엔진, Jupyter, VS Code.
#   Ubuntu 24.04 → ROS 2 Jazzy,  Python 3.12 = 시스템 Python
#   Ubuntu 26.04 → ROS 2 Lyrical, Python 3.12 = uv가 따로 받는다 (시스템 Python 3.14는 건드리지 않는다)
#                  ROS 설치가 실패해도 1~3회차 준비(Python, 엔진, Jupyter)는 끝까지 한다
# Run once on a fresh Ubuntu 24.04 or 26.04 install. Installs everything weeks 1-5 need.
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
# 지원하는 우분투와 ROS 2 배포판: 24.04 → jazzy, 26.04 → lyrical (setup/hw_profile.py도 같은 표를 쓴다)
case "${VERSION_ID:-}" in
  24.04|26.04) ;;
  *) echo "경고: Ubuntu 24.04나 26.04가 아닙니다. ROS 2는 건너뜁니다. Docker로 설치하는 편이 안전합니다 (README 참고)." ;;
esac
if ! /usr/bin/python3 setup/hw_profile.py --write; then
  if [ "${FORCE:-0}" != "1" ]; then
    echo "사양이 부족해서 멈춥니다. 위의 '문제'를 해결하고 다시 실행하세요 (무시하려면 FORCE=1)."
    exit 1
  fi
fi
set -a; . setup/hw_profile.env; set +a
ROS_D="${DSP_ROS_DISTRO:-}"     # jazzy / lyrical / 빈칸(지원하지 않는 OS)
# 24.04는 예전처럼 ROS 실패 = 설치 실패. 26.04는 경고만 하고 계속한다 (1회차는 ROS가 필요 없다)
ROS_REQUIRED=0; [ "$ROS_D" = "jazzy" ] && ROS_REQUIRED=1
ros_warn() {
  echo "경고: $1"
  echo "  1~3회차(노트북)는 ROS 없이 된다. 4~5회차 전에 'bash setup/install.sh'를 다시 돌리거나 setup/docker/(24.04 + Jazzy)를 쓴다."
}

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

say "4/8 ROS 2 ${ROS_D:-없음} (${DSP_ROS_VARIANT}판, 4~5회차)"
# 공식 문서의 deb 설치 절차 (docs.ros.org/en/<배포판>/Installation/Ubuntu-Install-Debs.html)
# 명령을 &&로 이어서, 하나라도 실패하면 함수가 실패를 돌려준다 (24.04는 거기서 멈추고, 26.04는 경고 후 계속)
ros_install() {
  local ver pkg
  # desktop: rviz2, rqt 포함 (디스크·메모리 넉넉할 때) / base: 화면 도구 없이 수업에 필요한 것만
  if [ "$DSP_ROS_VARIANT" = "desktop" ]; then pkg="ros-$ROS_D-desktop"; else pkg="ros-$ROS_D-ros-base"; fi
  $SUDO add-apt-repository -y universe \
  && ver=$(curl -fsS https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
  && [ -n "$ver" ] \
  && curl -fL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.${UBUNTU_CODENAME:-${VERSION_CODENAME}}_all.deb" \
  && $SUDO dpkg -i /tmp/ros2-apt-source.deb \
  && $SUDO apt-get update \
  && $SUDO nice -n 10 apt-get install -y "$pkg" ros-dev-tools
}
if [ -z "$ROS_D" ]; then
  echo "이 OS용 ROS 2 배포판이 정해져 있지 않아서 건너뜀 (Ubuntu 24.04 → jazzy, 26.04 → lyrical)"
elif [ -f "/opt/ros/$ROS_D/setup.bash" ]; then
  echo "이미 설치되어 있음: /opt/ros/$ROS_D"
elif [ "${SKIP_ROS:-0}" = "1" ] || [ "$NO_SUDO" = "1" ]; then
  echo "SKIP_ROS=1 또는 --no-sudo 라서 건너뜀"
elif [ "$ROS_REQUIRED" = "1" ]; then
  ros_install
elif ! ros_install; then
  ros_warn "ROS 2 $ROS_D 설치에 실패했습니다. Python과 1회차 준비는 계속합니다."
fi
# PlotJuggler: 비행 중 값을 실시간 그래프로 본다 (1~5회차). ROS 2 저장소에 있어서 ROS 다음에 깐다.
# 이미 ROS가 깔린 노트북도 설치를 다시 돌리면 이것만 더 깐다.
# plotjuggler-ros(ROS 2 토픽 읽기, 4~5회차)가 아직 없는 배포판이면 PlotJuggler 본체만 깐다 (1~3회차 UDP는 본체로 된다)
if [ -n "$ROS_D" ] && [ -f "/opt/ros/$ROS_D/setup.bash" ] && [ ! -x "/opt/ros/$ROS_D/lib/plotjuggler/plotjuggler" ] \
   && [ "${SKIP_ROS:-0}" != "1" ] && [ "$NO_SUDO" != "1" ]; then
  echo "PlotJuggler 설치 (약 0.1~0.4 GB)"
  PJ_PKG="ros-$ROS_D-plotjuggler-ros"
  if ! apt-cache show "$PJ_PKG" >/dev/null 2>&1; then
    PJ_PKG="ros-$ROS_D-plotjuggler"
    echo "ros-$ROS_D-plotjuggler-ros가 아직 없어서 $PJ_PKG(본체)만 깝니다. 4~5회차의 ROS 2 토픽 그래프는 rqt나 ros2 topic echo로 본다"
  fi
  $SUDO nice -n 10 apt-get install -y --no-install-recommends "$PJ_PKG" \
    || echo "PlotJuggler 설치 실패. 노트북 안의 실시간 그래프만으로도 수업은 된다"
fi

say "5/8 uv와 Python 가상환경 (.venv, Python 3.12), 물리 엔진, Jupyter"
export PATH="$HOME/.local/bin:$PATH"
if ! command -v uv >/dev/null 2>&1; then
  curl -LsSf https://astral.sh/uv/install.sh | sh
fi
uv --version
if [ ! -x .venv/bin/python ]; then
  if [ -x /usr/bin/python3.12 ]; then
    # 24.04: 시스템 Python 3.12. --system-site-packages: apt로 설치한 rclpy를 가상환경 안에서도 import하기 위해
    uv venv --python /usr/bin/python3.12 --system-site-packages .venv
  else
    # 26.04처럼 시스템 Python이 3.12가 아니면 uv가 Python 3.12를 ~/.local/share/uv 에 따로 받는다
    # (시스템 Python은 그대로 둔다. apt의 rclpy는 시스템 Python용이라 이 .venv에서는 못 불러온다)
    uv python install 3.12
    uv venv --python 3.12 --python-preference only-managed .venv
  fi
fi
VIRTUAL_ENV="$REPO/.venv" lim uv pip install -r setup/requirements.txt
VIRTUAL_ENV="$REPO/.venv" lim uv pip install -e .
.venv/bin/python -m ipykernel install --user --name icarus-dsp --display-name "ICARUS DSP (py3.12)" >/dev/null
# 그래프 한글 글꼴 목록을 새로 만든다 (fonts-nanum을 방금 깔았을 수 있어서)
rm -rf "$HOME/.cache/matplotlib"

say "6/8 ROS 2 패키지 빌드 (ros/dsp_ros)"
if [ -n "$ROS_D" ] && [ -f "/opt/ros/$ROS_D/setup.bash" ]; then
  set +u; source "/opt/ros/$ROS_D/setup.bash"; set -u
  # 24.04: .venv가 시스템 colcon을 본다(--system-site-packages). 26.04: .venv에 colcon이 없으니 시스템 colcon을 쓴다
  if "$REPO/.venv/bin/python" -c "import colcon_core" >/dev/null 2>&1; then
    COLCON=("$REPO/.venv/bin/python" -m colcon)
  else
    COLCON=(colcon)
  fi
  if ( cd ros && lim "${COLCON[@]}" build --symlink-install --parallel-workers "$DSP_JOBS" \
      >/tmp/dsp_colcon.log 2>&1 ); then
    echo "colcon OK"
  else
    echo "colcon 빌드 실패. 전체 로그: /tmp/dsp_colcon.log"; tail -20 /tmp/dsp_colcon.log
    if [ "$ROS_REQUIRED" = "1" ]; then exit 1; fi
    ros_warn "ros/dsp_ros 빌드에 실패했습니다."
  fi
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
if [ "$ROS_D" != "jazzy" ]; then
  cat <<EOF

참고 (Ubuntu ${VERSION_ID:-?}): 1~3회차는 이대로 된다. 4~5회차(ROS 2)는 노트북 커널(Python 3.12)에서
ROS 2 ${ROS_D:-없음}의 rclpy를 불러올 수 없어서, 그때는 setup/docker/(Ubuntu 24.04 + ROS 2 Jazzy)를 쓴다.
EOF
fi
