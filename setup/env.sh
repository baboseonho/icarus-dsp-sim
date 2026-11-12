# 새 터미널마다 `source setup/env.sh`. ROS 2, .venv, 우리 ROS 패키지를 한 번에 불러온다.
# Source this in every new terminal: loads ROS 2, the venv and our ROS package.
_DSP_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash
[ -f "$_DSP_REPO/ros/install/setup.bash" ] && source "$_DSP_REPO/ros/install/setup.bash"
source "$_DSP_REPO/.venv/bin/activate"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
# 같은 와이파이의 다른 부원 노드와 섞이지 않게 ROS 2 통신은 내 노트북 안에서만
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export PYTHONPATH="$_DSP_REPO:${PYTHONPATH:-}"
# 노트북 사양에 맞춘 자원 상한 (install.sh가 setup/hw_profile.py로 만든 파일). 실제 제한은 dsp/limits.py가 건다
if [ -f "$_DSP_REPO/setup/hw_profile.env" ]; then
  set -a; . "$_DSP_REPO/setup/hw_profile.env"; set +a
  export OMP_NUM_THREADS="${OMP_NUM_THREADS:-$DSP_CPU_LIMIT}" OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-$DSP_CPU_LIMIT}"
  export MKL_NUM_THREADS="${MKL_NUM_THREADS:-$DSP_CPU_LIMIT}"
fi
# 3D 창은 화면 주사율까지만 그린다 (GPU를 쉬지 않고 돌리지 않게). 계산에는 GPU를 쓰지 않는다
export vblank_mode=1 __GL_SYNC_TO_VBLANK=1
# JAX가 CPU 코어를 전부 잡아먹지 않게
export XLA_FLAGS="${XLA_FLAGS:---xla_cpu_multi_thread_eigen=false}"
export JAX_PLATFORMS=cpu
# 한 번 컴파일한 물리 엔진을 디스크에 저장해 두고 다음 실행부터 바로 쓴다
export JAX_COMPILATION_CACHE_DIR="${JAX_COMPILATION_CACHE_DIR:-$HOME/.cache/icarus-dsp-jax}"
unset _DSP_REPO
