#!/usr/bin/env bash
# 개발 컨테이너를 백그라운드로 띄운다. 노트북이 버벅이지 않게 CPU 8코어, RAM 5 GB로 제한한다.
# Starts the dev container in the background, capped at 8 CPUs and 5 GB RAM.
#   setup/docker/run.sh                # 시작
#   docker exec -it icarus-dsp bash    # 안으로 들어가기
set -euo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
NAME="${NAME:-icarus-dsp}"
CPUS="${CPUS:-8}"
MEM="${MEM:-5g}"
xhost +local: >/dev/null 2>&1 || true
docker rm -f "$NAME" >/dev/null 2>&1 || true
docker run -d --name "$NAME" \
  --cpus="$CPUS" --memory="$MEM" --memory-swap="$MEM" \
  --network host --ipc host \
  -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  --device /dev/dri --group-add video --group-add "$(getent group render | cut -d: -f3 || echo 109)" \
  -v icarus-dsp-home:/home/ubuntu \
  -v "$REPO":/ws/icarus-dsp-sim \
  -w /ws/icarus-dsp-sim \
  icarus-dsp:jazzy sleep infinity
echo "$NAME 컨테이너를 시작했습니다 (cpus=$CPUS mem=$MEM). 들어가기: docker exec -it $NAME bash"
