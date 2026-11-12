# ICARUS DSP 개발 컨테이너: Ubuntu 24.04 + ROS 2 Jazzy (apt) + Python 3.12
# 노트북이 Ubuntu 24.04가 아닐 때(예: 20.04) 수업과 같은 환경을 만드는 데 쓴다.
# Dev container for laptops that are not on Ubuntu 24.04 (e.g. 20.04).
#
#   docker build -t icarus-dsp:jazzy setup/docker
#   setup/docker/run.sh            # 컨테이너 시작 (CPU·RAM 제한 포함)
FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
# uv: Python 패키지 관리자
RUN curl -LsSf https://astral.sh/uv/install.sh | env UV_INSTALL_DIR=/usr/local/bin UV_NO_MODIFY_PATH=1 sh \
 && uv --version

# 컨테이너 안의 모든 셸에서 ROS 2 환경을 자동으로 불러온다
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /etc/bash.bashrc
ENV UV_LINK_MODE=copy \
    MUJOCO_GL=egl \
    PYTHONUNBUFFERED=1
USER ubuntu
WORKDIR /ws
