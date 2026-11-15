# ICARUS DSP 개발 컨테이너: Ubuntu 24.04 + ROS 2 Jazzy (apt) + Python 3.12
# 노트북이 Ubuntu 24.04/26.04가 아닐 때(예: 20.04), 또는 Ubuntu 26.04에서 4~5회차(ROS 2)를 할 때
# 수업과 같은 환경을 만드는 데 쓴다. 26.04의 ROS 2 Lyrical은 Python 3.14용이라 수업 커널(3.12)에서 rclpy를 못 쓰기 때문이다.
# Dev container (Ubuntu 24.04 + Jazzy) for laptops not on Ubuntu 24.04/26.04, and for weeks 4-5 on 26.04.
#
#   docker build -t icarus-dsp:jazzy setup/docker
#   setup/docker/run.sh            # 컨테이너 시작 (CPU·RAM 제한 포함)
#   (실험) docker build --build-arg ROS_DISTRO=lyrical -t icarus-dsp:lyrical setup/docker   # 26.04 + Lyrical, 시험 안 함
ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop-full
ARG ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive
# uv: Python 패키지 관리자
RUN curl -LsSf https://astral.sh/uv/install.sh | env UV_INSTALL_DIR=/usr/local/bin UV_NO_MODIFY_PATH=1 sh \
 && uv --version

# 컨테이너 안의 모든 셸에서 ROS 2 환경을 자동으로 불러온다
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc
ENV UV_LINK_MODE=copy \
    MUJOCO_GL=egl \
    PYTHONUNBUFFERED=1
USER ubuntu
WORKDIR /ws
