"""agent: 드론 한 대의 분산 제어 노드를 띄운다.

    ros2 run dsp_ros agent cf1 --week 4 --graph ring     # 4회차 Agent (분산 합의)
    ros2 run dsp_ros agent cf1 --week 5                  # 5회차 SwarmAgent (합의 + 충돌 회피)

수업 중에는 노트북 안에서 내가 쓴 Agent를 직접 띄우므로 이 실행 파일은 강사 시연용이다.
Starts one reference agent (used by the launch file for demos).
"""
import argparse

import rclpy
from rclpy.utilities import remove_ros_args

from dsp_ros.agents import agent_class
from dsp_ros.common import spin


def main():
    rclpy.init()
    ap = argparse.ArgumentParser()
    ap.add_argument("name")
    ap.add_argument("--week", type=int, default=4)
    ap.add_argument("--graph", default="ring")
    ap.add_argument("--k", type=float, default=None)
    args = ap.parse_args(remove_ros_args()[1:])   # launch 가 붙이는 --ros-args 는 떼고 읽는다
    kw = {"graph": args.graph}
    if args.k is not None:      # 안 주면 Agent 의 기본 게인
        kw["k"] = args.k
    node = agent_class(args.week)(args.name, **kw)
    try:
        spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
