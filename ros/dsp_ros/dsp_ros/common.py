"""dsp_ros 노드들이 같이 쓰는 이름, QoS, 메시지 변환 함수.

Names, QoS profiles and message helpers shared by the dsp_ros nodes.

토픽 (N = 1..6):
  /cfN/pose          PoseStamped        mocap -> agent, commander, 100 Hz
  /cfN/setpoint      PoseStamped        agent_cfN -> link, 50 Hz
  /cfN/command       String (JSON)      commander -> link, type: takeoff | go_to | land | emergency_stop
  /swarm/formation   String (JSON)      commander -> agent, {"name", "center"}
  /swarm/emergency   Empty              commander -> link, 모든 드론 비상 정지
  /swarm/status      String (JSON)      commander 상태
  /sim/truth         PoseArray          시뮬 전용. sim_bridge -> mocap, 엔진의 실제 위치
"""
from __future__ import annotations

import json
import time

import numpy as np
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

NAMES = [f"cf{i}" for i in range(1, 7)]

# 위치처럼 계속 들어오는 값은 최신 것만 있으면 된다.
SENSOR_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
# 대형·상태는 늦게 뜬 노드도 마지막 값을 받아야 한다 (TRANSIENT_LOCAL).
LATCHED_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
# 명령은 하나도 잃으면 안 된다.
CMD_QOS = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)


def now_s(node) -> float:
    """노드 시계의 현재 시각 [s]."""
    return node.get_clock().now().nanoseconds * 1e-9


def pose_to_np(msg: PoseStamped):
    """PoseStamped 를 (위치 p (3,), 쿼터니언 q wxyz (4,), 시각 t [s]) 로 푼다."""
    p = msg.pose.position
    q = msg.pose.orientation
    t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    return np.array([p.x, p.y, p.z]), np.array([q.w, q.x, q.y, q.z]), t


def np_to_pose(p, q=(1.0, 0.0, 0.0, 0.0), stamp=None, frame="world") -> PoseStamped:
    """pose_to_np 의 반대. stamp 를 주지 않으면 헤더 시각은 0 으로 남는다."""
    m = PoseStamped()
    m.header.frame_id = frame
    if stamp is not None:
        sec = int(stamp)
        m.header.stamp.sec, m.header.stamp.nanosec = sec, int((stamp - sec) * 1e9)
    m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in p)
    m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z = (float(v) for v in q)
    return m


def jdump(**kw) -> str:
    """키워드 인자를 JSON 문자열로. numpy 배열은 리스트로 바꾼다."""
    return json.dumps(kw, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))


def mono() -> float:
    return time.monotonic()


def make_executor():
    """노드를 돌릴 실행기. 가능하면 rclpy의 EventsExecutor를 쓴다.

    기본 실행기보다 메시지 하나당 처리 비용이 훨씬 작아서, 노드 열 개가 초당 수천 개의
    메시지를 주고받는 이 시스템에서 CPU를 크게 아낀다. 문제가 생기면 DSP_EXECUTOR=default 로
    기본 실행기로 돌아간다.

    Executor for our nodes: rclpy's EventsExecutor when available (much lower per-message
    overhead), or the default one with DSP_EXECUTOR=default.
    """
    import os
    if os.environ.get("DSP_EXECUTOR", "events") != "default":
        try:
            from rclpy.experimental import EventsExecutor
            return EventsExecutor()
        except ImportError:
            pass
    from rclpy.executors import SingleThreadedExecutor
    return SingleThreadedExecutor()


def spin(node):
    """make_executor()로 노드 하나를 돌린다."""
    exe = make_executor()
    exe.add_node(node)
    try:
        exe.spin()
    finally:
        exe.remove_node(node)
