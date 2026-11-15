"""agent 노드 클래스 고르기.

수업에서는 부원이 노트북에서 AgentBase(agent_base.py)를 이어받아 직접 채운다.
강사 시연과 최종 ROS 비행은 기준 구현을 쓰는데, 기준 구현은 그 회차가 끝난 뒤 공개되므로
쓸 때 불러온다.

    cls = agent_class(4)      # 4회차 Agent (dsp/ref/w4.py)
    cls = agent_class(5)      # 5회차 SwarmAgent (swarm_agent.py, 4회차 Agent 위에)

Picks the agent class; the reference implementations are imported only when used.
"""
from .agent_base import AgentBase, STALE  # noqa: F401


def agent_class(week=4):
    if week >= 5:
        from .swarm_agent import swarm_agent_class
        return swarm_agent_class()
    from dsp.ref.w4 import Agent
    return Agent
