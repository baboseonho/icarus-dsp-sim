"""군집 전체를 띄운다: sim_bridge, mocap, link, agent 6개, commander.

    ros2 launch dsp_ros swarm.launch.py                          # 4회차: 육각형 분산 합의
    ros2 launch dsp_ros swarm.launch.py week:=5 scenario:=final  # 최종: 이륙 후 일렬, 육각형, 삼각형, 착륙

인자와 기본값은 아래 ARGS 에 있다. commander 가 끝나면 launch 전체가 내려간다.

Launches the whole swarm. Arguments and defaults are in ARGS below; everything shuts
down when the commander exits.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, Shutdown, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node

NAMES = [f"cf{i}" for i in range(1, 7)]
# center 는 "x,y,z" 문자열, bag 은 비워 두면 녹화하지 않는다.
ARGS = {
    "week": "4", "graph": "ring", "scenario": "w4", "formation": "hexagon", "hold": "30.0", "seed": "0",
    "init": "grid", "loss": "0.02", "noise": "0.005", "delay": "0.020", "drop": "0.01", "link_delay": "0.010",
    "downwash": "true", "bag": "", "log_path": "", "center": "",
}


def nodes(context):
    # 인자 값을 실제 문자열로 받아야 숫자로 바꾸고 분기할 수 있어서 OpaqueFunction 안에서 만든다.
    v = {k: LC(k).perform(context) for k in ARGS}
    seed = int(v["seed"])
    out = [
        Node(package="dsp_ros", executable="sim_bridge", name="sim_bridge", output="screen",
             parameters=[{"seed": seed, "init": v["init"], "downwash": v["downwash"].lower() == "true",
                          "log_path": v["log_path"], "link_delay": float(v["link_delay"])}]),
        Node(package="dsp_ros", executable="mocap", name="mocap", output="screen",
             parameters=[{"seed": seed, "noise": float(v["noise"]), "delay": float(v["delay"]), "drop": float(v["drop"])}]),
        Node(package="dsp_ros", executable="link", name="link", output="screen",
             parameters=[{"seed": seed, "loss": float(v["loss"])}]),
    ]
    for nm in NAMES:
        out.append(Node(package="dsp_ros", executable="agent", name=f"agent_{nm}", output="screen",
                        arguments=[nm, "--week", v["week"], "--graph", v["graph"]]))
    cargs = ["--scenario", v["scenario"], "--formation", v["formation"], "--hold", v["hold"]]
    if v["center"]:
        cargs += ["--center"] + v["center"].split(",")
    commander = Node(package="dsp_ros", executable="commander", name="commander", output="screen", arguments=cargs)
    out.append(commander)
    out.append(RegisterEventHandler(OnProcessExit(target_action=commander, on_exit=[Shutdown(reason="commander done")])))
    if v["bag"]:
        topics = [f"/{nm}/{t}" for nm in NAMES for t in ("pose", "setpoint")] + \
                 ["/swarm/formation", "/swarm/status", "/swarm/emergency", "/sim/truth"]
        out.append(ExecuteProcess(cmd=["ros2", "bag", "record", "-o", v["bag"], "--topics"] + topics, output="log"))
    return out


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value=d) for k, d in ARGS.items()]
                             + [OpaqueFunction(function=nodes)])
