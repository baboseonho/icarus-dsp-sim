from glob import glob
from setuptools import setup

package_name = "dsp_ros"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    # ros2 run dsp_ros <이름> 으로 부르는 실행 파일들
    entry_points={
        "console_scripts": [
            "sim_bridge = dsp_ros.sim_bridge:main",
            "mocap = dsp_ros.mocap_node:main",
            "link = dsp_ros.link_node:main",
            "agent = dsp_ros.agent_node:main",
            "commander = dsp_ros.commander_node:main",
        ],
    },
)
