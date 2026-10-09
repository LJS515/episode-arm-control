from glob import glob

from setuptools import setup

package_name = "episode_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Shikai Lou",
    maintainer_email="LJS515@users.noreply.github.com",
    description="ROS2 front end for the Episode 1 safety layer",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_interface_node = episode_ros2.robot_interface_node:main",
            "diagnostics_node = episode_ros2.diagnostics_node:main",
            "mock_hardware_node = episode_ros2.mock_hardware_node:main",
        ],
    },
)
