from setuptools import find_packages, setup
import os
from glob import glob

package_name = "cobot2"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "weights"), glob("resource/weights/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jun",
    maintainer_email="jsl5828@gmail.com",
    description="Vision-based TCP follow and task automation system using ROS 2.",
    license="MIT",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "tcp_follow = cobot2.tcp_follow_node:main",
            "yolo_camera = cobot2.yolo_camera_node:main",
            "auth_action = cobot2.auth_action_server:main",
            "salute = cobot2.salute_node:main",
            "shoot = cobot2.shoot_node:main",
            "safety_monitor = cobot2.safety_monitor_node:main",
            "orchestrator = cobot2.orchestrator:main",
            'follow_ui_node = cobot2.follow_ui_node:main',
            'follow_logger_node = cobot2.follow_logger_node:main',
        ],
    },
)
