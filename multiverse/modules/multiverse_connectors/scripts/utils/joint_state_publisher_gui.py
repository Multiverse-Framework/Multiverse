#!/usr/bin/env python3

import subprocess

from .utils import run_subprocess


def run_joint_state_publisher_gui(robot_description: str = "robot_description") -> subprocess.Popen:
    cmd = [
        "rosrun",
        "joint_state_publisher_gui",
        "joint_state_publisher_gui",
        f"robot_description:={robot_description}",
    ]
    return run_subprocess(cmd)
    
def run_joint_state_publisher_gui2(robot_file_path: str) -> subprocess.Popen:
    cmd = [
        "ros2",
        "run",
        "joint_state_publisher_gui",
        "joint_state_publisher_gui",
        f"{robot_file_path}",
    ]
    return run_subprocess(cmd)
