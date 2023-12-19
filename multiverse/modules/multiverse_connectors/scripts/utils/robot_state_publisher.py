#!/usr/bin/env python3

import subprocess

from .utils import run_subprocess


def run_robot_state_publisher(robot_description: str = "robot_description") -> subprocess.Popen:
    cmd = [
        "rosrun",
        "robot_state_publisher",
        "robot_state_publisher",
        f"robot_description:={robot_description}",
    ]
    return run_subprocess(cmd)
    
def run_robot_state_publisher2(robot_file_string: str) -> subprocess.Popen:
    cmd = [
        "ros2",
        "run",
        "robot_state_publisher",
        "robot_state_publisher",
        "--ros-args",
        "-p", 
        f'robot_description:={robot_file_string}',
    ]
    return run_subprocess(cmd)
