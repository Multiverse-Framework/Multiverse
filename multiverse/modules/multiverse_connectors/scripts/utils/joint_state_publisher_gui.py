#!/usr/bin/env python3

import subprocess
from typing import Tuple

from .utils import run_subprocess


def run_joint_state_publisher_gui(robot_description: str = "robot_description") -> Tuple[subprocess.Popen, str]:
    cmd = [
        "rosrun",
        "joint_state_publisher_gui",
        "joint_state_publisher_gui",
        f"robot_description:={robot_description}",
    ]
    return run_subprocess(cmd)
