#!/usr/bin/env python3

import subprocess
from typing import Tuple

from .utils import run_subprocess


def run_robot_state_publisher(robot_description: str = "robot_description") -> Tuple[subprocess.Popen, str]:
    cmd = [
        "rosrun",
        "robot_state_publisher",
        "robot_state_publisher",
        f"robot_description:={robot_description}",
    ]
    return run_subprocess(cmd)
