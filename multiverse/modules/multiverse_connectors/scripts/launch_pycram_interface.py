#!/usr/bin/env python3

import os
import subprocess
from typing import Optional, List

from multiverse_launch import MultiverseLaunch
from utils import run_subprocess


class MultiversePycramLaunch(MultiverseLaunch):
    multiverse_control_pkg_path: Optional[str]
    mesh_abspath_prefix: Optional[str]

    def __init__(self):
        super().__init__()

    @property
    def mesh_abspath_prefix(self) -> str:
        mesh_abspath_prefix = os.path.relpath("/", self.multiverse_control_pkg_path)
        return os.path.join("package://multiverse_control", mesh_abspath_prefix)

    def start_pycram_socket(self) -> List[subprocess.Popen]:
        processes: List[subprocess.Popen] = [self.run_multiverse_pycram()]
        return processes

    def run_multiverse_pycram(self) -> Optional[subprocess.Popen]:
        cmd = [
            "python multiverse_pycram_run",
            f'--multiverse_server="{self.multiverse_server}"',
        ]
        return run_subprocess(cmd)


def main():
    multiverse_launch = MultiversePycramLaunch()
    multiverse_launch.start_pycram_socket()


if __name__ == "__main__":
    main()
