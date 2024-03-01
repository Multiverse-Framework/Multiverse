#!/usr/bin/env python3

import subprocess
from typing import Optional, List

from multiverse_launch import MultiverseLaunch
from utils import run_subprocess


class MultiversePycramLaunch(MultiverseLaunch):

    def __init__(self):
        super().__init__()

    def start_pycram_socket(self) -> List[subprocess.Popen]:
        processes: List[subprocess.Popen] = [self.run_multiverse_pycram()]
        return processes

    def run_multiverse_pycram(self) -> Optional[subprocess.Popen]:
        cmd = [
            "multiverse_pycram_run",
            f'--multiverse_server="{self.multiverse_server}"',
        ]
        return run_subprocess(cmd)


def main():
    multiverse_launch = MultiversePycramLaunch()
    _ = multiverse_launch.start_pycram_socket()


if __name__ == "__main__":
    main()
