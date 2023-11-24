#!/usr/bin/env python3

import subprocess
from typing import Tuple

from .utils import find_files, run_subprocess


def run_map_server(map_server_dict, resources_paths) -> Tuple[subprocess.Popen, str]:
    map_path = find_files(resources_paths, map_server_dict["map"])
    cmd = [
        "rosrun",
        "map_server",
        "map_server",
        f"{map_path}",
    ]
    return run_subprocess(cmd), map_path
