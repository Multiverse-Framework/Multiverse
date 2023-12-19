#!/usr/bin/env python3

import subprocess
import yaml

from .utils import find_files, run_subprocess

def run_move_base(move_base_dict, resources_paths, map_path) -> subprocess.Popen:
    import rosparam
    base_global_planner = move_base_dict.get("base_global_planner", "navfn/NavfnROS")
    base_local_planner = move_base_dict.get("base_local_planner", "dwa_local_planner/DWAPlannerROS")
    config_path = find_files(resources_paths, move_base_dict["config"])
    with open(config_path, "r") as file:
        move_base_params = yaml.safe_load(file)
    rosparam.upload_params("/move_base_node", move_base_params)
    cmd = [
        "rosrun",
        "move_base",
        "move_base",
        f"_base_global_planner:={base_global_planner}",
        f"base_local_planner:={base_local_planner}",
        f"{map_path}",
    ]
    return run_subprocess(cmd)
