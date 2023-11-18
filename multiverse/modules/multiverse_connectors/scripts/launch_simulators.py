#!/usr/bin/env python3.10

import sys
import yaml
import os
import subprocess
import re
import argparse

from utils import find_files, run_subprocess


def parse_mujoco(resources_paths, mujoco_data):
    worlds_path = find_files(resources_paths, mujoco_data["world"]["path"])
    mujoco_args = [f"--world={worlds_path}"]

    if "robots" in mujoco_data:
        for robot_name in mujoco_data["robots"]:
            if "path" in mujoco_data["robots"][robot_name]:
                mujoco_data["robots"][robot_name]["path"] = find_files(resources_paths, mujoco_data["robots"][robot_name]["path"])
        robots_dict = mujoco_data["robots"]
        mujoco_args.append(f"--robots={robots_dict}".replace(" ", ""))

    return mujoco_args


def parse_simulator(resources_paths, simulator_data):
    if simulator_data["simulator"] == "mujoco":
        return parse_mujoco(resources_paths, simulator_data)
    else:
        return None


def main():
    parser = argparse.ArgumentParser(prog="multiverse_launch", description="Launch the multiverse framework")
    parser.add_argument(
        "--muv_file",
        type=str,
        required=True,
        help="Path to .muv file",
    )
    args = parser.parse_args()
    muv_file = args.muv_file

    try:
        with open(muv_file, "r") as file:
            data = yaml.safe_load(file)
    except Exception as e:
        print(f"Error reading MUV file: {e}")
        sys.exit(1)

    resources_paths = data.get("resources", ["../robots", "../worlds", "../objects"])
    resources_paths = [
        os.path.join(os.path.dirname(muv_file), resources_path) if not os.path.isabs(resources_path) else resources_path
        for resources_path in resources_paths
    ]

    simulators = {"mujoco"}

    world_dict = data.get("worlds", {})
    simulation_dict = data.get("simulations", {})
    multiverse_server_dict = data.get("multiverse_server", {"host": "tcp://127.0.0.1", "port": "7000"})
    multiverse_client_dict = data.get("multiverse_clients")

    for simulation_name, simulator_data in simulation_dict.items():
        if "simulator" not in simulator_data or simulator_data["simulator"] not in simulators:
            continue

        simulator = simulator_data["simulator"]

        cmd = [f"mujoco_compile", f"--name={simulation_name}"]
        cmd += parse_simulator(resources_paths, simulator_data)

        cmd_str = " ".join(cmd)
        print(f'Execute "{cmd_str}"')

        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode == 0:
            scene_xml_path = re.search(r"Scene:\s*([^\n]+)", result.stdout).group(1)
            cmd = [f"{scene_xml_path}"]

            world = simulator_data.get("world")
            world_name = "world" if world is None else world["name"]
            rtf_desired = 1 if world_name not in world_dict else world_dict[world_name].get("rtf_desired", 1)

            config_dict = simulator_data.get("config", {}) | {"rtf_desired": rtf_desired, "resources": resources_paths}
            cmd += [f"{config_dict}".replace(" ", "").replace("'", '"')]

            if multiverse_client_dict is not None and multiverse_client_dict.get(simulation_name) is not None:
                multiverse_client_dict[simulation_name]["meta_data"] = {
                    "world_name": world_name,
                    "simulation_name": simulation_name,
                }
                multiverse_dict = {
                    "multiverse_server": multiverse_server_dict,
                    "multiverse_client": multiverse_client_dict[simulation_name] | {"resources": resources_paths},
                }
                cmd += [f"{multiverse_dict}".replace(" ", "").replace("'", '"')]

            suffix = "_headless" if simulator_data.get("headless", False) else ""
            cmd = [f"{simulator}{suffix}"] + cmd

            process = run_subprocess(cmd)


if __name__ == "__main__":
    main()
