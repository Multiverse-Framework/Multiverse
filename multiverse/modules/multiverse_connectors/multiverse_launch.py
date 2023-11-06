#!/usr/bin/env python3.10

import sys
import yaml
import os
import glob
import subprocess
import re
import signal
from typing import List
import rospy
import xml.etree.ElementTree as ET
import argparse


def find_files(filename_pattern):
    matches = []
    for resources_path in resources_paths:
        search_pattern = os.path.join(resources_path, "**", filename_pattern)
        matches += [path for path in glob.iglob(search_pattern, recursive=True)]
    if len(matches) == 0:
        raise FileNotFoundError(f"{filename_pattern} not found in {resources_paths}")
    elif len(matches) >= 1:
        if len(matches) > 1:
            print(f"Multiple matches for '{filename_pattern}' detected: {matches}. Using the first one.")
        match = matches[0]
    return match


def run_subprocess(cmd: List[str]):
    cmd_str = " ".join(cmd)
    print(f'Execute "{cmd_str}"')
    return subprocess.Popen(cmd)


def parse_mujoco(mujoco_data):
    worlds_path = find_files(mujoco_data["world"]["path"])
    mujoco_args = [f"--world={worlds_path}"]

    for robot_name in mujoco_data.get("robots", []):
        if "path" in mujoco_data["robots"][robot_name]:
            mujoco_data["robots"][robot_name]["path"] = find_files(mujoco_data["robots"][robot_name]["path"])
    robots_dict = mujoco_data["robots"]
    mujoco_args.append(f"--robots={robots_dict}".replace(" ", ""))

    return mujoco_args


def parse_simulator(simulator_data):
    if simulator_data["simulator"] == "mujoco":
        return parse_mujoco(simulator_data)
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

    global resources_paths
    resources_paths = data.get("resources", ["../robots", "../worlds", "../objects"])
    resources_paths = [
        os.path.join(os.path.dirname(muv_file), resources_path) if not os.path.isabs(resources_path) else resources_path
        for resources_path in resources_paths
    ]
    print(resources_paths)

    processes = {"multiverse_server": [], "ros": [], "ros_control": []}
    simulators = {"mujoco"}

    world_dict = data.get("worlds", {})
    simulation_dict = data.get("simulations", {})
    multiverse_server_dict = data.get("multiverse_server", {"host": "tcp://127.0.0.1", "port": "7000"})
    multiverse_client_dict = data.get("multiverse_clients")

    server_host = multiverse_server_dict.get("host", "tcp://127.0.0.1")
    server_port = multiverse_server_dict.get("port", "7000")
    process = run_subprocess(["multiverse_server", f"{server_host}:{server_port}"])
    processes["multiverse_server"] = [process]

    for simulator in simulators:
        processes[simulator] = []

    for simulation_name, simulator_data in simulation_dict.items():
        if "simulator" not in simulator_data or simulator_data["simulator"] not in simulators:
            continue

        cmd = [f"mujoco_compile", f"--name={simulation_name}"]
        cmd += parse_simulator(simulator_data)

        cmd_str = " ".join(cmd)
        print(f'Execute "{cmd_str}"')

        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode == 0:
            print(result.stdout)
            scene_xml_path = re.search(r"Scene:\s*([^\n]+)", result.stdout).group(1)
            cmd = [f"{scene_xml_path}"]

            world = simulator_data.get("world")
            world_name = "world" if world is None else world["name"]
            rtf_desired = 1 if world_name not in world_dict else world_dict[world_name].get("rtf_desired", 1)

            config_dict = simulator_data["config"] | {"rtf_desired": rtf_desired, "resources": resources_paths}
            cmd += [f"{config_dict}".replace(" ", "").replace("'", '"')]

            if multiverse_client_dict is not None and multiverse_client_dict.get(simulation_name) is not None:
                multiverse_client_dict[simulation_name]["meta_data"] = {
                    "world": world_name,
                    "name": simulation_name,
                }
                multiverse_dict = {
                    "multiverse_server": multiverse_server_dict,
                    "multiverse_client": multiverse_client_dict[simulation_name],
                }
                cmd += [f"{multiverse_dict}".replace(" ", "").replace("'", '"')]

            suffix = "_headless" if simulator_data.get("headless", False) else ""
            cmd = [f"{simulator}{suffix}"] + cmd

            process = run_subprocess(cmd)
            processes[cmd[0]].append(process)

    if multiverse_client_dict is not None and ("ros" in multiverse_client_dict or "ros_control" in multiverse_client_dict):
        import rosgraph

        if not rosgraph.is_master_online():
            cmd = ["roscore"]
            process = run_subprocess(cmd)
            processes["ros"] = [process]

        if "ros" in multiverse_client_dict:
            cmd = [
                "rosrun",
                "multiverse_socket",
                "multiverse_socket_node.py",
                f'--multiverse_server="{multiverse_server_dict}"',
            ]
            if "services" in multiverse_client_dict["ros"]:
                ros_services_dict = multiverse_client_dict["ros"]["services"]
                cmd.append(f'--services="{ros_services_dict}"')
            if "publishers" in multiverse_client_dict["ros"]:
                ros_publishers_dict = multiverse_client_dict["ros"]["publishers"]
                cmd.append(f'--publishers="{ros_publishers_dict}"')
            if "subscribers" in multiverse_client_dict["ros"]:
                ros_subscribers_dict = multiverse_client_dict["ros"]["subscribers"]
                cmd.append(f'--subscribers="{ros_subscribers_dict}"')

            process = run_subprocess(cmd)
            processes["ros"].append(process)

        if "ros_control" in multiverse_client_dict:
            rospy.init_node(name="multiverse_launch")

            for ros_control_param in multiverse_client_dict["ros_control"]:
                controller_manager = ros_control_param["controller_manager"]
                robot = controller_manager["robot"]
                robot_description = controller_manager["robot_description"]
                robot_urdf_path = find_files(controller_manager["urdf"])

                tree = ET.parse(robot_urdf_path)
                root = tree.getroot()
                robot_urdf_str = ET.tostring(root, encoding="unicode")
                rospy.set_param(f"{robot_description}", f"{robot_urdf_str}")
                multiverse_dict = {
                    "multiverse_server": multiverse_server_dict,
                    "multiverse_client": {
                        "host": ros_control_param["host"],
                        "port": ros_control_param["port"],
                        "meta_data": ros_control_param["meta_data"],
                    },
                    "controller_manager": {
                        "robot": robot,
                        "robot_description": robot_description,
                    },
                }
                cmd = [
                    "rosrun",
                    "multiverse_control",
                    "multiverse_control_node",
                    f"{multiverse_dict}".replace(" ", "").replace("'", '"').replace('"', '"'),
                ]
                process = run_subprocess(cmd)
                processes["ros_control"].append(process)

                control_config_path = find_files(controller_manager["config"])
                os.environ["ROS_NAMESPACE"] = f"{robot}"
                cmd = [
                    "rosparam",
                    "load",
                    f"{control_config_path}",
                ]
                process = run_subprocess(cmd)
                process.wait()

                for command, controllers in controller_manager["controllers"].items():
                    cmd = [
                        "rosrun",
                        "controller_manager",
                        "controller_manager",
                        f"{command}",
                    ] + controllers[0].split()
                    process = run_subprocess(cmd)
                    processes["ros_control"].append(process)

    try:
        processes["multiverse_server"][0].wait()
    except KeyboardInterrupt:
        print("CTRL+C detected in parent. Sending SIGINT to subprocess.")
        for process in processes["ros_control"]:
            print(f"Terminate ROS with PID {process.pid}")
            process.send_signal(signal.SIGINT)
            process.wait()

        for process in processes["ros"]:
            print(f"Terminate ROS with PID {process.pid}")
            process.send_signal(signal.SIGINT)
            process.wait()

        for simulator in simulators:
            if len(processes[simulator]) > 0:
                for process in processes[simulator]:
                    print(f"Terminate {simulator} with PID {process.pid}")
                    process.send_signal(signal.SIGINT)
                    process.wait()

        for process in processes["multiverse_server"]:
            print(f"Terminate multiverse_server with PID {process.pid}")
            process.send_signal(signal.SIGINT)
            process.wait()


if __name__ == "__main__":
    main()
