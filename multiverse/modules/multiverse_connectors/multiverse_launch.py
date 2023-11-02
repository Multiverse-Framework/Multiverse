#!/usr/bin/env python3.10

import sys
import yaml
import os
import subprocess
import re
import signal
from typing import List

processes = {"multiverse_server": [], "ros": [], "ros_control": []}
simulators = {"mujoco"}
multiverse_path = os.path.dirname(os.path.dirname(os.path.dirname(sys.argv[0])))
resources_path = os.path.join(multiverse_path, "resources")
muv_file = sys.argv[1]


def run_subprocess(cmd: List[str]):
    cmd_str = " ".join(cmd)
    print(f'Execute "{cmd_str}"')
    return subprocess.Popen(cmd)


def parse_mujoco(mujoco_data):
    world_xml_path = os.path.join(resources_path, "world")
    if "world" in mujoco_data and "path" in mujoco_data["world"]:
        if os.path.isabs(mujoco_data["world"]["path"]):
            world_xml_path = mujoco_data["world"]["path"]
        else:
            world_xml_path = os.path.join(world_xml_path, mujoco_data["world"]["path"])
    else:
        world_xml_path = os.path.join(world_xml_path, "floor/mjcf/floor.xml")

    mujoco_args = [f"--world={world_xml_path}"]

    robots_xml_path = os.path.join(resources_path, "robot")
    if "robots" in mujoco_data:
        for robot_name in mujoco_data["robots"]:
            if "path" in mujoco_data["robots"][robot_name]:
                if not os.path.isabs(mujoco_data["robots"][robot_name]["path"]):
                    mujoco_data["robots"][robot_name]["path"] = os.path.join(
                        robots_xml_path, mujoco_data["robots"][robot_name]["path"]
                    )
        robots_dict = mujoco_data["robots"]
        mujoco_args.append(f"--robots={robots_dict}".replace(" ", ""))

    return mujoco_args


def parse_simulator(simulator_data):
    if simulator_data["simulator"] == "mujoco":
        return parse_mujoco(simulator_data)
    else:
        return None


def main():
    try:
        with open(muv_file, "r") as file:
            data = yaml.safe_load(file)
    except Exception as e:
        print(f"Error reading MUV file: {e}")
        sys.exit(1)

    multiverse_server_dict = data.get("multiverse_server", None)
    multiverse_client_dict = data.get("multiverse_client", None)

    server_host = "tcp://127.0.0.1"
    server_port = "7000"
    if multiverse_server_dict is not None:
        server_host = multiverse_server_dict.get("host", "tcp://127.0.0.1")
        server_port = multiverse_server_dict.get("port", "7000")
    process = run_subprocess(["multiverse_server", f"{server_host}:{server_port}"])
    processes["multiverse_server"] = [process]

    for simulator in simulators:
        processes[simulator] = []

    for simulation_name, simulator_data in data.items():
        if (
            "simulator" not in simulator_data
            or simulator_data["simulator"] not in simulators
        ):
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

            if (
                multiverse_server_dict is not None
                and multiverse_client_dict is not None
            ):
                if multiverse_client_dict.get(simulation_name, None) is not None:
                    multiverse_dict = {
                        "multiverse_server": multiverse_server_dict,
                        "multiverse_client": multiverse_client_dict.get(simulation_name, None),
                    }
                    cmd += [f"{multiverse_dict}".replace(" ", "").replace("'", '"')]
            
            suffix = "_headless" if simulator_data.get("headless", False) else ""
            cmd = [f"{simulator}{suffix}"] + cmd

            process = run_subprocess(cmd)
            processes[cmd[0]].append(process)

    if "multiverse_client" in data and "ros" in data["multiverse_client"]:
        cmd_1 = ["roscore"]
        process_1 = run_subprocess(cmd_1)

        ros_params_dict = data["multiverse_client"]["ros"]
        ros_services_dict = ros_params_dict["services"]
        ros_publishers_dict = ros_params_dict["publishers"]
        ros_subscribers_dict = ros_params_dict["subscribers"]
        cmd_2 = [
            "rosrun",
            "multiverse_socket",
            "multiverse_socket_node.py",
            f"--multiverse_server=\"{multiverse_server_dict}\"",
            f"--services=\"{ros_services_dict}\"",
            f"--publishers=\"{ros_publishers_dict}\"",
            f"--subscribers=\"{ros_subscribers_dict}\""
        ]
        process_2 = run_subprocess(cmd_2)

        processes["ros"] = [process_1, process_2]

    # if multiverse_server_dict is not None and "multiverse_client" in data and "ros_control" in data["multiverse_client"]:
    #     for world, robots in data["multiverse_client"]["ros_control"].items():
    #         for robot, robot_params in robots.items():
    #             multiverse_dict = {
    #                 "multiverse_server": multiverse_server_dict,
    #                 "multiverse_client": robot_params["meta_data"] | {"world": world},
    #                 "robot": robot
    #             }
    #             cmd = [
    #                 "rosrun",
    #                 "multiverse_control",
    #                 "multiverse_control_node",
    #                 f"{multiverse_dict}".replace(" ", "").replace("'", '"').replace('"','\"'),
    #             ]
    #             process = run_subprocess(cmd)
    #             processes["ros_control"].append(process)

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
