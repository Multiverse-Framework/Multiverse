#!/usr/bin/env python3.10

import sys
import yaml
import os
import argparse
from utils import run_subprocess


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

    multiverse_server_dict = data.get("multiverse_server", {"host": "tcp://127.0.0.1", "port": "7000"})
    multiverse_client_dict = data.get("multiverse_clients")

    if multiverse_client_dict is not None and "ros2" in multiverse_client_dict:
        ros_dict = multiverse_client_dict.get("ros2", {})
        if "ros2" in multiverse_client_dict and any(key in ros_dict for key in ["services", "publishers", "subscribers"]):
            cmd = [
                "ros2",
                "run",
                "multiverse_socket",
                "multiverse_socket_node",
                f'--multiverse_server="{multiverse_server_dict}"',
            ]
            # if "services" in ros_dict:
            #     ros_services_dict = ros_dict["services"]
            #     cmd.append(f'--services="{ros_services_dict}"')
            if "publishers" in ros_dict:
                ros_publishers_dict = ros_dict["publishers"]
                cmd.append(f'--publishers="{ros_publishers_dict}"')
            # if "subscribers" in ros_dict:
            #     ros_subscribers_dict = ros_dict["subscribers"]
            #     cmd.append(f'--subscribers="{ros_subscribers_dict}"')

            process = run_subprocess(cmd)


if __name__ == "__main__":
    main()
