#!/usr/bin/env python3.10

import sys
import yaml
import os
import xml.etree.ElementTree as ET
import argparse
from utils import find_files, run_subprocess


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

    if multiverse_client_dict is not None and "ros" in multiverse_client_dict:
        import rosgraph

        if not rosgraph.is_master_online():
            cmd = ["roscore"]
            process = run_subprocess(cmd)

        ros_dict = multiverse_client_dict.get("ros", {})
        if "ros" in multiverse_client_dict and any(key in ros_dict for key in ["services", "publishers", "subscribers"]):
            cmd = [
                "rosrun",
                "multiverse_socket",
                "multiverse_socket_node.py",
                f'--multiverse_server="{multiverse_server_dict}"',
            ]
            if "services" in ros_dict:
                ros_services_dict = ros_dict["services"]
                cmd.append(f'--services="{ros_services_dict}"')
            if "publishers" in ros_dict:
                ros_publishers_dict = ros_dict["publishers"]
                cmd.append(f'--publishers="{ros_publishers_dict}"')
            if "subscribers" in ros_dict:
                ros_subscribers_dict = ros_dict["subscribers"]
                cmd.append(f'--subscribers="{ros_subscribers_dict}"')

            process = run_subprocess(cmd)

        if "ros_run" in ros_dict or "ros_control" in ros_dict:
            import rospkg, rospy, rosparam

            rospy.init_node(name="multiverse_launch")
            rospack = rospkg.RosPack()
            multiverse_control_pkg_path = rospack.get_path("multiverse_control")
            mesh_abspath_prefix = os.path.relpath("/", multiverse_control_pkg_path)
            mesh_abspath_prefix = os.path.join("package://multiverse_control", mesh_abspath_prefix)

            def get_urdf_str(urdf_path: str) -> str:
                tree = ET.parse(urdf_path)
                root = tree.getroot()
                robot_urdf_str = ET.tostring(root, encoding="unicode")
                mesh_relpath_prefix = os.path.relpath(os.path.dirname(urdf_path), multiverse_control_pkg_path)
                mesh_relpath_prefix = os.path.join("package://multiverse_control", mesh_relpath_prefix) + "/"
                robot_urdf_str = robot_urdf_str.replace("file:///", mesh_abspath_prefix)
                robot_urdf_str = robot_urdf_str.replace("file://", mesh_relpath_prefix)
                return robot_urdf_str

            if "ros_run" in ros_dict:
                ros_run_dict = ros_dict["ros_run"]
                if "rviz" in ros_run_dict:
                    rviz_dict = ros_run_dict["rviz"]

                    for robot_description, urdf_path in rviz_dict.get("robot_descriptions", {}).items():
                        urdf_path = find_files(resources_paths, urdf_path)
                        urdf_str = get_urdf_str(urdf_path)
                        rospy.set_param(f"/{robot_description}", f"{urdf_str}")

                    rviz_config_path = find_files(resources_paths, rviz_dict["config"])
                    cmd = [
                        "rosrun",
                        "rviz",
                        "rviz",
                        "--display-config",
                        f"{rviz_config_path}",
                    ]
                    process = run_subprocess(cmd)

                if "map_server" in ros_run_dict:
                    map_server_dict = ros_run_dict["map_server"]
                    map_path = find_files(resources_paths, map_server_dict["map"])
                    cmd = [
                        "rosrun",
                        "map_server",
                        "map_server",
                        f"{map_path}",
                    ]
                    process = run_subprocess(cmd)

                if "move_base" in ros_run_dict:
                    move_base_dict = ros_run_dict["move_base"]
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
                    process = run_subprocess(cmd)

            if "ros_control" in ros_dict:
                for ros_control_dict in ros_dict["ros_control"]:
                    controller_manager = ros_control_dict["controller_manager"]
                    robot = controller_manager["robot"]
                    robot_description = controller_manager["robot_description"]
                    actuators = controller_manager["actuators"]
                    robot_urdf_path = find_files(resources_paths, controller_manager["urdf"])
                    robot_urdf_str = get_urdf_str(robot_urdf_path)
                    rospy.set_param(f"{robot_description}", f"{robot_urdf_str}")
                    multiverse_dict = {
                        "multiverse_server": multiverse_server_dict,
                        "multiverse_client": {
                            "host": ros_control_dict["host"],
                            "port": ros_control_dict["port"],
                            "meta_data": ros_control_dict["meta_data"],
                        },
                        "controller_manager": {"robot": robot, "robot_description": robot_description, "actuators": actuators},
                    }
                    cmd = [
                        "rosrun",
                        "multiverse_control",
                        "multiverse_control_node",
                        f"{multiverse_dict}".replace(" ", "").replace("'", '"').replace('"', '"'),
                    ]
                    process = run_subprocess(cmd)

                    control_config_path = find_files(resources_paths, controller_manager["config"])
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


if __name__ == "__main__":
    main()
