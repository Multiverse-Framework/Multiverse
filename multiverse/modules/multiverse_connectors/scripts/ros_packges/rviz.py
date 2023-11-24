#!/usr/bin/env python3

import rospy
import subprocess
from ..multiverse_launch import find_files, run_subprocess, get_urdf_str_from_ros_package


def run_rviz(rviz_dict, resources_paths, mesh_abspath_prefix, multiverse_control_pkg_path) -> subprocess.Popen:
    for robot_description, urdf_path in rviz_dict.get("robot_descriptions", {}).items():
        urdf_path = find_files(resources_paths, urdf_path)
        urdf_str = get_urdf_str_from_ros_package(mesh_abspath_prefix, multiverse_control_pkg_path,
                                                 urdf_path)
        rospy.set_param(f"/{robot_description}", f"{urdf_str}")

    rviz_config_path = find_files(resources_paths, rviz_dict["config"])
    cmd = [
        "rosrun",
        "rviz",
        "rviz",
        "--display-config",
        f"{rviz_config_path}",
    ]
    return run_subprocess(cmd)
