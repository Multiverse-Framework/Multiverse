#!/usr/bin/env python3

import subprocess
import os
from .utils import find_files, get_urdf_str_from_ros_package, get_urdf_str_abs, run_subprocess


def run_rviz(rviz_dict, resources_paths, mesh_abspath_prefix, multiverse_control_pkg_path) -> subprocess.Popen:
    import rospy
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


def run_rviz2(rviz2_dict, resources_paths) -> subprocess.Popen:
    for _, urdf_path in rviz2_dict.get("robot_descriptions", {}).items():
        urdf_path = find_files(resources_paths, urdf_path)
        urdf_str = get_urdf_str_abs(urdf_path)
        tmp_urdf_path = os.path.join("/tmp", os.path.basename(urdf_path))
        with open(tmp_urdf_path, 'w') as file:
            file.write(urdf_str)

    rviz2_config_path = find_files(resources_paths, rviz2_dict["config"])
    cmd = [
        "ros2",
        "run",
        "rviz2",
        "rviz2",
        "--display-config",
        f"{rviz2_config_path}",
    ]
    return run_subprocess(cmd)
