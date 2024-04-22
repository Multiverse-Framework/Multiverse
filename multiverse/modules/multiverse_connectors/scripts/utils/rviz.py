#!/usr/bin/env python3

import subprocess
import os
from .utils import find_files, get_urdf_str_from_ros_package, get_urdf_str_abs, run_subprocess
import yaml
from urdf_parser_py import urdf

def run_rviz(rviz_dict, resources_paths, mesh_abspath_prefix, multiverse_control_pkg_path) -> subprocess.Popen:
    import rospy
    for robot_description, urdf_path in rviz_dict.get("robot_descriptions", {}).items():
        urdf_path = find_files(resources_paths, urdf_path)
        urdf_str = get_urdf_str_from_ros_package(mesh_abspath_prefix, multiverse_control_pkg_path,
                                                 urdf_path)
        rospy.set_param(f"/{robot_description}", f"{urdf_str}")
    if "config" in rviz_dict:
        rviz_config_path = find_files(resources_paths, rviz_dict["config"])
    else:
        rviz_config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config", "rviz.rviz")

    if "fix_frame" in rviz_dict:
        with open(rviz_config_path) as file:
            rviz_config = yaml.safe_load(file)
        rviz_config["Visualization Manager"]["Global Options"]["Fixed Frame"] = rviz_dict["fix_frame"]
        with open(rviz_config_path, 'w') as file:
            yaml.dump(rviz_config, file)

    cmd = [
        "rosrun",
        "rviz",
        "rviz",
        "--display-config",
        f"{rviz_config_path}"
    ]
    
    return run_subprocess(cmd)


def run_rviz2(rviz2_dict, resources_paths) -> subprocess.Popen:
    for _, urdf_path in rviz2_dict.get("robot_descriptions", {}).items():
        urdf_path = find_files(resources_paths, urdf_path)
        urdf_str = get_urdf_str_abs(urdf_path)
        tmp_urdf_path = os.path.join("/tmp", os.path.basename(urdf_path))
        with open(tmp_urdf_path, 'w') as file:
            file.write(urdf_str)
    if "config" in rviz2_dict:
        rviz2_config_path = find_files(resources_paths, rviz2_dict["config"])
    else:
        rviz2_config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config", "rviz2.rviz")
        with open(rviz2_config_path) as file:
            rviz2_config = yaml.safe_load(file)
        rviz2_config["Visualization Manager"]["Global Options"]["Fixed Frame"] = rviz2_dict["fix_frame"]
        for display in rviz2_config["Visualization Manager"]["Displays"]:
            if display["Class"] == "rviz_default_plugins/RobotModel":
                display["Description File"] = tmp_urdf_path
        with open(rviz2_config_path, 'w') as file:
            yaml.dump(rviz2_config, file)

    cmd = [
        "ros2",
        "run",
        "rviz2",
        "rviz2",
        "--display-config",
        f"{rviz2_config_path}"
    ]
    
    return run_subprocess(cmd)
