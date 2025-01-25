#!/usr/bin/env python3

import subprocess
import os
from .utils import find_files, get_urdf_str_from_ros_package, get_urdf_str_abs, run_subprocess
import yaml
import shutil

def run_rviz(rviz_dict, resources_paths, mesh_abspath_prefix, multiverse_control_pkg_path) -> subprocess.Popen:
    import rospy
    for robot_description, urdf_path in rviz_dict.get("robot_descriptions", {}).items():
        prefix = ""
        suffix = ""
        if isinstance(urdf_path, dict):
            prefix = urdf_path.get("prefix", "")
            suffix = urdf_path.get("suffix", "")
            urdf_path = urdf_path.get("path")

        if urdf_path.endswith(".urdf"):
            urdf_path = find_files(resources_paths, urdf_path)
            urdf_str = get_urdf_str_from_ros_package(mesh_abspath_prefix, multiverse_control_pkg_path, urdf_path)
            if prefix != "" or suffix != "":
                from xml.etree import ElementTree as ET
                root = ET.fromstring(urdf_str)
                for link in root.findall(".//link"):
                    link_name = link.get("name")
                    print(f"Link name: {link_name}")
                    link.set("name", f"{prefix}{link_name}{suffix}")
                urdf_str = ET.tostring(root, encoding="unicode")
            rospy.set_param(f"/{robot_description}", f"{urdf_str}")
        else:
            saved_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "saved")
            model_path = find_files([saved_path], urdf_path)
            if model_path.endswith(".xml"):
                from multiverse_parser import MjcfImporter, UrdfExporter
                print("Converting MJCF to URDF")
                factory = MjcfImporter(file_path=model_path,
                                    fixed_base=True,
                                    with_physics=False,
                                    with_visual=True,
                                    with_collision=True)
                factory.import_model()
                model_dir = os.path.dirname(model_path)
                urdf_path = os.path.join(model_dir, f"{urdf_path.replace('.xml', '.urdf')}")
                exporter = UrdfExporter(file_path=urdf_path,
                                        factory=factory)
                exporter.build()
                exporter.export(keep_usd=True)
                print("Conversion complete and URDF saved at: ", urdf_path)
                urdf_str = get_urdf_str_from_ros_package(mesh_abspath_prefix, multiverse_control_pkg_path, urdf_path)
                rospy.set_param(f"/{robot_description}", f"{urdf_str}")

    if "config" in rviz_dict:
        rviz_config_path = find_files(resources_paths, rviz_dict["config"])
    else:
        default_rviz_config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config", "rviz.rviz")
        rviz_config_path = os.path.join("/tmp", "rviz.rviz")
        shutil.copy(default_rviz_config_path, rviz_config_path)

        with open(rviz_config_path) as file:
            rviz_config = yaml.safe_load(file)
        rviz_config["Visualization Manager"]["Displays"] = [rviz_config["Visualization Manager"]["Displays"][0]]
        for robot_description in rviz_dict.get("robot_descriptions", {}).keys():
            rviz_config["Visualization Manager"]["Displays"].append({
                "Alpha": 1.0,
                "Class": "rviz/RobotModel",
                "Collision Enabled": True,
                "Enabled": True,
                "Links": {
                    "All Links Enabled": True,
                    "Expand Joint Details": False,
                    "Expand Link Details": False,
                    "Expand Tree": False,
                    "Link Tree Style": ""
                },
                "Name": f"{robot_description}",
                "Robot Description": f"{robot_description}",
                "TF Prefix": "",
                "Update Interval": 0,
                "Value": True,
                "Visual Enabled": True
            })
        with open(rviz_config_path, 'w') as file:
            yaml.dump(rviz_config, file)

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
