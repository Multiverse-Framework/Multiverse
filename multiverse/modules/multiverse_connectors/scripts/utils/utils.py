#!/usr/bin/env python3.10

from typing import List
import glob
import subprocess
import os
import xml.etree.ElementTree as ET


def run_subprocess(cmd: List[str]) -> subprocess.Popen:
    cmd_str = " ".join(cmd)
    print(f'Execute "{cmd_str}"')
    return subprocess.Popen(cmd)


def find_files(resources_paths: List[str], filename_pattern: str) -> str:
    matches = []
    for resources_path in resources_paths:
        search_pattern = os.path.join(resources_path, "**", filename_pattern)
        matches += [path for path in glob.iglob(search_pattern, recursive=True)]
    if len(matches) <= 0:
        raise FileNotFoundError(f"{filename_pattern} not found in {resources_paths}")
    else:
        if len(matches) > 1:
            print(f"Found multiple {filename_pattern} files, using the first one")
        return matches[0]


def get_urdf_str_from_ros_package(mesh_abspath_prefix: str, ros_pkg_path: str, urdf_path: str) -> str:
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    robot_urdf_str = ET.tostring(root, encoding="unicode")
    mesh_relpath_prefix = os.path.relpath(os.path.dirname(urdf_path), ros_pkg_path)
    mesh_relpath_prefix = os.path.join("package://multiverse_control", mesh_relpath_prefix) + "/"
    robot_urdf_str = robot_urdf_str.replace("file:///", mesh_abspath_prefix)
    robot_urdf_str = robot_urdf_str.replace("file://", mesh_relpath_prefix)
    return robot_urdf_str


def get_urdf_str_abs(urdf_path: str) -> str:
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    robot_urdf_str = ET.tostring(root, encoding="unicode")
    mesh_abspath_prefix = "file://" + os.path.dirname(urdf_path) + "/"
    robot_urdf_str = robot_urdf_str.replace("file://", mesh_abspath_prefix)
    return robot_urdf_str
