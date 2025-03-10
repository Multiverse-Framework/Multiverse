#!/usr/bin/env python3

import re
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
    if os.path.isabs(filename_pattern):
        return filename_pattern
    matches = []
    for resources_path in resources_paths:
        search_pattern = os.path.join(resources_path, "**", filename_pattern)
        matches += [path for path in glob.iglob(search_pattern, recursive=True)]
    if len(matches) <= 0:
        raise FileNotFoundError(f"{filename_pattern} not found in {resources_paths}")
    else:
        if len(matches) > 1:
            print(f"Found multiple {filename_pattern} files, using the first one")
        match = os.path.normpath(matches[0])
        return match


def get_urdf_str_from_ros_package(mesh_abspath_prefix: str, ros_pkg_path: str, urdf_path: str) -> str:
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    robot_urdf_str = ET.tostring(root, encoding="unicode")
    mesh_relpath_prefix = os.path.relpath(os.path.dirname(urdf_path), ros_pkg_path)
    mesh_relpath_prefix = os.path.join("package://multiverse_control", mesh_relpath_prefix) + "/"
    robot_urdf_str = robot_urdf_str.replace("file:///", mesh_abspath_prefix + "/")
    robot_urdf_str = robot_urdf_str.replace("file://", mesh_relpath_prefix)
    return robot_urdf_str


def get_urdf_str_abs(urdf_path: str) -> str:
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    for link in root.findall(".//link/visual/geometry/mesh"):
        filename = link.get("filename")
        if not filename.startswith("package://") and not filename.startswith("file://"):
            link.set("filename", f"file://{os.path.abspath(os.path.join(os.path.dirname(urdf_path), filename))}")
    for link in root.findall(".//link/collision/geometry/mesh"):
        filename = link.get("filename")
        if not filename.startswith("package://") and not filename.startswith("file://"):
            link.set("filename", f"file://{os.path.abspath(os.path.join(os.path.dirname(urdf_path), filename))}")
    robot_urdf_str = ET.tostring(root, encoding="unicode")
    mesh_abspath_prefix = os.path.dirname(urdf_path) + "/"
    pattern = r'filename="file://([^"]*)"'
    matches = re.findall(pattern, robot_urdf_str)
    for match in matches:
        if match.startswith('/'):
            continue  # Skip if it's not an absolute path
        robot_urdf_str = robot_urdf_str.replace(f'filename="file://{match}"', f'filename="file://{mesh_abspath_prefix}{match}"')
    return robot_urdf_str


def is_roscore_running() -> bool:
    import rosgraph
    return rosgraph.is_master_online()