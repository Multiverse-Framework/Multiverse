#!/usr/bin/env python3.10
import argparse
import glob
import os
import subprocess
import xml.etree.ElementTree as ET
from typing import List, Any, Dict, Optional

import yaml


def get_muv_file() -> str:
    parser = argparse.ArgumentParser(prog="multiverse_launch", description="Launch the multiverse framework")
    parser.add_argument(
        "--muv_file",
        type=str,
        required=True,
        help="Path to .muv file",
    )
    args = parser.parse_args()
    return args.muv_file


class MultiverseLaunch:
    muv_file: str
    data: Dict[str, Any]
    resources_paths: List[str]

    def __init__(self):
        self.muv_file = get_muv_file()
        self.data = self.read_muv_file()

    def read_muv_file(self) -> Dict[str, Any]:
        with open(self.muv_file, "r") as f:
            data = yaml.safe_load(f)
        return data

    @property
    def resources_paths(self) -> List[str]:
        resources_paths = self.data.get("resources", ["../robots", "../worlds", "../objects"])
        resources_paths = [
            os.path.join(os.path.dirname(self.muv_file), resources_path) if not os.path.isabs(
                resources_path) else resources_path
            for resources_path in resources_paths
        ]
        return resources_paths

    @property
    def multiverse_server(self) -> Dict[str, Any]:
        return self.data.get("multiverse_server", {"host": "tcp://127.0.0.1", "port": 7000})

    @property
    def multiverse_clients(self) -> Dict[str, Any]:
        return self.data.get("multiverse_clients", {})

    @property
    def worlds(self) -> Dict[str, Any]:
        return self.data.get("worlds", {})

    @property
    def simulations(self) -> Dict[str, Any]:
        return self.data.get("simulations", {})


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
