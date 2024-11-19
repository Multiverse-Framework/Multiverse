#!/usr/bin/env python3

import argparse
import os
from typing import List, Any, Dict

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
        resources_paths = [os.path.abspath(resources_path) for resources_path in resources_paths]
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
