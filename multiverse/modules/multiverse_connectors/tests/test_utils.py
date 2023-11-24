#!/usr/bin/env python3

import argparse
import os
import unittest
from unittest.mock import patch

from launch_simulators import parse_mujoco, MultiverseSimulationLaunch
from multiverse_launch import MultiverseLaunch
from multiverse_launch import get_muv_file, run_subprocess

CURRENT_PATH = os.path.dirname(os.path.realpath(__file__))
RESOURCES_PATH = os.path.join(CURRENT_PATH, "..", "..", "..", "resources")
MUV_PATH = os.path.join(RESOURCES_PATH, "muv")
DEFAULT_MUV_FILE = os.path.join(MUV_PATH, "default.muv")


class MultiverseLaunchTestCase(unittest.TestCase):
    @patch('argparse.ArgumentParser.parse_args', return_value=argparse.Namespace(muv_file=DEFAULT_MUV_FILE))
    def test_get_muv_file(self, args):
        self.assertEqual(get_muv_file(), DEFAULT_MUV_FILE)

    @patch('argparse.ArgumentParser.parse_args', return_value=argparse.Namespace(muv_file=DEFAULT_MUV_FILE))
    def test_multiverse_launch(self, args):
        multiverse_launch = MultiverseLaunch()
        self.assertEqual(multiverse_launch.muv_file, DEFAULT_MUV_FILE)
        self.assertEqual(multiverse_launch.multiverse_server["host"], "tcp://127.0.0.1")
        self.assertEqual(multiverse_launch.multiverse_server["port"], 7000)
        self.assertEqual(multiverse_launch.resources_paths, [os.path.join(MUV_PATH, "..", "robots"),
                                                             os.path.join(MUV_PATH, "..", "worlds"),
                                                             os.path.join(MUV_PATH, "..", "objects")])

    def test_run_subprocess(self):
        process = run_subprocess(["echo", "test"])
        process.wait()
        self.assertEqual(process.returncode, 0)


class LaunchSimulatorsTestCase(unittest.TestCase):
    mujoco_data = {
        "simulator": "mujoco",
        "world": {
            "name": "world_1",
            "path": "floor.xml"
        },
        "robots": {
            "tiago_dual_1": {
                "path": "tiago_dual.xml",
                "apply": {
                    "body": {
                        "tiago_dual_1": {
                            "pos": [1, 2, 0]
                        },
                        "gravcomp": 1
                    }
                },
                "disable_self_collision": "auto",
                "prefix": {
                    "geom": "tiago_dual_",
                },
                "suffix": {
                    "body": "_1",
                    "joint": "_1",
                    "geom": "_1",
                    "position": "_1"
                }
            },
            "tiago_dual_2": {
                "path": "tiago_dual.xml",
                "apply": {
                    "body": {
                        "tiago_dual_2": {
                            "pos": [-1, -2, 0]
                        },
                    }
                },
                "disable_self_collision": "auto",
                "prefix": {
                    "geom": "tiago_dual_",
                },
                "suffix": {
                    "body": "_2",
                    "joint": "_2",
                    "geom": "_2",
                    "position": "_2"
                }
            }
        },
        "objects": {
            "task_board": {
                "path": "task_board.xml",
                "apply": {
                    "body": {
                        "task_board": {
                            "pos": [0, 0, 1]
                        },
                    }
                },
                "prefix": {
                    "geom": "task_board_",
                },
            }
        }
    }

    def test_parse_mujoco(self):
        resources_paths = [RESOURCES_PATH]
        mujoco_args = parse_mujoco(resources_paths, self.mujoco_data)
        self.assertIn('--world=' + os.path.join(RESOURCES_PATH, "worlds", "floor", "mjcf", "floor.xml"), mujoco_args)

    @patch('argparse.ArgumentParser.parse_args', return_value=argparse.Namespace(muv_file=DEFAULT_MUV_FILE))
    def test_run_simulator_compile(self, args):
        multiverse_simulation_launch = MultiverseSimulationLaunch()
        result = multiverse_simulation_launch.run_simulator_compile(simulation_name="sim_1",
                                                                    simulation_data=self.mujoco_data)
        self.assertEqual(result.returncode, 0)
        self.assertIn("World:", result.stdout)
        process = multiverse_simulation_launch.run_simulator(result, simulation_name="sim_1",
                                                             simulation_data=self.mujoco_data)
        process.wait()
        self.assertEqual(process.returncode, 0)


if __name__ == '__main__':
    unittest.main()
