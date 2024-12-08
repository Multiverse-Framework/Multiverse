#!/usr/bin/env python3

import unittest
from unittest.mock import patch
import argparse
import os

from multiverse_simulator import multiverse_simulator_compiler_main
from mujoco_connector import MujocoCompiler

resources_path = os.path.join(os.path.dirname(__file__), "..", "resources")
save_dir_path = os.path.join(resources_path, "saved")


class MultiverseSimulatorCompilerTestCase(unittest.TestCase):
    @patch('argparse.ArgumentParser.parse_args',
           return_value=argparse.Namespace(name="multiverse_simulation",
                                           world_path="",
                                           robots="{}",
                                           objects="{}",
                                           references="{}",
                                           save_dir_path=save_dir_path,
                                           multiverse_params=None))
    def test_multiverse_simulator_compiler_main(self, _):
        self.assertRaises(AssertionError, multiverse_simulator_compiler_main)


class MujocoCompilerTestCase(MultiverseSimulatorCompilerTestCase):
    world_path = os.path.join(resources_path, "mjcf/floor/floor.xml")
    franka_emika_pandas = {
        "panda_1": {
            "path": os.path.join(resources_path, "mjcf/mujoco_menagerie/franka_emika_panda/panda.xml"),
            "apply": {
                "body": {
                    "gravcomp": 1,
                    "link0": {
                        "pos": [-0.5, 0.5, 1.0],
                        "quat": [0.5, 0.5, 0.5, 0.5],
                    }
                }
            }
        },
        "panda_2": {
            "path": os.path.join(resources_path, "mjcf/mujoco_menagerie/franka_emika_panda/panda.xml"),
            "apply": {
                "body": {
                    "gravcomp": 1,
                    "link0": {
                        "pos": [0.5, -0.5, 1.0],
                        "quat": [-0.5, -0.5, 0.5, 0.5],
                    }
                }
            }
        }
    }
    multiverse_params = {
        "host": "tcp://127.0.0.1",
        "server_port": "7000",
        "client_port": "8000",
        "world_name": "world",
        "simulation_name": "mujoco_simulation_3",
        "send": {
            "body": ["position", "quaternion", "relative_velocity"],
            "joint": ["joint_rvalue", "joint_tvalue", "joint_angular_velocity", "joint_linear_velocity", "joint_torque", "joint_force"]
        },
        "receive": {}
    }

    @patch('argparse.ArgumentParser.parse_args',
           return_value=argparse.Namespace(name="mujoco_simulation_1",
                                           world_path=world_path,
                                           robots="{}",
                                           objects="{}",
                                           references="{}",
                                           save_dir_path=save_dir_path,
                                           multiverse_params=None))
    def test_mujoco_compiler_world(self, _):
        multiverse_simulator_compiler_main(Compiler=MujocoCompiler)
        xml_file_path = os.path.join(save_dir_path, "mujoco_simulation_1.xml")
        self.assertTrue(os.path.exists(xml_file_path))

    @patch('argparse.ArgumentParser.parse_args',
           return_value=argparse.Namespace(name="mujoco_simulation_2",
                                           world_path=world_path,
                                           robots=str(franka_emika_pandas),
                                           objects="{}",
                                           references="{}",
                                           save_dir_path=save_dir_path,
                                           multiverse_params=None))
    def test_mujoco_compiler_world_with_robots(self, _):
        multiverse_simulator_compiler_main(Compiler=MujocoCompiler)
        xml_file_path = os.path.join(save_dir_path, "mujoco_simulation_2.xml")
        self.assertTrue(os.path.exists(xml_file_path))

    @patch('argparse.ArgumentParser.parse_args',
           return_value=argparse.Namespace(name="mujoco_simulation_3",
                                           world_path=world_path,
                                           robots=str(franka_emika_pandas),
                                           objects="{}",
                                           references="{}",
                                           save_dir_path=save_dir_path,
                                           multiverse_params=str(multiverse_params)))
    def test_mujoco_compiler_world_with_robots_and_multiverse(self, _):
        multiverse_simulator_compiler_main(Compiler=MujocoCompiler)


if __name__ == "__main__":
    unittest.main()
