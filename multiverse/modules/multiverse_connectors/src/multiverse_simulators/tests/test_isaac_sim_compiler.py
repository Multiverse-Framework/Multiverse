#!/usr/bin/env python3

import argparse
import os
import unittest
from unittest.mock import patch

from isaac_sim_connector import IsaacSimCompiler
from multiverse_simulator import multiverse_simulator_compiler_main

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


class IsaacSimCompilerTestCase(MultiverseSimulatorCompilerTestCase):
    world_path = os.path.join(resources_path, "usd/floor/floor.usda")
    franka_emika_panda = {
        "panda_arm": {
            "path": os.path.join(resources_path, "usd/isaac_sim_examples/panda_arm.usd"),
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
            "joint": ["joint_rvalue", "joint_tvalue", "joint_angular_velocity", "joint_linear_velocity", "joint_torque",
                      "joint_force"]
        },
        "receive": {}
    }

    @patch('argparse.ArgumentParser.parse_args',
           return_value=argparse.Namespace(name="isaac_sim_simulation_1",
                                           world_path=world_path,
                                           robots="{}",
                                           objects="{}",
                                           references="{}",
                                           save_dir_path=save_dir_path,
                                           multiverse_params=None))
    def test_mujoco_compiler_world(self, _):
        multiverse_simulator_compiler_main(Compiler=IsaacSimCompiler)
        usd_file_path = os.path.join(save_dir_path, "isaac_sim_simulation_1.usda")
        self.assertTrue(os.path.exists(usd_file_path))

    @patch('argparse.ArgumentParser.parse_args',
           return_value=argparse.Namespace(name="isaac_sim_simulation_2",
                                           world_path=world_path,
                                           robots=str(franka_emika_panda),
                                           objects="{}",
                                           references="{}",
                                           save_dir_path=save_dir_path,
                                           multiverse_params=None))
    def test_mujoco_compiler_world_with_robot(self, _):
        multiverse_simulator_compiler_main(Compiler=IsaacSimCompiler)
        usd_file_path = os.path.join(save_dir_path, "isaac_sim_simulation_2.usda")
        self.assertTrue(os.path.exists(usd_file_path))

    @patch('argparse.ArgumentParser.parse_args',
           return_value=argparse.Namespace(name="isaac_sim_simulation_3",
                                           world_path=world_path,
                                           robots=str(franka_emika_panda),
                                           objects="{}",
                                           references="{}",
                                           save_dir_path=save_dir_path,
                                           multiverse_params=str(multiverse_params)))
    def test_mujoco_compiler_world_with_robot_with_multiverse_params(self, _):
        multiverse_simulator_compiler_main(Compiler=IsaacSimCompiler)
        usd_file_path = os.path.join(save_dir_path, "isaac_sim_simulation_3.usda")
        self.assertTrue(os.path.exists(usd_file_path))


if __name__ == "__main__":
    unittest.main()
