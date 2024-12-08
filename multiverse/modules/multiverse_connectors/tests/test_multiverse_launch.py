#!/usr/bin/env python3

import argparse
import os
import unittest
from time import sleep
from unittest.mock import patch

from multiverse_ros_socket import Interface, INTERFACE

from launch_ros import MultiverseRosLaunch
from launch_simulators import parse_mujoco, MultiverseSimulationLaunch
from multiverse_launch import MultiverseLaunch, get_muv_file
from utils import run_subprocess

CURRENT_PATH = os.path.dirname(os.path.realpath(__file__))
RESOURCES_PATH = os.path.join(CURRENT_PATH, "..", "..", "..", "resources")
MUV_PATH = os.path.join(RESOURCES_PATH, "muv")
DEFAULT_MUV_FILE = os.path.join(MUV_PATH, "table_with_bowling.muv")


class MultiverseLaunchTestCase(unittest.TestCase):
    @patch('argparse.ArgumentParser.parse_args', return_value=argparse.Namespace(muv_file=DEFAULT_MUV_FILE))
    def test_get_muv_file(self, _):
        self.assertEqual(get_muv_file(), DEFAULT_MUV_FILE)

    @patch('argparse.ArgumentParser.parse_args', return_value=argparse.Namespace(muv_file=DEFAULT_MUV_FILE))
    def test_multiverse_launch(self, _):
        multiverse_launch = MultiverseLaunch()
        self.assertEqual(multiverse_launch.muv_file, DEFAULT_MUV_FILE)
        self.assertEqual(multiverse_launch.multiverse_server["host"], "tcp://127.0.0.1")
        self.assertEqual(multiverse_launch.multiverse_server["port"], 7000)
        resources_paths = [os.path.join(MUV_PATH, "..", "robots"),
                           os.path.join(MUV_PATH, "..", "worlds"),
                           os.path.join(MUV_PATH, "..", "objects")]
        resources_paths = [os.path.abspath(path) for path in resources_paths]
        self.assertEqual(multiverse_launch.resources_paths, resources_paths)

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
    }

    def test_parse_mujoco(self):
        resources_paths = [RESOURCES_PATH]
        mujoco_args = parse_mujoco(resources_paths, self.mujoco_data)
        self.assertIn('--world=' + os.path.join(RESOURCES_PATH, "worlds", "floor", "mjcf", "floor.xml"), mujoco_args)

    @patch('argparse.ArgumentParser.parse_args', return_value=argparse.Namespace(muv_file=DEFAULT_MUV_FILE))
    def test_run_mujoco(self, _):
        multiverse_simulation_launch = MultiverseSimulationLaunch()
        result = multiverse_simulation_launch.run_simulator_compile(simulation_name="sim_1",
                                                                    simulation_data=self.mujoco_data)
        self.assertEqual(result.returncode, 0)
        self.assertIn("World:", result.stdout)
        process = multiverse_simulation_launch.run_simulator(result, simulation_name="sim_1",
                                                             simulation_data=self.mujoco_data)
        process.wait()
        self.assertEqual(process.returncode, 0)


class LaunchRosTestCase(unittest.TestCase):
    processes = []

    @classmethod
    def tearDownClass(cls):
        for process in cls.processes:
            process.kill()
            process.wait()
        if INTERFACE == Interface.ROS1:
            run_subprocess(["killall", "rosmaster"]).wait()
            run_subprocess(["killall", "rosout"]).wait()

    @patch('argparse.ArgumentParser.parse_args', return_value=argparse.Namespace(muv_file=DEFAULT_MUV_FILE))
    def test_run_ros(self, _):
        self.processes += [run_subprocess(["multiverse_server"])]
        self.assertEqual(len(self.processes), 1)
        multiverse_simulation_launch = MultiverseSimulationLaunch()
        self.processes += multiverse_simulation_launch.run_simulations()
        self.assertEqual(len(self.processes), 3)
        multiverse_ros_launch = MultiverseRosLaunch()
        self.processes += multiverse_ros_launch.start_ros_socket()
        self.assertEqual(len(self.processes), 8)
        sleep(5)


if __name__ == '__main__':
    unittest.main()
