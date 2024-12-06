import unittest

from multiverse_simulator import MultiverseSimulatorConstraints, MultiverseSimulatorState
from mujoco_connector import MultiverseMujocoConnector
from test_multiverse_simulator import MultiverseSimulatorTestCase

import os

resources_path = os.path.join(os.path.dirname(__file__), "..", "resources")


class MultiverseMujocoConnectorBaseTestCase(MultiverseSimulatorTestCase):
    file_path = os.path.join(resources_path, "mjcf/floor/floor.xml")
    Simulator = MultiverseMujocoConnector
    headless = False
    step_size = 1E-3


class MultiverseMujocoConnectorHeadlessBaseTestCase(MultiverseMujocoConnectorBaseTestCase):
    file_path = os.path.join(resources_path, "mjcf/floor/floor.xml")
    Simulator = MultiverseMujocoConnector
    headless = True
    step_size = 1E-3


class MultiverseMujocoConnectorComplexTestCase(MultiverseMujocoConnectorBaseTestCase):
    file_path = os.path.join(resources_path, "mjcf/mujoco_menagerie/franka_emika_panda/mjx_single_cube.xml")
    Simulator = MultiverseMujocoConnector
    headless = False
    step_size = 0.02

    def test_running_in_10s(self):
        simulator = self.test_initialize_multiverse_simulator()
        constraints = MultiverseSimulatorConstraints(max_real_time=10.0)
        simulator.start(constraints=constraints)
        while simulator.state == MultiverseSimulatorState.RUNNING:
            pass
        self.assertIs(simulator.state, MultiverseSimulatorState.STOPPED)


if __name__ == '__main__':
    unittest.main()
