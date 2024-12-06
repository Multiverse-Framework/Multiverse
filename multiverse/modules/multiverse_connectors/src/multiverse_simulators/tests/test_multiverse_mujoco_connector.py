import unittest

from mujoco_connector import MultiverseMujocoConnector
from test_multiverse_simulator import MultiverseSimulatorTestCase


class MultiverseMujocoConnectorHeadlessTestCase(MultiverseSimulatorTestCase):
    file_path = "../resources/floor/mjcf/floor.xml"
    Simulator = MultiverseMujocoConnector
    headless = True


class MultiverseMujocoConnectorTestCase(MultiverseSimulatorTestCase):
    file_path = "../resources/floor/mjcf/floor.xml"
    Simulator = MultiverseMujocoConnector


if __name__ == '__main__':
    unittest.main()
