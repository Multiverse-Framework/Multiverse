import os
import time
import unittest
import xml.etree.ElementTree as ET

import numpy

from isaac_sim_connector import MultiverseIsaacSimConnector
from multiverse_simulator import MultiverseSimulatorConstraints, MultiverseSimulatorState, MultiverseViewer, \
    MultiverseFunctionResult
from test_multiverse_simulator import MultiverseSimulatorTestCase

resources_path = os.path.join(os.path.dirname(__file__), "..", "resources")


# @unittest.skip("This test is not meant to be run in CI")
class MultiverseIsaacSimConnectorBaseTestCase(MultiverseSimulatorTestCase):
    file_path = os.path.join(resources_path, "mjcf/floor/floor.xml")
    Simulator = MultiverseIsaacSimConnector
    headless = False
    step_size = 1E-3
    use_mjx = False

    def test_functions(self):
        simulator = self.Simulator(file_path=os.path.join(resources_path, "mjcf/mujoco_menagerie/franka_emika_panda/mjx_single_cube.xml"))
        simulator.start(run_in_thread=False)

        for step in range(10000):
            if step < 1000:
                result = simulator.get_all_body_names()
                self.assertIsInstance(result, MultiverseFunctionResult)
                self.assertEqual(result.type, MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION)
                self.assertEqual(result.result, ['world', 'link0', 'link1', 'link2', 'link3', 'link4',
                                                 'link5', 'link6', 'link7', 'hand', 'left_finger',
                                                 'right_finger', 'box', 'mocap_target'])

                result = simulator.get_all_joint_names()
                self.assertIsInstance(result, MultiverseFunctionResult)
                self.assertEqual(result.type, MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION)
                self.assertEqual(result.result, ['joint1', 'joint2', 'joint3', 'joint4', 'joint5',
                                                 'joint6', 'joint7', 'finger_joint1', 'finger_joint2', ''])

            if step == 1000 or step == 3000:
                result = simulator.attach(body_1_name="abc")
                self.assertEqual(MultiverseFunctionResult.ResultType.FAILURE_BEFORE_EXECUTION_ON_MODEL, result.type)
                self.assertEqual("Body 1 abc not found", result.info)

                result = simulator.attach(body_1_name="world")
                self.assertEqual(MultiverseFunctionResult.ResultType.FAILURE_BEFORE_EXECUTION_ON_MODEL, result.type)
                self.assertEqual("Body 1 and body 2 are the same", result.info)

                result = simulator.attach(body_1_name="box")
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION, result.type)
                self.assertEqual("Body 1 box is already attached to body 2 world", result.info)

                result = simulator.attach(body_1_name="box", body_2_name="hand")
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_MODEL, result.type)
                self.assertIn("Attached body 1 box to body 2 hand", result.info)

                result = simulator.attach(body_1_name="box", body_2_name="hand")
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION, result.type)
                self.assertEqual("Body 1 box is already attached to body 2 hand", result.info)

            if step == 2000 or step == 4000:
                result = simulator.detach(body_name="abc")
                self.assertEqual(MultiverseFunctionResult.ResultType.FAILURE_BEFORE_EXECUTION_ON_MODEL, result.type)
                self.assertEqual("Body abc not found", result.info)

                result = simulator.detach(body_name="world")
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION, result.type)
                self.assertEqual("Body world is already detached", result.info)

                result = simulator.detach(body_name="box")
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_MODEL, result.type)
                self.assertEqual("Detached body box from body hand", result.info)

                result = simulator.detach(body_name="box")
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION, result.type)
                self.assertEqual("Body box is already detached", result.info)

            if step == 8000:
                result = simulator.get_contact_bodies(body_name="abc")
                self.assertEqual(MultiverseFunctionResult.ResultType.FAILURE_WITHOUT_EXECUTION, result.type)
                self.assertEqual("Body abc not found", result.info)

                result = simulator.get_contact_bodies(body_name="hand")
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION, result.type)
                self.assertIsInstance(result.result, set)

            simulator.step()
            simulator.run_callback()
            time.sleep(0.001)
        simulator.stop()


class MultiverseIsaacSimConnectorHeadlessBaseTestCase(MultiverseIsaacSimConnectorBaseTestCase):
    world_path = os.path.join(resources_path, "usd/floor/floor.usda")
    robots_path = os.path.join(resources_path, "usd/isaac_sim_examples/h1_only_primitives.usda")
    Simulator = MultiverseIsaacSimConnector
    headless = True
    step_size = 1E-3
    use_mjx = False

# @unittest.skip("This test is not meant to be run in CI")
class MultiverseIsaacSimConnectorComplexTestCase(MultiverseIsaacSimConnectorBaseTestCase):
    world_path = os.path.join(resources_path, "usd/floor/floor.usda")
    robots_path = os.path.join(resources_path, "usd/isaac_sim_examples/h1_only_primitives.usda")
    Simulator = MultiverseIsaacSimConnector
    headless = False
    step_size = 1E-3
    number_of_envs = 10

    def test_running_in_10s(self):
        simulator = self.test_initialize_multiverse_simulator()
        simulator.start(run_in_thread=False)
        start_time = time.time()
        while time.time() - start_time < 10.0:
            simulator.step()
        simulator.stop()
        self.assertIs(simulator.state, MultiverseSimulatorState.STOPPED)

    def test_running_in_10s_with_viewer(self):
        write_objects = {
            "left_ankle_joint": {
                "joint_rvalue": [0.0],
            },
            "right_ankle_joint": {
                "joint_rvalue": [0.0]
            },
        }
        read_objects = {
            "left_ankle_joint": {
                "joint_rvalue": [0.0],
                "joint_angular_velocity": [0.0]
            },
            "right_ankle_joint": {
                "joint_rvalue": [0.0],
                "joint_angular_velocity": [0.0]
            },
        }
        viewer = MultiverseViewer(write_objects=write_objects, read_objects=read_objects)
        simulator = self.test_initialize_multiverse_simulator(viewer=viewer)
        simulator.start(run_in_thread=False)
        use_write_data = True
        start_time = time.time()
        while time.time() - start_time < 100.0:
            time_now = time.time()
            f = 0.5
            act_1_value = numpy.pi / 6 * numpy.sin(2.0 * numpy.pi * f * time_now)
            act_2_value = numpy.pi / 6 * numpy.sin(-1.0 * numpy.pi * f * time_now)
            if use_write_data:
                viewer.write_data = numpy.array([[act_1_value, act_2_value] for _ in range(self.number_of_envs)])
            else:
                viewer.write_objects["left_ankle_joint"]["joint_rvalue"].values[:][0] = act_1_value
                viewer.write_objects["right_ankle_joint"]["joint_rvalue"].values[:][0] = act_2_value
            use_write_data = not use_write_data
            simulator.step()
            self.assertEqual(viewer.read_data.shape, (10, 4))
            # if simulator.current_real_time - start_time > 1.0:
            #     self.assertAlmostEqual(viewer.read_objects["left_ankle_joint"]["joint_rvalue"].values[0][0], act_1_value,
            #                            places=0)
            #     self.assertAlmostEqual(viewer.read_objects["right_ankle_joint"]["joint_rvalue"].values[0][0], act_2_value,
            #                            places=0)
        simulator.stop()
        self.assertIs(simulator.state, MultiverseSimulatorState.STOPPED)


if __name__ == '__main__':
    unittest.main()
