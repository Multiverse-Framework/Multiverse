import os
import time
import unittest
import xml.etree.ElementTree as ET

import mujoco
import mujoco.viewer
import numpy

from mujoco_connector import MultiverseMujocoConnector
from multiverse_simulator import MultiverseSimulatorConstraints, MultiverseSimulatorState, MultiverseViewer, \
    MultiverseFunctionResult
from test_multiverse_simulator import MultiverseSimulatorTestCase

resources_path = os.path.join(os.path.dirname(__file__), "..", "resources")


# @unittest.skip("This test is not meant to be run in CI")
class MultiverseMujocoConnectorBaseTestCase(MultiverseSimulatorTestCase):
    file_path = os.path.join(resources_path, "mjcf/floor/floor.xml")
    Simulator = MultiverseMujocoConnector
    headless = False
    step_size = 1E-3
    use_mjx = False

    def test_functions(self):
        simulator = self.Simulator(
            file_path=os.path.join(resources_path, "mjcf/mujoco_menagerie/franka_emika_panda/mjx_single_cube.xml"))
        simulator.start(run_in_thread=False)

        for step in range(10000):
            if step < 1000:
                result = simulator.callbacks["get_all_body_names"]()
                self.assertIsInstance(result, MultiverseFunctionResult)
                self.assertEqual(result.type, MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION)
                self.assertEqual(result.result, ['world', 'link0', 'link1', 'link2', 'link3', 'link4',
                                                 'link5', 'link6', 'link7', 'hand', 'left_finger',
                                                 'right_finger', 'box', 'mocap_target'])

                result = simulator.callbacks["get_all_joint_names"]()
                self.assertIsInstance(result, MultiverseFunctionResult)
                self.assertEqual(result.type, MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION)
                self.assertEqual(result.result, ['joint1', 'joint2', 'joint3', 'joint4', 'joint5',
                                                 'joint6', 'joint7', 'finger_joint1', 'finger_joint2'])

            if step == 1000 or step == 3000:
                result = simulator.callbacks["attach"](body_1_name='abc')
                self.assertEqual(MultiverseFunctionResult.ResultType.FAILURE_BEFORE_EXECUTION_ON_MODEL, result.type)
                self.assertEqual("Body 1 abc not found", result.info)

                result = simulator.callbacks["attach"](body_1_name="world")
                self.assertEqual(MultiverseFunctionResult.ResultType.FAILURE_BEFORE_EXECUTION_ON_MODEL, result.type)
                self.assertEqual("Body 1 and body 2 are the same", result.info)

                result = simulator.callbacks["attach"](body_1_name="box")
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION, result.type)
                self.assertEqual("Body 1 box is already attached to body 2 world", result.info)

                result = simulator.callbacks["attach"](body_1_name="box", body_2_name="hand")
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_MODEL, result.type)
                self.assertIn("Attached body 1 box to body 2 hand", result.info)

                result = simulator.callbacks["attach"](body_1_name="box", body_2_name="hand")
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION, result.type)
                self.assertEqual("Body 1 box is already attached to body 2 hand", result.info)

            if step == 1200:
                result = simulator.callbacks["get_joint_value"](joint_name="joint1")
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION, result.type)
                self.assertIsInstance(result.result, float)
                joint1_value = result.result

                result = simulator.callbacks["get_joints_values"](joint_names=["joint1", "joint2"])
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION, result.type)
                self.assertIsInstance(result.result, dict)
                self.assertEqual(len(result.result), 2)
                self.assertIn("joint1", result.result)
                self.assertIsInstance(result.result["joint1"], float)
                self.assertIn("joint2", result.result)
                self.assertIsInstance(result.result["joint2"], float)
                self.assertEqual(joint1_value, result.result["joint1"])

                result = simulator.callbacks["get_body_position"](body_name="box")
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION, result.type)
                self.assertIsInstance(result.result, numpy.ndarray)
                box_position = result.result

                result = simulator.callbacks["get_body_quaternion"](body_name="box")
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION, result.type)
                self.assertIsInstance(result.result, numpy.ndarray)
                box_quaternion = result.result

                result = simulator.callbacks["get_bodies_positions"](body_names=["box", "link0"])
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION, result.type)
                self.assertIsInstance(result.result, dict)
                self.assertEqual(len(result.result), 2)
                self.assertIn("box", result.result)
                self.assertIsInstance(result.result["box"], numpy.ndarray)
                self.assertIn("link0", result.result)
                self.assertIsInstance(result.result["link0"], numpy.ndarray)
                self.assertTrue(numpy.allclose(box_position, result.result["box"]))

                result = simulator.callbacks["get_bodies_quaternions"](body_names=["box", "link0"])
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION, result.type)
                self.assertIsInstance(result.result, dict)
                self.assertEqual(len(result.result), 2)
                self.assertIn("box", result.result)
                self.assertIsInstance(result.result["box"], numpy.ndarray)
                self.assertIn("link0", result.result)
                self.assertIsInstance(result.result["link0"], numpy.ndarray)
                self.assertTrue(numpy.allclose(box_quaternion, result.result["box"]))

            if step == 800:
                box_position = numpy.array([0.7, 0.0, 1.0])
                result = simulator.callbacks["set_body_position"](body_name="box", position=box_position)
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_DATA, result.type)

                result = simulator.callbacks["get_body_position"](body_name="box")
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION, result.type)
                self.assertIsInstance(result.result, numpy.ndarray)
                self.assertTrue(numpy.allclose(box_position, result.result))

                box_position = numpy.array([0.7, 0.0, 2.0])
                result = simulator.callbacks["set_bodies_positions"](bodies_positions={"box": box_position})
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_DATA, result.type)

                result = simulator.callbacks["get_body_position"](body_name="box")
                self.assertTrue(numpy.allclose(box_position, result.result))

                box_quaternion = numpy.array([0.707, 0.707, 0.0, 0.0])
                box_quaternion /= numpy.linalg.norm(box_quaternion)
                result = simulator.callbacks["set_body_quaternion"](body_name="box", quaternion=box_quaternion)
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_DATA, result.type)

                result = simulator.callbacks["get_body_quaternion"](body_name="box")
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION, result.type)
                self.assertIsInstance(result.result, numpy.ndarray)
                self.assertTrue(numpy.allclose(box_quaternion, result.result))

                box_quaternion = numpy.array([0.707, 0.0, 0.707, 0.0])
                box_quaternion /= numpy.linalg.norm(box_quaternion)
                result = simulator.callbacks["set_bodies_quaternions"](bodies_quaternions={"box": box_quaternion})
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_DATA, result.type)

                result = simulator.callbacks["get_body_quaternion"](body_name="box")
                self.assertTrue(numpy.allclose(box_quaternion, result.result))

                joint1_value = 0.3
                result = simulator.callbacks["set_joint_value"](joint_name="joint1", value=joint1_value)
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_DATA, result.type)

                result = simulator.callbacks["get_joint_value"](joint_name="joint1")
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION, result.type)
                self.assertIsInstance(result.result, float)
                self.assertAlmostEqual(joint1_value, result.result, places=3)

                joints_values = {
                    "joint1": joint1_value,
                    "joint2": 0.5
                }
                result = simulator.callbacks["set_joints_values"](joints_values=joints_values)
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_DATA, result.type)

                result = simulator.callbacks["get_joints_values"](joint_names=["joint1", "joint2"])
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION, result.type)
                self.assertIsInstance(result.result, dict)
                self.assertEqual(len(result.result), 2)
                self.assertIn("joint1", result.result)
                self.assertIsInstance(result.result["joint1"], float)
                self.assertIn("joint2", result.result)
                self.assertIsInstance(result.result["joint2"], float)
                self.assertAlmostEqual(joint1_value, result.result["joint1"], places=3)

            if step == 1550:
                result = simulator.callbacks["save"](key_name="step_1550")
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION, result.type)
                self.key_id = result.result

            if step == 1560:
                result = simulator.callbacks["load"](key_id=self.key_id)
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_DATA, result.type)
                self.assertEqual(self.key_id, result.result)

            if step == 1570:
                self.save_file_path = os.path.join(resources_path, "../output/step_1570.xml")
                if not os.path.exists(os.path.dirname(self.save_file_path)):
                    os.makedirs(os.path.dirname(self.save_file_path))
                result = simulator.callbacks["save"](file_path=self.save_file_path, key_name="step_1570")
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION, result.type)
                self.key_id = result.result

            if step == 1580:
                result = simulator.callbacks["load"](file_path=self.save_file_path, key_id=self.key_id)
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_DATA, result.type)
                self.assertEqual(self.key_id, result.result)

            if step == 2000 or step == 4000:
                result = simulator.callbacks["detach"](body_name="abc")
                self.assertEqual(MultiverseFunctionResult.ResultType.FAILURE_BEFORE_EXECUTION_ON_MODEL, result.type)
                self.assertEqual("Body abc not found", result.info)

                result = simulator.callbacks["detach"](body_name="world")
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION, result.type)
                self.assertEqual("Body world is already detached", result.info)

                result = simulator.callbacks["detach"](body_name="box")
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_MODEL, result.type)
                self.assertEqual("Detached body box from body hand", result.info)

                result = simulator.callbacks["detach"](body_name="box")
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION, result.type)
                self.assertEqual("Body box is already detached", result.info)

            if step == 8000:
                result = simulator.callbacks["get_contact_bodies"](body_name="abc")
                self.assertEqual(MultiverseFunctionResult.ResultType.FAILURE_WITHOUT_EXECUTION, result.type)
                self.assertEqual("Body abc not found", result.info)

                result = simulator.callbacks["get_contact_bodies"](body_name="hand")
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION, result.type)
                self.assertIsInstance(result.result, set)

            if step == 100:
                result = simulator.callbacks["get_contact_points"](body_1_names=["abc"])
                self.assertEqual(MultiverseFunctionResult.ResultType.FAILURE_WITHOUT_EXECUTION, result.type)
                self.assertEqual("Body abc not found", result.info)

                result = simulator.callbacks["get_contact_points"](body_1_names=["hand"])
                self.assertEqual(MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION, result.type)
                self.assertIsInstance(result.result, list)
                self.assertEqual(len(result.result), 4)

            simulator.step()
            simulator.run_callback()
            time.sleep(0.001)
        simulator.stop()


class MultiverseMujocoConnectorHeadlessBaseTestCase(MultiverseMujocoConnectorBaseTestCase):
    file_path = os.path.join(resources_path, "mjcf/floor/floor.xml")
    Simulator = MultiverseMujocoConnector
    headless = True
    step_size = 1E-3
    use_mjx = False


# @unittest.skip("This test is not meant to be run in CI")
class MultiverseMujocoConnectorComplexTestCase(MultiverseMujocoConnectorBaseTestCase):
    file_path = os.path.join(resources_path, "mjcf/mujoco_menagerie/franka_emika_panda/mjx_single_cube.xml")
    Simulator = MultiverseMujocoConnector
    headless = False
    step_size = 5E-4

    def test_running_in_10s(self):
        simulator = self.test_initialize_multiverse_simulator()
        constraints = MultiverseSimulatorConstraints(max_real_time=10.0)
        simulator.start(constraints=constraints)
        while simulator.state != MultiverseSimulatorState.STOPPED:
            time.sleep(1)
        self.assertIs(simulator.state, MultiverseSimulatorState.STOPPED)

    def test_running_in_10s_with_viewer(self):
        write_objects = {
            "actuator1": {
                "cmd_joint_rvalue": [0.0]
            },
            "actuator2": {
                "cmd_joint_rvalue": [0.0]
            },
        }
        read_objects = {
            "joint1": {
                "joint_rvalue": [0.0],
                "joint_angular_velocity": [0.0]
            },
            "joint2": {
                "joint_rvalue": [0.0],
                "joint_angular_velocity": [0.0]
            },
            "actuator1": {
                "cmd_joint_rvalue": [0.0]
            },
            "actuator2": {
                "cmd_joint_rvalue": [0.0]
            },
            "world": {
                "energy": [0.0, 0.0]
            }
        }
        viewer = MultiverseViewer(write_objects=write_objects, read_objects=read_objects, logging_interval=0.01)
        simulator = self.test_initialize_multiverse_simulator(viewer=viewer)
        constraints = MultiverseSimulatorConstraints(max_simulation_time=10.0)
        simulator.start(constraints=constraints)
        use_write_data = False
        while simulator.state != MultiverseSimulatorState.STOPPED:
            time_now = time.time()
            f = 0.5
            act_1_value = numpy.pi / 6 * numpy.sin(2.0 * numpy.pi * f * time_now)
            act_2_value = numpy.pi / 6 * numpy.sin(-1.0 * numpy.pi * f * time_now)
            if use_write_data:
                viewer.write_data = numpy.array([[act_1_value, act_2_value]])
            else:
                write_objects = viewer.write_objects
                write_objects["actuator1"]["cmd_joint_rvalue"].values[0][0] = act_1_value
                write_objects["actuator2"]["cmd_joint_rvalue"].values[0][0] = act_2_value
                viewer.write_objects = write_objects
            use_write_data = not use_write_data
            time.sleep(0.01)
            self.assertEqual(viewer.read_data.shape, (1, 8))
            if simulator.current_simulation_time > 1.0:
                self.assertAlmostEqual(viewer.read_objects["joint1"]["joint_rvalue"].values[0][0], act_1_value,
                                       places=0)
                self.assertAlmostEqual(viewer.read_objects["joint2"]["joint_rvalue"].values[0][0], act_2_value,
                                       places=0)
        self.assertIs(simulator.state, MultiverseSimulatorState.STOPPED)
        save_file_path = os.path.join(resources_path, "../output/data.csv")
        if os.path.exists(save_file_path):
            os.remove(save_file_path)
            self.assertFalse(os.path.exists(save_file_path))
        viewer.logger.save_data(save_file_path)
        self.assertTrue(os.path.exists(save_file_path))

    def test_read_and_write_data_in_the_loop(self):
        viewer = MultiverseViewer()
        simulator = self.test_initialize_multiverse_simulator(viewer=viewer)
        simulator.start(run_in_thread=False)
        for step in range(10000):
            if step == 100:
                read_objects = {
                    "joint1": {
                        "joint_rvalue": [0.0],
                        "joint_angular_velocity": [0.0]
                    },
                    "joint2": {
                        "joint_rvalue": [0.0],
                        "joint_angular_velocity": [0.0]
                    }
                }
                viewer.read_objects = read_objects
            elif step == 101:
                read_objects = {
                    "joint1": {
                        "joint_angular_velocity": [0.0]
                    },
                    "joint2": {
                        "joint_rvalue": [0.0],
                        "joint_torque": [0.0]
                    }
                }
                viewer.read_objects = read_objects
            elif step == 102:
                write_objects = {
                    "joint1": {
                        "joint_rvalue": [1.0]
                    },
                    "actuator2": {
                        "cmd_joint_rvalue": [2.0]
                    },
                    "box": {
                        "position": [1.1, 2.2, 3.3],
                        "quaternion": [0.707, 0.0, 0.707, 0.0]
                    }
                }
                read_objects = {
                    "joint1": {
                        "joint_rvalue": [0.0],
                        "joint_angular_velocity": [0.0]
                    },
                    "actuator2": {
                        "cmd_joint_rvalue": [0.0]
                    },
                    "box": {
                        "position": [0.0, 0.0, 0.0],
                        "quaternion": [0.0, 0.0, 0.0, 0.0]
                    }
                }
                viewer.write_objects = write_objects
                viewer.read_objects = read_objects
            else:
                viewer.read_objects = {}
            print(step)
            simulator.step()
            if step == 100:
                self.assertEqual(viewer.read_data.shape, (1, 4))
                self.assertEqual(viewer.read_objects["joint1"]["joint_rvalue"].values.shape, (1, 1))
                self.assertEqual(viewer.read_objects["joint2"]["joint_rvalue"].values.shape, (1, 1))
                self.assertEqual(viewer.read_objects["joint1"]["joint_angular_velocity"].values.shape, (1, 1))
                self.assertEqual(viewer.read_objects["joint2"]["joint_angular_velocity"].values.shape, (1, 1))
            elif step == 101:
                self.assertEqual(viewer.read_data.shape, (1, 3))
                self.assertEqual(viewer.read_objects["joint1"]["joint_angular_velocity"].values.shape, (1, 1))
                self.assertEqual(viewer.read_objects["joint2"]["joint_rvalue"].values.shape, (1, 1))
                self.assertEqual(viewer.read_objects["joint2"]["joint_torque"].values.shape, (1, 1))
            elif step == 102:
                self.assertEqual(viewer.write_data.shape, (1, 9))
                self.assertEqual(viewer.write_objects["joint1"]["joint_rvalue"].values[0], (1.0,))
                self.assertEqual(viewer.write_objects["actuator2"]["cmd_joint_rvalue"].values[0], (2.0,))
                self.assertEqual(viewer.write_objects["box"]["position"].values[0][0], 1.1)
                self.assertEqual(viewer.write_objects["box"]["position"].values[0][1], 2.2)
                self.assertEqual(viewer.write_objects["box"]["position"].values[0][2], 3.3)
                self.assertEqual(viewer.write_objects["box"]["quaternion"].values[0][0], 0.707)
                self.assertEqual(viewer.write_objects["box"]["quaternion"].values[0][1], 0.0)
                self.assertEqual(viewer.write_objects["box"]["quaternion"].values[0][2], 0.707)
                self.assertEqual(viewer.write_objects["box"]["quaternion"].values[0][3], 0.0)
                self.assertEqual(viewer.read_data.shape, (1, 10))
                self.assertAlmostEqual(viewer.read_objects["joint1"]["joint_rvalue"].values[0][0], 1.0, places=3)
                self.assertEqual(viewer.read_objects["actuator2"]["cmd_joint_rvalue"].values[0][0], 2.0)
                self.assertEqual(viewer.read_objects["box"]["position"].values[0][0], 1.1)
                self.assertEqual(viewer.read_objects["box"]["position"].values[0][1], 2.2)
                self.assertAlmostEqual(viewer.read_objects["box"]["position"].values[0][2], 3.3, places=3)
                self.assertAlmostEqual(viewer.read_objects["box"]["quaternion"].values[0][0], 0.7071067811865475)
                self.assertAlmostEqual(viewer.read_objects["box"]["quaternion"].values[0][1], 0.0)
                self.assertAlmostEqual(viewer.read_objects["box"]["quaternion"].values[0][2], 0.7071067811865475)
                self.assertAlmostEqual(viewer.read_objects["box"]["quaternion"].values[0][3], 0.0)
            else:
                self.assertEqual(viewer.read_data.shape, (1, 0))
        simulator.stop()
        self.assertIs(simulator.state, MultiverseSimulatorState.STOPPED)

    def test_running_with_mjx_in_10s(self):
        simulator = MultiverseMujocoConnector(file_path=os.path.join(resources_path, "mjcf/unitree/h1_scene.xml"),
                                              use_mjx=True,
                                              headless=False,
                                              real_time_factor=-1,
                                              step_size=0.001)
        constraints = MultiverseSimulatorConstraints(max_simulation_time=10.0)
        simulator.start(constraints=constraints)
        while simulator.state != MultiverseSimulatorState.STOPPED:
            time.sleep(1)
        self.assertIs(simulator.state, MultiverseSimulatorState.STOPPED)


@unittest.skip("This test is not meant to be run in CI")
class MujocoSpeedTestCase(unittest.TestCase):
    file_path = os.path.join(resources_path, "mjcf/mujoco_menagerie/franka_emika_panda/mjx_single_cube.xml")
    step_size = 5E-4
    real_time_factor = 1.0
    lib_path = os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "..", "build", "mujoco", "lib",
                            "libmultiverse_connector.so")
    include_extension = """
    <extension>
        <plugin plugin="mujoco.multiverse_connector">
              <instance name="mujoco_client">
                    <config key="host" value="tcp://127.0.0.1"/>
                    <config key="server_port" value="7000"/>
                    <config key="client_port" value="7500"/>
                    <config key="world_name" value="world"/>
                    <config key="simulation_name" value="mujoco_sim"/>
                    <config key="send" value="{'world': ['position', 'quaternion']}" />
              </instance>
        </plugin>
    </extension>
    """

    def test_running_speed(self):
        m = mujoco.MjModel.from_xml_path(self.file_path)
        m.opt.timestep = self.step_size
        d = mujoco.MjData(m)
        with mujoco.viewer.launch_passive(m, d) as viewer:
            time_now = time.time()
            start_real_time = time.time()
            while viewer.is_running():
                real_time_pass = time.time() - start_real_time
                simulation_time_pass = d.time * self.real_time_factor
                delta_time = simulation_time_pass - real_time_pass
                if delta_time <= self.step_size:
                    mujoco.mj_step(m, d)
                if delta_time > self.step_size * 10:
                    print(
                        f"Real time is {delta_time} seconds ({delta_time / self.step_size} step_size) behind simulation time")
                elif delta_time < -self.step_size * 10:
                    print(
                        f"Real time is {-delta_time} seconds ({-delta_time / self.step_size} step_size) ahead of simulation time")
                if time.time() - time_now > 1.0 / 60.0:
                    viewer.sync()
                    time_now = time.time()

    def test_running_with_multiverse(self):
        """
        This test must be run with 'multiverse_server' running
        """
        mujoco.mj_loadPluginLibrary(self.lib_path)
        file_xml = ET.parse(os.path.join(resources_path, "mjcf/floor/floor.xml"))
        root = file_xml.getroot()
        root.append(ET.fromstring(self.include_extension))
        file_xml_string = ET.tostring(root, encoding='unicode', method='xml')
        m = mujoco.MjModel.from_xml_string(file_xml_string)
        self.assertIsNotNone(m)
        d = mujoco.MjData(m)
        self.assertEqual(d.time, 0.0)


if __name__ == '__main__':
    unittest.main()
