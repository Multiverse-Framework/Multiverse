from typing import Dict, List

from multiverse_client_py import MultiverseClient, MultiverseMetaData

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time
import numpy

class UpperBodyCommand(MultiverseClient, Node):    
    mujoco_arm_joint_state: Dict[str, float] = {
        'mujoco_arm_left_1_joint': numpy.nan,
        'mujoco_arm_left_2_joint': numpy.nan,
        'mujoco_arm_left_3_joint': numpy.nan,
        'mujoco_arm_left_4_joint': numpy.nan,
        'mujoco_arm_left_5_joint': numpy.nan,
        'mujoco_arm_left_6_joint': numpy.nan,
        'mujoco_arm_left_7_joint': numpy.nan,
        'mujoco_arm_right_1_joint': numpy.nan,
        'mujoco_arm_right_2_joint': numpy.nan,
        'mujoco_arm_right_3_joint': numpy.nan,
        'mujoco_arm_right_4_joint': numpy.nan,
        'mujoco_arm_right_5_joint': numpy.nan,
        'mujoco_arm_right_6_joint': numpy.nan,
        'mujoco_arm_right_7_joint': numpy.nan,
    }

    mujoco_left_hand_joint_state: Dict[str, float] = {
        "mujoco_gripper_left_right_finger_joint": numpy.nan, 
        "mujoco_gripper_left_left_finger_joint": numpy.nan,
    }

    mujoco_right_hand_joint_state: Dict[str, float] = {
        "mujoco_gripper_right_right_finger_joint": numpy.nan, 
        "mujoco_gripper_right_left_finger_joint": numpy.nan,
    }

    mujoco_neck_joint_state: Dict[str, float] = {
        "mujoco_head_1_joint": numpy.nan,
        "mujoco_head_2_joint": numpy.nan,
    }

    joint_state_map: Dict[str, str] = {
        "arm_left_1_joint": "mujoco_arm_left_1_joint",
        "arm_left_2_joint": "mujoco_arm_left_2_joint",
        "arm_left_3_joint": "mujoco_arm_left_3_joint",
        "arm_left_4_joint": "mujoco_arm_left_4_joint",
        "arm_left_5_joint": "mujoco_arm_left_5_joint",
        "arm_left_6_joint": "mujoco_arm_left_6_joint",
        "arm_left_7_joint": "mujoco_arm_left_7_joint",
        "arm_right_1_joint": "mujoco_arm_right_1_joint",
        "arm_right_2_joint": "mujoco_arm_right_2_joint",
        "arm_right_3_joint": "mujoco_arm_right_3_joint",
        "arm_right_4_joint": "mujoco_arm_right_4_joint",
        "arm_right_5_joint": "mujoco_arm_right_5_joint",
        "arm_right_6_joint": "mujoco_arm_right_6_joint",
        "arm_right_7_joint": "mujoco_arm_right_7_joint",
        "head_1_joint": "mujoco_head_1_joint",
        "head_2_joint": "mujoco_head_2_joint",
        "gripper_left_right_finger_joint": "mujoco_gripper_left_right_finger_joint",
        "gripper_left_left_finger_joint": "mujoco_gripper_left_left_finger_joint",
        "gripper_right_right_finger_joint": "mujoco_gripper_right_right_finger_joint",
        "gripper_right_left_finger_joint": "mujoco_gripper_right_left_finger_joint",
    }

    arm_publisher = None
    left_hand_publisher = None
    right_hand_publisher = None
    neck_publisher = None
    upper_body_publisher = None
    use_upper_body_publisher: bool = False

    def __init__(self, port: str, multiverse_meta_data: MultiverseMetaData, use_upper_body_publisher: bool = False, rate: float = 100.0) -> None:
        self._transport = "Zmq"
        self._host = "tcp://127.0.0.1"
        self._server_port = "7000"
        MultiverseClient.__init__(self, port=port, multiverse_meta_data=multiverse_meta_data)
        Node.__init__(self, 'arms_command')
        self.use_upper_body_publisher = use_upper_body_publisher
        if self.use_upper_body_publisher:
            self.upper_body_publisher = self.create_publisher(Float64MultiArray, '/upper_body_controller/commands', 10)
        else:
            self.arm_publisher = self.create_publisher(Float64MultiArray, '/arm_position_controller/commands', 10)
            self.left_hand_publisher = self.create_publisher(Float64MultiArray, '/left_gripper_controller/commands', 10)
            self.right_hand_publisher = self.create_publisher(Float64MultiArray, '/right_gripper_controller/commands', 10)
            self.neck_publisher = self.create_publisher(Float64MultiArray, '/head_position_controller/commands', 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.rate = rate

    def joint_state_callback(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            if name not in self.joint_state_map:
                continue
            joint = self.joint_state_map[name]
            for group in (
                self.mujoco_arm_joint_state,
                self.mujoco_left_hand_joint_state,
                self.mujoco_right_hand_joint_state,
                self.mujoco_neck_joint_state,
            ):
                if joint in group:
                    group[joint] = position

    def start(self):
        self.run()
        self.request_meta_data["send"] = {}
        self.request_meta_data["receive"] = {}
        for group in (
            self.mujoco_arm_joint_state,
            self.mujoco_neck_joint_state,
        ):
            for joint_name in group.keys():
                self.request_meta_data["receive"][f"{joint_name}"] = ["cmd_joint_angular_position"]
        for group in (
            self.mujoco_left_hand_joint_state,
            self.mujoco_right_hand_joint_state,
        ):
            for joint_name in group.keys():
                self.request_meta_data["receive"][f"{joint_name}"] = ["cmd_joint_linear_position"]
        self.send_and_receive_meta_data()
        response_meta_data = self.response_meta_data
        joint_indices = {name: i for i, name in enumerate(self.response_meta_data["receive"].keys())}
        arm_indices = [joint_indices[name] for name in self.mujoco_arm_joint_state if name in self.response_meta_data["receive"]]
        left_hand_indices = [joint_indices[name] for name in self.mujoco_left_hand_joint_state if name in self.response_meta_data["receive"]]
        right_hand_indices = [joint_indices[name] for name in self.mujoco_right_hand_joint_state if name in self.response_meta_data["receive"]]
        neck_indices = [joint_indices[name] for name in self.mujoco_neck_joint_state if name in self.response_meta_data["receive"]]

        arm_start_joint_state = {}
        left_hand_start_joint_state = {}
        right_hand_start_joint_state = {}
        neck_start_joint_state = {}
        while True:
            should_break = True
            for object_name, object_attributes in response_meta_data["receive"].items():
                for attribute_name, attribute_data in object_attributes.items():
                    if any(value is None or numpy.isnan(value) for value in attribute_data):
                        self.loginfo(f"Waiting for valid data for {object_name} {attribute_name} {attribute_data}...")
                        should_break = False
            if should_break:
                break
            time.sleep(1.0)
            self.send_and_receive_meta_data()
            response_meta_data = self.response_meta_data
        for object_name, object_attributes in response_meta_data["receive"].items():
            if object_name in self.mujoco_arm_joint_state:
                for attribute_data in object_attributes.values():
                    arm_start_joint_state[object_name] = attribute_data[0]
            elif object_name in self.mujoco_left_hand_joint_state:
                for attribute_data in object_attributes.values():
                    left_hand_start_joint_state[object_name] = attribute_data[0]
            elif object_name in self.mujoco_right_hand_joint_state:
                for attribute_data in object_attributes.values():
                    right_hand_start_joint_state[object_name] = attribute_data[0]
            elif object_name in self.mujoco_neck_joint_state:
                for attribute_data in object_attributes.values():
                    neck_start_joint_state[object_name] = attribute_data[0]

        rclpy.spin_once(self, timeout_sec=0.1)
        while any(value is None or numpy.isnan(value) for value in self.mujoco_arm_joint_state.values()) or \
              any(value is None or numpy.isnan(value) for value in self.mujoco_left_hand_joint_state.values()) or \
              any(value is None or numpy.isnan(value) for value in self.mujoco_right_hand_joint_state.values()) or \
              any(value is None or numpy.isnan(value) for value in self.mujoco_neck_joint_state.values()):
            self.loginfo("Waiting for arm, left_hand, right_hand, and neck joint states to be read...")
            self.get_logger().info(f"Arm joint state: {self.mujoco_arm_joint_state}")
            self.get_logger().info(f"Left hand joint state: {self.mujoco_left_hand_joint_state}")
            self.get_logger().info(f"Right hand joint state: {self.mujoco_right_hand_joint_state}")
            self.get_logger().info(f"Neck joint state: {self.mujoco_neck_joint_state}")
            rclpy.spin_once(self, timeout_sec=0.1)
        self.loginfo("Arm, left_hand, right_hand, and neck joint states are read.")

        num_points = 400

        self.loginfo(f"Moving left_hand from\n{self.mujoco_left_hand_joint_state} to\n{left_hand_start_joint_state}...")
        start_left_hand_joint_state_array = [self.mujoco_left_hand_joint_state[joint_name] for joint_name in self.mujoco_left_hand_joint_state.keys()]
        start_right_hand_joint_state_array = [self.mujoco_right_hand_joint_state[joint_name] for joint_name in self.mujoco_right_hand_joint_state.keys()]
        start_arm_joint_state_array = [self.mujoco_arm_joint_state[joint_name] for joint_name in self.mujoco_arm_joint_state.keys()]
        start_neck_joint_state_array = [self.mujoco_neck_joint_state[joint_name] for joint_name in self.mujoco_neck_joint_state.keys()]
        end_left_hand_joint_state_array = [left_hand_start_joint_state[joint_name] for joint_name in self.mujoco_left_hand_joint_state.keys()]
        for i in range(num_points):
            interpolated_state = [
                start + (end - start) * i / (num_points - 1)
                for start, end in zip(start_left_hand_joint_state_array, end_left_hand_joint_state_array)
            ]
            while rclpy.ok():
                start_time = time.time()
                if self.use_upper_body_publisher:
                    hand_state = start_arm_joint_state_array + interpolated_state + start_neck_joint_state_array
                    self.send_upper_body_command(hand_state)
                else:
                    self.send_mujoco_left_hand_joint_command(interpolated_state)
                rclpy.spin_once(self, timeout_sec=1.0/self.rate)
                stop_loop = True
                for joint_idx, joint_name in enumerate(self.mujoco_left_hand_joint_state.keys()):
                    current_value = self.mujoco_left_hand_joint_state[joint_name]
                    cmd_value = interpolated_state[joint_idx]
                    if not numpy.isclose(current_value, cmd_value, atol=0.1):
                        self.get_logger().warning(f"Joint {joint_name} not close: current {current_value}, cmd {cmd_value}")
                        stop_loop = False
                self.loginfo(f"Moving left_hand joint {i+1}/{num_points} with state: {interpolated_state}")
                elapsed_time = time.time() - start_time
                if elapsed_time < 1.0/self.rate:
                    time.sleep(1.0/self.rate - elapsed_time)
                if stop_loop:
                    break

        self.loginfo(f"Moving right_hand from\n{self.mujoco_right_hand_joint_state} to\n{right_hand_start_joint_state}...")
        start_left_hand_joint_state_array = [self.mujoco_left_hand_joint_state[joint_name] for joint_name in self.mujoco_left_hand_joint_state.keys()]
        start_right_hand_joint_state_array = [self.mujoco_right_hand_joint_state[joint_name] for joint_name in self.mujoco_right_hand_joint_state.keys()]
        start_arm_joint_state_array = [self.mujoco_arm_joint_state[joint_name] for joint_name in self.mujoco_arm_joint_state.keys()]
        start_neck_joint_state_array = [self.mujoco_neck_joint_state[joint_name] for joint_name in self.mujoco_neck_joint_state.keys()]
        end_right_hand_joint_state_array = [right_hand_start_joint_state[joint_name] for joint_name in self.mujoco_right_hand_joint_state.keys()]
        for i in range(num_points):
            interpolated_state = [
                start + (end - start) * i / (num_points - 1)
                for start, end in zip(start_right_hand_joint_state_array, end_right_hand_joint_state_array)
            ]
            while rclpy.ok():
                start_time = time.time()
                if self.use_upper_body_publisher:
                    hand_state = start_arm_joint_state_array + interpolated_state + start_neck_joint_state_array
                    self.send_upper_body_command(hand_state)
                else:
                    self.send_mujoco_right_hand_joint_command(interpolated_state)
                rclpy.spin_once(self, timeout_sec=1.0/self.rate)
                stop_loop = True
                for joint_idx, joint_name in enumerate(self.mujoco_right_hand_joint_state.keys()):
                    current_value = self.mujoco_right_hand_joint_state[joint_name]
                    cmd_value = interpolated_state[joint_idx]
                    if not numpy.isclose(current_value, cmd_value, atol=0.1):
                        self.get_logger().warning(f"Joint {joint_name} not close: current {current_value}, cmd {cmd_value}")
                        stop_loop = False
                self.loginfo(f"Moving right_hand joint {i+1}/{num_points} with state: {interpolated_state}")
                elapsed_time = time.time() - start_time
                if elapsed_time < 1.0/self.rate:
                    time.sleep(1.0/self.rate - elapsed_time)
                if stop_loop:
                    break

        self.loginfo(f"Moving arm from\n{self.mujoco_arm_joint_state} to\n{arm_start_joint_state}...")
        start_left_hand_joint_state_array = [self.mujoco_left_hand_joint_state[joint_name] for joint_name in self.mujoco_left_hand_joint_state.keys()]
        start_right_hand_joint_state_array = [self.mujoco_right_hand_joint_state[joint_name] for joint_name in self.mujoco_right_hand_joint_state.keys()]
        start_arm_joint_state_array = [self.mujoco_arm_joint_state[joint_name] for joint_name in self.mujoco_arm_joint_state.keys()]
        start_neck_joint_state_array = [self.mujoco_neck_joint_state[joint_name] for joint_name in self.mujoco_neck_joint_state.keys()]
        end_arm_joint_state_array = [arm_start_joint_state[joint_name] for joint_name in self.mujoco_arm_joint_state.keys()]
        for i in range(num_points):
            interpolated_state = [
                start + (end - start) * i / (num_points - 1)
                for start, end in zip(start_arm_joint_state_array, end_arm_joint_state_array)
            ]
            while rclpy.ok():
                start_time = time.time()
                if self.use_upper_body_publisher:
                    arm_state = interpolated_state + start_left_hand_joint_state_array + start_right_hand_joint_state_array + start_neck_joint_state_array
                    self.send_upper_body_command(arm_state)
                else:
                    self.send_mujoco_arm_joint_command(interpolated_state)
                rclpy.spin_once(self, timeout_sec=1.0/self.rate)
                stop_loop = True
                for joint_idx, joint_name in enumerate(self.mujoco_arm_joint_state.keys()):
                    current_value = self.mujoco_arm_joint_state[joint_name]
                    cmd_value = interpolated_state[joint_idx]
                    if not numpy.isclose(current_value, cmd_value, atol=0.05):
                        self.get_logger().warning(f"Joint {joint_name} not close: current {current_value}, cmd {cmd_value}")
                        stop_loop = False
                self.loginfo(f"Moving arm joint {i+1}/{num_points} with state: {interpolated_state}")
                elapsed_time = time.time() - start_time
                if elapsed_time < 1.0/self.rate:
                    time.sleep(1.0/self.rate - elapsed_time)
                if stop_loop:
                    break

        self.loginfo("Arm, left_hand, right_hand and neck joint states initialized.")
        self.loginfo(f"Current joint states: {self.mujoco_arm_joint_state}, {self.mujoco_left_hand_joint_state}, {self.mujoco_right_hand_joint_state}, {self.mujoco_neck_joint_state}")
        self.loginfo(f"End arm joint states: {arm_start_joint_state}, End left_hand joint states: {left_hand_start_joint_state}, End right_hand joint states: {right_hand_start_joint_state}, End neck joint states: {neck_start_joint_state}")
        self.loginfo("Starting the main loop to send and receive data.")
        self.loginfo(f"Arm indices: {arm_indices}, Left_hand indices: {left_hand_indices}, Right_hand indices: {right_hand_indices}, Neck indices: {neck_indices}")

        self.send_data = [self.sim_time]
        self.send_and_receive_data()

        try:
            self.create_timer(1.0/self.rate, lambda: self.loop(arm_indices, left_hand_indices, right_hand_indices, neck_indices))
            rclpy.spin(self)
        except KeyboardInterrupt:
            self.logwarn("KeyboardInterrupt: Exiting the program.")

    def loop(self, arm_indices, left_hand_indices, right_hand_indices, neck_indices) -> None:
        self.send_data = [self.sim_time]
        self.send_and_receive_data()
        receive_data = numpy.array(self.receive_data[1:])
        mujoco_arm_joint_state = receive_data[arm_indices].tolist()
        mujoco_left_hand_joint_state = receive_data[left_hand_indices].tolist()
        mujoco_right_hand_joint_state = receive_data[right_hand_indices].tolist()
        mujoco_neck_joint_state = receive_data[neck_indices].tolist()
        if self.use_upper_body_publisher:
            self.send_upper_body_command(mujoco_arm_joint_state + mujoco_left_hand_joint_state + mujoco_right_hand_joint_state + mujoco_neck_joint_state)
        else:
            self.send_mujoco_arm_joint_command(mujoco_arm_joint_state)
            self.send_mujoco_left_hand_joint_command(mujoco_left_hand_joint_state)
            self.send_mujoco_right_hand_joint_command(mujoco_right_hand_joint_state)
            self.send_mujoco_neck_joint_command(mujoco_neck_joint_state)
    
    def loginfo(self, message: str) -> None:
        print(f"INFO: {message}")

    def logwarn(self, message: str) -> None:
        print(f"WARN: {message}")

    def _run(self) -> None:
        self.loginfo("Start running the client.")
        self._connect_and_start()
 
    def send_and_receive_meta_data(self) -> None:
        self.loginfo("Sending request meta data: " + str(self.request_meta_data))
        self._communicate(True)
        self.loginfo("Received response meta data: " + str(self.response_meta_data))

    def send_and_receive_data(self) -> None:
        self._communicate(False)

    def send_mujoco_arm_joint_command(self, joint_positions: List[float]) -> None:
        if self.arm_publisher is None:
            self.logwarn("Arm publisher is not initialized.")
            return
        command_msg = Float64MultiArray()
        command_msg.data = joint_positions
        self.arm_publisher.publish(command_msg)

    def send_mujoco_left_hand_joint_command(self, joint_positions: List[float]) -> None:
        if self.left_hand_publisher is None:
            self.logwarn("Left hand publisher is not initialized.")
            return
        command_msg = Float64MultiArray()
        command_msg.data = joint_positions
        self.left_hand_publisher.publish(command_msg)

    def send_mujoco_right_hand_joint_command(self, joint_positions: List[float]) -> None:
        if self.right_hand_publisher is None:
            self.logwarn("Right hand publisher is not initialized.")
            return
        command_msg = Float64MultiArray()
        command_msg.data = joint_positions
        self.right_hand_publisher.publish(command_msg)
    
    def send_mujoco_neck_joint_command(self, joint_positions: List[float]) -> None:
        if self.neck_publisher is None:
            self.logwarn("Neck publisher is not initialized.")
            return
        command_msg = Float64MultiArray()
        command_msg.data = joint_positions
        self.neck_publisher.publish(command_msg)

    def send_upper_body_command(self, joint_positions: List[float]) -> None:
        if self.upper_body_publisher is None:
            self.logwarn("Upper body publisher is not initialized.")
            return
        command_msg = Float64MultiArray()
        command_msg.data = joint_positions
        self.upper_body_publisher.publish(command_msg)

def main(world_name: str, port: str, use_upper_body_publisher: bool, rate: float) -> None:
    rclpy.init()
    multiverse_meta_data = MultiverseMetaData(
        world_name=world_name,
        simulation_name="upper_body_command",
        length_unit="m",
        angle_unit="rad",
        mass_unit="kg",
        time_unit="s",
        handedness="rhs",
    )
    node = UpperBodyCommand(port=port, multiverse_meta_data=multiverse_meta_data, use_upper_body_publisher=use_upper_body_publisher, rate=rate)
    node.start()
    rclpy.shutdown()

import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=f"Command hands data from multiverse.")

    # Define arguments
    parser.add_argument("--world_name", type=str, required=False, default="world", help="Name of the world")
    parser.add_argument("--port", type=str, required=False, default="4000", help="Port number")
    parser.add_argument("--use_upper_body_publisher", action="store_true", help="Use upper body publisher")
    parser.add_argument("--rate", type=float, required=False, default=100.0, help="Rate of sending commands")

    args = parser.parse_args()
    main(args.world_name, args.port, args.use_upper_body_publisher, args.rate)
