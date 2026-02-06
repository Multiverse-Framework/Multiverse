from typing import Dict, List
from dataclasses import dataclass

from multiverse_client_py import MultiverseClient, MultiverseMetaData

import time
import numpy


@dataclass
class GripperThreshold:
    lower: float
    upper: float
    fingers_command: Dict[str, float]


class FingersCommand(MultiverseClient):
    finger_joints_command: Dict[str, float] = {
        "gripper_left_left_finger_joint_position": numpy.nan,
        "gripper_left_right_finger_joint_position": numpy.nan,
        "gripper_right_left_finger_joint_position": numpy.nan,
        "gripper_right_right_finger_joint_position": numpy.nan,
    }

    gripper_names: List[str] = [
        "left_gripper",
        "right_gripper",
    ]

    gripper_threshold: Dict[str, List[GripperThreshold]] = {
        "left_gripper": [
            GripperThreshold( # Opening position
                lower=0.0,
                upper=0.1,
                fingers_command={
                    "gripper_left_left_finger_joint_position": 0.045,
                    "gripper_left_right_finger_joint_position": 0.045,
                },
            ),
            GripperThreshold( # Closing position
                lower=0.9,
                upper=1.0,
                fingers_command={
                    "gripper_left_left_finger_joint_position": 0.0,
                    "gripper_left_right_finger_joint_position": 0.0,
                },
            )
        ],
        "right_gripper": [
            GripperThreshold(  # Opening position
                lower=0.0,
                upper=0.1,
                fingers_command={
                    "gripper_right_left_finger_joint_position": 0.045,
                    "gripper_right_right_finger_joint_position": 0.045,
                },
            ),         
            GripperThreshold(  # Closing position
                lower=0.9,
                upper=1.0,
                fingers_command={
                    "gripper_right_left_finger_joint_position": 0.0,
                    "gripper_right_right_finger_joint_position": 0.0,
                },
            )
        ],
    }

    rate: float

    def __init__(
        self,
        port: str,
        multiverse_meta_data: MultiverseMetaData,
        rate: float = 100.0
    ):
        self.rate = rate
        self._transport = "Zmq"
        self._host = "tcp://127.0.0.1"
        self._server_port = "7000"
        MultiverseClient.__init__(
            self, port=port, multiverse_meta_data=multiverse_meta_data
        )

    def start(self):
        self.run()
        self.request_meta_data["send"] = {}
        self.request_meta_data["receive"] = {}
        for finger_joint_name in self.finger_joints_command.keys():
            self.request_meta_data["send"][finger_joint_name] = ["cmd_joint_angular_position"]
        for gripper_name in self.gripper_names:
            self.request_meta_data["receive"][gripper_name] = ["scalar"]
        
        self.send_and_receive_meta_data()
        while "send" not in self.response_meta_data or "receive" not in self.response_meta_data:
            self.logwarn("Waiting for response meta data...")
            time.sleep(0.1)
        
        finger_indices = []
        gripper_indices = []
        response_data = self.response_meta_data
        for finger_joint_name in response_data["send"].keys():
            if finger_joint_name not in self.finger_joints_command:
                raise ValueError(f"Unknown finger joint name: {finger_joint_name}")
            finger_index = list(self.finger_joints_command.keys()).index(finger_joint_name)
            finger_indices.append(finger_index)
        for gripper_name in response_data["receive"].keys():
            if gripper_name not in self.gripper_names:
                raise ValueError(f"Unknown gripper name: {gripper_name}")
            gripper_index = list(self.gripper_names).index(gripper_name)
            gripper_indices.append(gripper_index)

        self.loginfo(f"Starting fingers command with finger indices: {finger_indices} and gripper indices: {gripper_indices}")

        try:
            self.loop(finger_indices=finger_indices, gripper_indices=gripper_indices)
            while True:
                time_start = time.time()
                self.loop(finger_indices=finger_indices, gripper_indices=gripper_indices)
                current_time = time.time()
                if current_time - time_start < 1.0 / self.rate:
                    time.sleep(1.0 / self.rate - (current_time - time_start))
        except KeyboardInterrupt:
            self.logwarn("KeyboardInterrupt: Exiting the program.")

    def loop(self, finger_indices: List[int], gripper_indices: List[int]) -> None:
        finger_joints_command = numpy.array([*self.finger_joints_command.values()])
        self.send_data = [self.sim_time] + finger_joints_command[finger_indices].tolist()
        self.send_and_receive_data()
        receive_data = numpy.array(self.receive_data[1:])
        gripper_states = receive_data[gripper_indices]
        for gripper_index in gripper_indices:
            gripper_name = self.gripper_names[gripper_index]
            gripper_value = gripper_states[gripper_index]
            for threshold in self.gripper_threshold[gripper_name]:
                if threshold.lower <= gripper_value <= threshold.upper:
                    self.finger_joints_command.update(threshold.fingers_command)
                    # self.loginfo(f"Gripper '{gripper_name}' state: {gripper_value}, applying command: {threshold.fingers_command}")
                    break
        
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

def main(world_name: str, port: str, rate: float) -> None:
    multiverse_meta_data = MultiverseMetaData(
        world_name=world_name,
        simulation_name="fingers_command",
        length_unit="m",
        angle_unit="rad",
        mass_unit="kg",
        time_unit="s",
        handedness="rhs",
    )
    fingers_command = FingersCommand(
        port=port,
        multiverse_meta_data=multiverse_meta_data,
        rate=rate
    )
    fingers_command.start()


import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=f"Command hands data from multiverse.")

    # Define arguments
    parser.add_argument(
        "--world_name",
        type=str,
        required=False,
        default="world",
        help="Name of the world",
    )
    parser.add_argument(
        "--port", type=str, required=False, default="4001", help="Port number"
    )
    parser.add_argument(
        "--rate",
        type=float,
        required=False,
        default=100.0,
        help="Rate at which to run the command",
    )

    args = parser.parse_args()
    main(args.world_name, args.port, args.rate)