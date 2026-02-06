#!/usr/bin/env python3

from typing import Dict

from sensor_msgs.msg import JointState

from .multiverse_subscriber import MultiverseSubscriber, MultiverseMetaData


class JointStateSubscriber(MultiverseSubscriber):
    _use_meta_data = True
    _joint_types: str
    _msg_type = JointState

    def __init__(
        self,
        port: str,
        topic_name: str = "/joint_states",
        multiverse_meta_data: MultiverseMetaData = MultiverseMetaData(),
        **kwargs: Dict
    ) -> None:
        self._joint_types = kwargs.get("joint_types", "revolute")
        if not isinstance(self._joint_types, dict) and not isinstance(self._joint_types, str):
            raise ValueError("joint_types must be a dict or str.")
        super().__init__(port=port, 
                         topic_name=topic_name, 
                         multiverse_meta_data=multiverse_meta_data)

    def _bind_send_data(self, joint_state_msg: JointState) -> None:
        if "send" not in self.response_meta_data or self.response_meta_data["send"] == {}:
            self.__bind_request_meta_data(joint_state_msg)
            self._communicate(True)

        joint_id_dict = {}
        idx = 0
        for joint_name in self.response_meta_data["send"].keys():
            if isinstance(self._joint_types, dict):
                if joint_name not in self._joint_types:
                    continue
                joint_type = self._joint_types[joint_name]
            else:
                joint_type = self._joint_types
            joint_id_dict[joint_name] = {}
            for attribute_name, attribute_data in self.response_meta_data["send"][joint_name].items():
                assert len(attribute_data) == 1
                joint_id_dict[joint_name][attribute_name] = idx
                idx += len(attribute_data)

        send_data = [0.0] * idx

        for i in range(len(joint_state_msg.name)):
            joint_name = joint_state_msg.name[i]
            if joint_name not in self._joint_types:
                continue
            joint_type = self._joint_types[joint_name]
            if joint_type not in ["revolute", "prismatic"]:
                continue
            joint_position = joint_state_msg.position[i]
            joint_velocity = joint_state_msg.velocity[i]
            joint_effort = joint_state_msg.effort[i]
            if joint_type == "revolute":
                send_data[joint_id_dict[joint_name]["joint_angular_position"]] = joint_position
                send_data[joint_id_dict[joint_name]["joint_angular_velocity"]] = joint_velocity
                send_data[joint_id_dict[joint_name]["joint_torque"]] = joint_effort
            elif joint_type == "prismatic":
                send_data[joint_id_dict[joint_name]["joint_linear_position"]] = joint_position
                send_data[joint_id_dict[joint_name]["joint_linear_velocity"]] = joint_velocity
                send_data[joint_id_dict[joint_name]["joint_force"]] = joint_effort

        self.send_data = [self.sim_time] + send_data

    def __bind_request_meta_data(self, joint_state_msg: JointState) -> None:
        self.request_meta_data["send"] = {}
        for joint_name in joint_state_msg.name:
            if isinstance(self._joint_types, dict):
                if joint_name not in self._joint_types:
                    continue
                joint_type = self._joint_types[joint_name]
            else:
                joint_type = self._joint_types
            if joint_type not in ["revolute", "prismatic"]:
                continue
            if joint_type == "revolute":
                self.request_meta_data["send"][joint_name] = [
                    "joint_angular_position",
                    "joint_angular_velocity",
                    "joint_torque",
                ]
            elif joint_type == "prismatic":
                self.request_meta_data["send"][joint_name] = [
                    "joint_linear_position",
                    "joint_linear_velocity",
                    "joint_force",
                ]