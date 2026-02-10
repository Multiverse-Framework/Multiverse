#!/usr/bin/env python3

from typing import Dict

from std_msgs.msg import Float64MultiArray

from .multiverse_subscriber import MultiverseSubscriber, MultiverseMetaData


class JointPositionCommandSubscriber(MultiverseSubscriber):
    _use_meta_data = False
    _joint_types: str
    _msg_type = Float64MultiArray

    def __init__(
        self,
        port: str,
        topic_name: str = "/commands",
        multiverse_meta_data: MultiverseMetaData = MultiverseMetaData(),
        **kwargs: Dict
    ) -> None:
        self._joint_types = kwargs.get("joint_types", {})
        if not isinstance(self._joint_types, dict):
            raise ValueError("joint_types must be a dict.")
        self._joint_id_dict = {}
        super().__init__(port=port, 
                         topic_name=topic_name, 
                         multiverse_meta_data=multiverse_meta_data)
        
        def bind_request_meta_data() -> None:
            self.request_meta_data["send"] = {}
            for joint_name, joint_type in self._joint_types.items():
                if joint_type == "revolute":
                    self.request_meta_data["send"][joint_name] = [
                        "cmd_joint_angular_position",
                    ]
                elif joint_type == "prismatic":
                    self.request_meta_data["send"][joint_name] = [
                        "cmd_joint_linear_position",
                    ]
        
        self.bind_request_meta_data_callback = bind_request_meta_data

        def bind_response_meta_data() -> None:
            response_meta_data = self.response_meta_data
            if response_meta_data.get("send") is None:
                return
            idx = 1
            for joint_name in self.response_meta_data["send"].keys():
                self._joint_id_dict[joint_name] = {}
                for attribute_name, attribute_data in self.response_meta_data["send"][joint_name].items():
                    assert len(attribute_data) == 1
                    self._joint_id_dict[joint_name][attribute_name] = idx
                    idx += len(attribute_data)

        self.bind_response_meta_data_callback = bind_response_meta_data

    def _bind_send_data(self, joint_command_msg: Float64MultiArray) -> None:
        if len(joint_command_msg.data) != len(self._joint_types):
            self.logwarn(
                f"Received joint command of size {len(joint_command_msg.data)}, "
                f"but expected size {len(self._joint_types)}."
            )

        send_data = [self.sim_time] + [0.0] * len(self._joint_id_dict)
        for i, (joint_name, joint_type) in enumerate(self._joint_types.items()):
            if joint_type == "revolute":
                send_data[self._joint_id_dict[joint_name]["cmd_joint_angular_position"]] = joint_command_msg.data[i]
            elif joint_type == "prismatic":
                send_data[self._joint_id_dict[joint_name]["cmd_joint_linear_position"]] = joint_command_msg.data[i]
        self.send_data = send_data
