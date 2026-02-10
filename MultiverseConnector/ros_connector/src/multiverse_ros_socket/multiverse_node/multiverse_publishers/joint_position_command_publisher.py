#!/usr/bin/env python3

from typing import Dict

from std_msgs.msg import Float64MultiArray

from .multiverse_publisher import MultiversePublisher, MultiverseMetaData


class JointPositionCommandPublisher(MultiversePublisher):
    _use_meta_data = False
    _msg_types = [Float64MultiArray]
    _joint_types: str

    def __init__(
        self,
        port: str,
        topic_name: str = "/commands",
        rate: float = 60.0,
        multiverse_meta_data: MultiverseMetaData = MultiverseMetaData(),
        **kwargs: Dict
    ) -> None:
        super().__init__(
            port=port,
            topic_name=topic_name,
            rate=rate,
            multiverse_meta_data=multiverse_meta_data,
        )
        self._joint_types = kwargs.get("joint_types", {})
        if not isinstance(self._joint_types, dict):
            raise ValueError("joint_types must be a dict.")
        self._joint_id_dict = {}
        
        def bind_request_meta_data() -> None:
            self.request_meta_data["receive"] = {}
            for joint_name, joint_type in self._joint_types.items():
                if joint_type == "revolute":
                    self.request_meta_data["receive"][joint_name] = [
                        "cmd_joint_angular_position",
                    ]
                elif joint_type == "prismatic":
                    self.request_meta_data["receive"][joint_name] = [
                        "cmd_joint_linear_position",
                    ]
        self.bind_request_meta_data_callback = bind_request_meta_data

        def bind_response_meta_data() -> None:
            response_meta_data = self.response_meta_data
            if response_meta_data.get("receive") is None:
                return
            idx = 0
            for joint_name in self.response_meta_data["receive"].keys():
                self._joint_id_dict[joint_name] = {}
                for attribute_name, attribute_data in self.response_meta_data["receive"][joint_name].items():
                    assert len(attribute_data) == 1
                    self._joint_id_dict[joint_name][attribute_name] = idx
                    idx += len(attribute_data)
            self._msgs[0].data = [0.0] * len(self._joint_id_dict)
        self.bind_response_meta_data_callback = bind_response_meta_data

        def bind_send_data() -> None:
            self.send_data = [self.sim_time]
        self.bind_send_data_callback = bind_send_data

        def bind_receive_data() -> None:
            receive_data = self.receive_data[1:]
            if len(receive_data) != len(self._joint_types):
                self.logwarn(
                    f"Received joint command of size {len(receive_data)}, "
                    f"but expected size {len(self._joint_types)}."
                )

            for i, (joint_name, joint_type) in enumerate(self._joint_types.items()):
                if joint_type == "revolute":
                    self._msgs[0].data[i] = receive_data[self._joint_id_dict[joint_name]["cmd_joint_angular_position"]]
                elif joint_type == "prismatic":
                    self._msgs[0].data[i] = receive_data[self._joint_id_dict[joint_name]["cmd_joint_linear_position"]]
        self.bind_receive_data_callback = bind_receive_data
