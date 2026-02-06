#!/usr/bin/env python3

from typing import Dict, List

from sensor_msgs.msg import JointState

from .multiverse_publisher import Interface, INTERFACE

if INTERFACE == Interface.ROS1:
    import rospy

from .multiverse_publisher import MultiversePublisher, MultiverseMetaData


class JointStatePublisher(MultiversePublisher):
    _use_meta_data = False
    _msg_types = [JointState]
    _frame_id: str
    _revolute_joint_names: List[str] = []
    _prismatic_joint_names: List[str] = []
    _joint_states_indices: Dict[str, Dict[str, int]] = {}

    def __init__(
            self,
            port: str,
            topic_name: str = "/joint_states",
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
        self._frame_id = kwargs.get("frame_id", "map")
        self._revolute_joint_names = kwargs["revolute_joint_names"] if "revolute_joint_names" in kwargs else []
        self._prismatic_joint_names = kwargs["prismatic_joint_names"] if "prismatic_joint_names" in kwargs else []
        self._joint_states_indices = {}
        self._msgs[0].header.frame_id = self._frame_id
        if INTERFACE == Interface.ROS1:
            self._msgs[0].header.stamp = rospy.Time.now()
            self._msgs[0].header.seq = 0
        elif INTERFACE == Interface.ROS2:
            self._msgs[0].header.stamp = self.get_clock().now().to_msg()

        def bind_request_meta_data() -> None:
            if len(self._revolute_joint_names) > 0:
                for joint_name in self._revolute_joint_names:
                    self.request_meta_data["receive"][joint_name] = [
                        "joint_angular_position",
                        "joint_angular_velocity",
                        "joint_torque",
                    ]
            if len(self._prismatic_joint_names) > 0:
                for joint_name in self._prismatic_joint_names:
                    self.request_meta_data["receive"][joint_name] = [
                        "joint_linear_position",
                        "joint_linear_velocity",
                        "joint_force",
                    ]
            if len(self._revolute_joint_names) == 0 and len(self._prismatic_joint_names) == 0:
                self.request_meta_data["receive"][""] = [
                    "joint_angular_position",
                    "joint_linear_position",
                    "joint_angular_velocity",
                    "joint_linear_velocity",
                    "joint_force",
                    "joint_torque",
                ]
        self.bind_request_meta_data_callback = bind_request_meta_data

        def bind_response_meta_data() -> None:
            response_meta_data = self.response_meta_data
            if response_meta_data.get("receive") is None:
                return

            idx = 1
            for joint_name, joint_data in response_meta_data["receive"].items():
                self._msgs[0].name.append(joint_name)
                self._msgs[0].position.append(0.0)
                self._msgs[0].velocity.append(0.0)
                self._msgs[0].effort.append(0.0)
                self._joint_states_indices[joint_name] = {"position": -1, "velocity": -1, "effort": -1}
                if "joint_angular_position" in joint_data or "joint_linear_position" in joint_data:
                    self._joint_states_indices[joint_name]["position"] = idx
                    idx += 1
                if "joint_angular_velocity" in joint_data or "joint_linear_velocity" in joint_data:
                    self._joint_states_indices[joint_name]["velocity"] = idx
                    idx += 1
                if "joint_force" in joint_data or "joint_torque" in joint_data:
                    self._joint_states_indices[joint_name]["effort"] = idx
                    idx += 1
        self.bind_response_meta_data_callback = bind_response_meta_data

        def bind_send_data() -> None:
            self.send_data = [self.sim_time]
        self.bind_send_data_callback = bind_send_data

        def bind_receive_data() -> None:
            receive_data = self.receive_data
            if INTERFACE == Interface.ROS1:
                self._msgs[0].header.stamp = rospy.Time.now()
                self._msgs[0].header.seq += 1
            elif INTERFACE == Interface.ROS2:
                self._msgs[0].header.stamp = self.get_clock().now().to_msg()
            for joint_idx, joint_state_indices in enumerate(self._joint_states_indices.values()):
                joint_position_index = joint_state_indices["position"]
                joint_velocity_index = joint_state_indices["velocity"]
                joint_effort_index = joint_state_indices["effort"]
                if joint_position_index != -1:
                    self._msgs[0].position[joint_idx] = receive_data[joint_position_index]
                if joint_velocity_index != -1:
                    self._msgs[0].velocity[joint_idx] = receive_data[joint_velocity_index]
                if joint_effort_index != -1:
                    self._msgs[0].effort[joint_idx] = receive_data[joint_effort_index]
        self.bind_receive_data_callback = bind_receive_data
