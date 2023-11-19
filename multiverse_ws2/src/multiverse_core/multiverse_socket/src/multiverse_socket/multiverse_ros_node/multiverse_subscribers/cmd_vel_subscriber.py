#!/usr/bin/env python3

from geometry_msgs.msg import Twist

from .ros_subscriber import MultiverseRosSubscriber
from ..multiverse_ros_node import SimulationMetaData


class CmdVelSubscriber(MultiverseRosSubscriber):
    def __init__(
        self,
        topic_name: str = "/cmd_vel",
        node_name: str = "cmd_vel_subscriber",
        client_host: str = "tcp://127.0.0.1",
        client_port: str = "",
        simulation_metadata: SimulationMetaData = SimulationMetaData(),
        **kwargs
    ) -> None:
        self._body = kwargs.get("body")
        if self._body is None:
            raise Exception("Body not found.")
        elif not isinstance(self._body, str):
            raise TypeError("Body is not a string.")
        self._msg_type = Twist
        super().__init__(
            topic_name=topic_name,
            node_name=node_name,
            client_host=client_host,
            client_port=client_port,
            simulation_metadata=simulation_metadata,
        )

    def _init_request_meta_data(self) -> None:
        super()._init_request_meta_data()
        self._request_meta_data_dict["send"] = {}
        self._request_meta_data_dict["send"][self._body] = ["odometric_velocity"]
        self._request_meta_data_dict["receive"] = {}

    def _init_send_data(self) -> None:
        self._send_data = [0.0] * 7

    def _bind_send_data(self, twist_msg: Twist) -> None:
        self._send_data[1] = twist_msg.linear.x
        self._send_data[2] = twist_msg.linear.y
        self._send_data[3] = twist_msg.linear.z
        self._send_data[4] = twist_msg.angular.x
        self._send_data[5] = twist_msg.angular.y
        self._send_data[6] = twist_msg.angular.z
