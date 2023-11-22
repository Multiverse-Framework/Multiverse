#!/usr/bin/env python3

from typing import Dict

from geometry_msgs.msg import Twist

from .ros_subscriber import MultiverseRosSubscriber
from ..multiverse_ros_node import SimulationMetaData


class CmdVelSubscriber(MultiverseRosSubscriber):
    _body_name: str
    _msg_type = Twist

    def __init__(
            self,
            topic_name: str = "/cmd_vel",
            client_host: str = "tcp://127.0.0.1",
            client_port: str = "",
            simulation_metadata: SimulationMetaData = SimulationMetaData(),
            **kwargs: Dict
    ) -> None:
        self._body_name = None if "body" not in kwargs else str(kwargs["body"])
        if self._body_name is None:
            raise Exception("Body not found.")
        super().__init__(
            topic_name=topic_name,
            client_host=client_host,
            client_port=client_port,
            simulation_metadata=simulation_metadata,
        )
        self.request_meta_data["send"][self._body_name] = ["odometric_velocity"]

    def _init_send_data(self) -> None:
        self.send_data = [0.0] * 7

    def _bind_send_data(self, twist_msg: Twist) -> Twist:
        self.send_data[1:] = [
            twist_msg.linear.x,
            twist_msg.linear.y,
            twist_msg.linear.z,
            twist_msg.angular.x,
            twist_msg.angular.y,
            twist_msg.angular.z
        ]
        return twist_msg
