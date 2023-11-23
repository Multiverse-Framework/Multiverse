#!/usr/bin/env python3

from typing import Dict

from geometry_msgs.msg import Twist

from .multiverse_subscriber import MultiverseSubscriber
from ..multiverse_meta_node import SocketAddress, MultiverseMetaData


class CmdVelSubscriber(MultiverseSubscriber):
    _body_name: str
    _msg_type = Twist

    def __init__(
            self,
            node_name: str,
            topic_name: str = "/cmd_vel",
            client_addr: SocketAddress = SocketAddress(),
            multiverse_meta_data: MultiverseMetaData = MultiverseMetaData(),
            **kwargs: Dict
    ) -> None:
        if "body" not in kwargs:
            raise Exception("Body not found.")
        self._body_name = str(kwargs["body"])
        super().__init__(
            node_name=node_name,
            topic_name=topic_name,
            client_addr=client_addr,
            multiverse_meta_data=multiverse_meta_data
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
            twist_msg.angular.z,
        ]
        return twist_msg
