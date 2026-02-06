#!/usr/bin/env python3

from typing import Dict

from geometry_msgs.msg import Twist

from .multiverse_subscriber import MultiverseSubscriber, MultiverseMetaData


class CmdVelSubscriber(MultiverseSubscriber):
    _use_meta_data = False
    _body_name: str
    _msg_type = Twist

    def __init__(
            self,
            port: str,
            topic_name: str = "/cmd_vel",
            multiverse_meta_data: MultiverseMetaData = MultiverseMetaData(),
            **kwargs: Dict
    ) -> None:
        if "body" not in kwargs:
            raise Exception("Body not found.")
        self._body_name = str(kwargs["body"])
        super().__init__(
            port=port,
            topic_name=topic_name,
            multiverse_meta_data=multiverse_meta_data
        )

        def bind_request_meta_data() -> None:
            self.request_meta_data["send"][self._body_name] = ["odometric_velocity"]

        self.bind_request_meta_data_callback = bind_request_meta_data

    def _init_send_data(self) -> None:
        self.send_data = [self.sim_time] + [0.0] * 6

    def _bind_send_data(self, twist_msg: Twist) -> None:
        self.send_data = [
            self.sim_time,
            twist_msg.linear.x,
            twist_msg.linear.y,
            twist_msg.linear.z,
            twist_msg.angular.x,
            twist_msg.angular.y,
            twist_msg.angular.z,
        ]
