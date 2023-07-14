#!/usr/bin/env python3

from geometry_msgs.msg import Twist
from .ros_subscriber import MultiverseRosSubscriber


class cmd_vel_subscriber(MultiverseRosSubscriber):
    def __init__(self, **kwargs) -> None:
        self.__body = kwargs.get("body")
        if self.__body is None:
            raise Exception("Body not found.")
        elif not isinstance(self.__body, str):
            raise TypeError("Body is not a string.")
        super().__init__(**kwargs)
        self._topic_name = "/cmd_vel"
        self._data_class = Twist

    def _init_send_meta_data(self):
        super()._init_send_meta_data()
        self._send_meta_data_dict["send"] = {}
        self._send_meta_data_dict["send"][self.__body] = ["relative_velocity"]
        self._send_meta_data_dict["receive"] = {}

    def _init_send_data(self) -> None:
        self._send_data = [0.0] * 7

    def _bind_send_data(self, twist_msg: Twist):
        self._send_data[1] = twist_msg.linear.x
        self._send_data[2] = twist_msg.linear.y
        self._send_data[3] = twist_msg.linear.z
        self._send_data[4] = twist_msg.angular.x
        self._send_data[5] = twist_msg.angular.y
        self._send_data[6] = twist_msg.angular.z
