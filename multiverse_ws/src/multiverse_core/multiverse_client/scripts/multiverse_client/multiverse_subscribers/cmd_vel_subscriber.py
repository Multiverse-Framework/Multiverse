#!/usr/bin/env python3

import rospy
from typing import Any, List
from geometry_msgs.msg import Twist
from .ros_subscriber import MultiverseRosSubscriber


class cmd_vel_subscriber(MultiverseRosSubscriber):
    def __init__(self, host: str, port: str, **kwargs) -> None:
        super().__init__(host, port)
        self.__body = kwargs.get("body")
        if self.__body is None:
            raise Exception("Body not found.")
        elif not isinstance(self.__body, str):
            raise TypeError("Body is not a string.")
        self._topic_name = "/cmd_vel"
        self._data_class = Twist

    def _construct_send_meta_data(self):
        self._send_meta_data_dict["send"] = {}
        self._send_meta_data_dict["send"][self.__body] = ["relative_velocity"]
        self._send_meta_data_dict["receive"] = {}

    def _init_send_data(self) -> None:
        self._data = [0.0 for _ in range(7)]

    def _bind_send_data(self, twist_msg: Twist) -> List[float]:
        self._data[0] = rospy.Time.now().to_sec()
        self._data[1] = twist_msg.linear.x
        self._data[2] = twist_msg.linear.y
        self._data[3] = twist_msg.linear.z
        self._data[4] = twist_msg.angular.x
        self._data[5] = twist_msg.angular.y
        self._data[6] = twist_msg.angular.z
