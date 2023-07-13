#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from .ros_subscriber import MultiverseRosSubscriber
from math import sin, cos


class cmd_vel_subscriber(MultiverseRosSubscriber):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
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
        self._send_meta_data_dict["receive"][self.__body] = ["quaternion"]

    def _init_send_data(self) -> None:
        self._send_data = [0.0 for _ in range(7)]

    def _bind_send_data(self, twist_msg: Twist):
        (ang_x_pos, ang_y_pos, ang_z_pos) = euler_from_quaternion(
            [self._receive_data[2], self._receive_data[3], self._receive_data[4], self._receive_data[1]]
        )
        self._send_data[0] = rospy.Time.now().to_sec()
        self._send_data[1] = (
            twist_msg.linear.x * cos(ang_y_pos) * cos(ang_z_pos)
            + twist_msg.linear.y * (sin(ang_x_pos) * sin(ang_y_pos) * cos(ang_z_pos) - cos(ang_x_pos) * sin(ang_z_pos))
            + twist_msg.linear.z * (cos(ang_x_pos) * sin(ang_y_pos) * cos(ang_z_pos) + sin(ang_x_pos) * sin(ang_z_pos))
        )
        self._send_data[2] = (
            twist_msg.linear.x * cos(ang_y_pos) * sin(ang_z_pos)
            + twist_msg.linear.y * (sin(ang_x_pos) * sin(ang_y_pos) * sin(ang_z_pos) + cos(ang_x_pos) * cos(ang_z_pos))
            + twist_msg.linear.z * (cos(ang_x_pos) * sin(ang_y_pos) * sin(ang_z_pos) - sin(ang_x_pos) * cos(ang_z_pos))
        )
        self._send_data[3] = (
            -twist_msg.linear.x * sin(ang_y_pos)
            + twist_msg.linear.y * sin(ang_x_pos) * cos(ang_y_pos)
            + twist_msg.linear.z * cos(ang_x_pos) * cos(ang_y_pos)
        )
        self._send_data[4] = twist_msg.angular.x
        self._send_data[5] = twist_msg.angular.y
        self._send_data[6] = twist_msg.angular.z
