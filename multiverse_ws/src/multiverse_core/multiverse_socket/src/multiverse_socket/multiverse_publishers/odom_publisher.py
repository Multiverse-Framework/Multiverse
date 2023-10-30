#!/usr/bin/env python3

import rospy
from typing import List
from nav_msgs.msg import Odometry
from multiverse_socket.multiverse_publishers import MultiverseRosPublisher


class odom_publisher(MultiverseRosPublisher):
    def __init__(self, **kwargs) -> None:
        self.__body_name = kwargs.get("body")
        if self.__body_name is None:
            raise Exception("Body not found.")
        elif not isinstance(self.__body_name, str):
            raise TypeError("Body is not a string.")
        super().__init__(**kwargs)
        self.__odom_publisher = rospy.Publisher("odom", Odometry, queue_size=0)
        self.__odom_msg = Odometry()

    def _init_request_meta_data(self) -> None:
        super()._init_request_meta_data()
        self._request_meta_data_dict["receive"][self.__body_name] = ["position", "quaternion", "relative_velocity"]

    def _construct_rosmsg(self, response_meta_data_dict) -> None:
        if response_meta_data_dict.get("receive") is None:
            return
        self.__odom_msg.header.frame_id = rospy.get_param("/multiverse_client/ros/root_frame_id") if rospy.has_param("/multiverse_client/ros/root_frame_id") else "map"
        self.__odom_msg.child_frame_id = self.__body_name
        self.__odom_msg.pose.covariance = [0.0] * 36
        self.__odom_msg.twist.covariance = [0.0] * 36

    def _bind_rosmsg(self, receive_data: List[float]) -> None:
        self.__odom_msg.header.seq += 1
        self.__odom_msg.header.stamp = rospy.Time.from_sec(receive_data[0])
        self.__odom_msg.pose.pose.position.x = receive_data[1]
        self.__odom_msg.pose.pose.position.y = receive_data[2]
        self.__odom_msg.pose.pose.position.z = receive_data[3]
        self.__odom_msg.pose.pose.orientation.w = receive_data[4]
        self.__odom_msg.pose.pose.orientation.x = receive_data[5]
        self.__odom_msg.pose.pose.orientation.y = receive_data[6]
        self.__odom_msg.pose.pose.orientation.z = receive_data[7]
        self.__odom_msg.twist.twist.linear.x = receive_data[8]
        self.__odom_msg.twist.twist.linear.y = receive_data[9]
        self.__odom_msg.twist.twist.linear.z = receive_data[10]
        self.__odom_msg.twist.twist.angular.x = receive_data[11]
        self.__odom_msg.twist.twist.angular.y = receive_data[12]
        self.__odom_msg.twist.twist.angular.z = receive_data[13]

    def _publish(self) -> None:
        self.__odom_publisher.publish(self.__odom_msg)
