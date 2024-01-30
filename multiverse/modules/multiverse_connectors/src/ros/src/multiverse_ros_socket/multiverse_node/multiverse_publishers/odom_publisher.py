#!/usr/bin/env python3

from typing import List, Dict

from nav_msgs.msg import Odometry

from .multiverse_publisher import Interface, INTERFACE

if INTERFACE == Interface.ROS1:
    import rospy

from .multiverse_publisher import MultiversePublisher, SocketAddress, MultiverseMetaData


class OdomPublisher(MultiversePublisher):
    _use_meta_data = False
    _msg_type = Odometry
    _body_name: str
    _frame_id: str

    def __init__(
            self,
            topic_name: str = "/tf",
            rate: float = 60.0,
            client_addr: SocketAddress = SocketAddress(),
            multiverse_meta_data: MultiverseMetaData = MultiverseMetaData(),
            **kwargs: Dict
    ) -> None:
        super().__init__(
            topic_name=topic_name,
            rate=rate,
            client_addr=client_addr,
            multiverse_meta_data=multiverse_meta_data,
        )
        if "body" not in kwargs:
            raise Exception("Body not found.")
        self._body_name = str(kwargs["body"])
        self._frame_id = str(kwargs.get("frame_id", "map"))
        self.request_meta_data["receive"][self._body_name] = [
            "position",
            "quaternion",
            "odometric_velocity",
        ]

    def _bind_response_meta_data(self, response_meta_data) -> None:
        if response_meta_data.get("receive") is None:
            return

        if INTERFACE == Interface.ROS1:
            self._msg.header.seq += 1
        self._msg.header.frame_id = self._frame_id
        self._msg.child_frame_id = self._body_name
        self._msg.pose.covariance = [0.0] * 36
        self._msg.twist.covariance = [0.0] * 36

    def _bind_receive_data(self, receive_data: List[float]) -> None:
        if len (receive_data) != 14:
            return
        if INTERFACE == Interface.ROS1:
            self._msg.header.stamp = rospy.Time.now()
        elif INTERFACE == Interface.ROS2:
            self._msg.header.stamp = self.get_clock().now().to_msg()
        self._msg.pose.pose.position.x = receive_data[1]
        self._msg.pose.pose.position.y = receive_data[2]
        self._msg.pose.pose.position.z = receive_data[3]
        self._msg.pose.pose.orientation.w = receive_data[4]
        self._msg.pose.pose.orientation.x = receive_data[5]
        self._msg.pose.pose.orientation.y = receive_data[6]
        self._msg.pose.pose.orientation.z = receive_data[7]
        self._msg.twist.twist.linear.x = receive_data[8]
        self._msg.twist.twist.linear.y = receive_data[9]
        self._msg.twist.twist.linear.z = receive_data[10]
        self._msg.twist.twist.angular.x = receive_data[11]
        self._msg.twist.twist.angular.y = receive_data[12]
        self._msg.twist.twist.angular.z = receive_data[13]
