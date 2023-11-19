#!/usr/bin/env python3

from typing import List

from nav_msgs.msg import Odometry

from .ros_publisher import MultiverseRosPublisher
from ..multiverse_ros_node import SimulationMetaData


class OdomPublisher(MultiverseRosPublisher):
    _use_meta_data = False
    _msg_type = Odometry
    _odom_msg = Odometry()
    _body_name: str
    _frame_id: str

    def __init__(
            self,
            topic_name: str = "/tf",
            node_name: str = "tf_publisher",
            rate: float = 60.0,
            client_host: str = "tcp://127.0.0.1",
            client_port: str = "",
            simulation_metadata: SimulationMetaData = SimulationMetaData(),
            **kwargs
    ) -> None:
        super().__init__(topic_name=topic_name, node_name=node_name, rate=rate, client_host=client_host,
                         client_port=client_port, simulation_metadata=simulation_metadata)
        self._body_name = kwargs.get("body")
        if self._body_name is None:
            raise Exception("Body not found.")
        elif not isinstance(self._body_name, str):
            raise TypeError("Body is not a string.")
        self._frame_id = str(kwargs.get("frame_id", "map"))
        self.request_meta_data["receive"][self._body_name] = ["position", "quaternion", "odometric_velocity"]


    def _construct_ros_message(self, response_meta_data) -> None:
        if response_meta_data.get("receive") is None:
            return
        
        self._odom_msg.header.frame_id = self._frame_id
        self._odom_msg.child_frame_id = self._body_name
        self._odom_msg.pose.covariance = [0.0] * 36
        self._odom_msg.twist.covariance = [0.0] * 36

    def _bind_ros_message(self, receive_data: List[float]) -> None:
        self._odom_msg.header.stamp = self.get_clock().now().to_msg()
        self._odom_msg.pose.pose.position.x = receive_data[1]
        self._odom_msg.pose.pose.position.y = receive_data[2]
        self._odom_msg.pose.pose.position.z = receive_data[3]
        self._odom_msg.pose.pose.orientation.w = receive_data[4]
        self._odom_msg.pose.pose.orientation.x = receive_data[5]
        self._odom_msg.pose.pose.orientation.y = receive_data[6]
        self._odom_msg.pose.pose.orientation.z = receive_data[7]
        self._odom_msg.twist.twist.linear.x = receive_data[8]
        self._odom_msg.twist.twist.linear.y = receive_data[9]
        self._odom_msg.twist.twist.linear.z = receive_data[10]
        self._odom_msg.twist.twist.angular.x = receive_data[11]
        self._odom_msg.twist.twist.angular.y = receive_data[12]
        self._odom_msg.twist.twist.angular.z = receive_data[13]

    def _publish(self) -> None:
        self._publisher.publish(self._odom_msg)
