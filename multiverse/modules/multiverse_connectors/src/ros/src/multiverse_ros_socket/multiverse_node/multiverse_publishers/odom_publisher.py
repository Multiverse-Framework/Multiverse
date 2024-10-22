#!/usr/bin/env python3

from typing import List, Dict

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

from .multiverse_publisher import Interface, INTERFACE

if INTERFACE == Interface.ROS1:
    import rospy

from .multiverse_publisher import MultiversePublisher, SocketAddress, MultiverseMetaData


class OdomPublisher(MultiversePublisher):
    _use_meta_data = False
    _msg_types = [Odometry, TFMessage]
    _body_name: str
    _frame_id: str

    def __init__(
            self,
            client_addr: SocketAddress,
            rate: float = 60.0,
            multiverse_meta_data: MultiverseMetaData = MultiverseMetaData(),
            **kwargs: Dict
    ) -> None:
        if "body" not in kwargs:
            raise Exception("body not found.")
        if "odom_topic" not in kwargs:
            raise Exception("odom_topic not found.")
        if "tf_topic" not in kwargs:
            raise Exception("tf_topic not found.")
        self._body_name = str(kwargs["body"])
        self._frame_id = str(kwargs.get("frame_id", "map"))
        super().__init__(
            client_addr=client_addr,
            topic_name=[kwargs["odom_topic"], kwargs["tf_topic"]],
            rate=rate,
            multiverse_meta_data=multiverse_meta_data,
        )

        def bind_request_meta_data() -> None:
            self.request_meta_data["receive"][self._body_name] = [
                "position",
                "quaternion",
                "odometric_velocity",
            ]
        self.bind_request_meta_data_callback = bind_request_meta_data

        def bind_response_meta_data() -> None:
            response_meta_data = self.response_meta_data
            if response_meta_data.get("receive") is None:
                return

            if INTERFACE == Interface.ROS1:
                self._msgs[0].header.seq = 0
            self._msgs[0].header.frame_id = self._frame_id
            self._msgs[0].child_frame_id = self._body_name
            self._msgs[0].pose.covariance = [0.0] * 36
            self._msgs[0].twist.covariance = [0.0] * 36

            self._msgs[1].transforms.clear()
            tf_msg = TransformStamped()
            tf_msg.header.frame_id = self._frame_id
            tf_msg.child_frame_id = self._body_name
            self._msgs[1].transforms.append(tf_msg)
        self.bind_response_meta_data_callback = bind_response_meta_data

        def bind_send_data() -> None:
            self.send_data = [self.world_time + self.sim_time]
        self.bind_send_data_callback = bind_send_data

        def bind_receive_data() -> None:
            receive_data = self.receive_data
            if len(receive_data) != 14:
                return
            if INTERFACE == Interface.ROS1:
                self._msgs[0].header.stamp = rospy.Time.now()
            elif INTERFACE == Interface.ROS2:
                self._msgs[0].header.stamp = self.get_clock().now().to_msg()
            self._msgs[0].pose.pose.position.x = receive_data[1]
            self._msgs[0].pose.pose.position.y = receive_data[2]
            self._msgs[0].pose.pose.position.z = receive_data[3]
            self._msgs[0].pose.pose.orientation.w = receive_data[4]
            self._msgs[0].pose.pose.orientation.x = receive_data[5]
            self._msgs[0].pose.pose.orientation.y = receive_data[6]
            self._msgs[0].pose.pose.orientation.z = receive_data[7]
            self._msgs[0].twist.twist.linear.x = receive_data[8]
            self._msgs[0].twist.twist.linear.y = receive_data[9]
            self._msgs[0].twist.twist.linear.z = receive_data[10]
            self._msgs[0].twist.twist.angular.x = receive_data[11]
            self._msgs[0].twist.twist.angular.y = receive_data[12]
            self._msgs[0].twist.twist.angular.z = receive_data[13]

            if INTERFACE == Interface.ROS1:
                self._msgs[1].transforms[0].header.stamp = rospy.Time.now()
                self._msgs[1].transforms[0].header.seq += 1
            elif INTERFACE == Interface.ROS2:
                self._msgs[1].transforms[0].header.stamp = self.get_clock().now().to_msg()

            self._msgs[1].transforms[0].transform.translation.x = receive_data[1]
            self._msgs[1].transforms[0].transform.translation.y = receive_data[2]
            self._msgs[1].transforms[0].transform.translation.z = receive_data[3]
            self._msgs[1].transforms[0].transform.rotation.w = receive_data[4]
            self._msgs[1].transforms[0].transform.rotation.x = receive_data[5]
            self._msgs[1].transforms[0].transform.rotation.y = receive_data[6]
            self._msgs[1].transforms[0].transform.rotation.z = receive_data[7]
        self.bind_receive_data_callback = bind_receive_data