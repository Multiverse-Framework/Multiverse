#!/usr/bin/env python3

from typing import Dict

from sensor_msgs.msg import Imu

from .multiverse_publisher import Interface, INTERFACE

if INTERFACE == Interface.ROS1:
    import rospy

from .multiverse_publisher import MultiversePublisher, MultiverseMetaData


class ImuPublisher(MultiversePublisher):
    _use_meta_data = False
    _msg_types = [Imu]
    _seq: int = 0

    def __init__(
            self,
            port: str,
            topic_name: str = "/imu",
            rate: float = 60.0,
            multiverse_meta_data: MultiverseMetaData = MultiverseMetaData(),
            **kwargs: Dict
    ) -> None:
        super().__init__(
            port=port,
            topic_name=topic_name,
            rate=rate,
            multiverse_meta_data=multiverse_meta_data,
        )
        assert "body" in kwargs, "Body not found."
        self._body_name = str(kwargs["body"])
        self._msgs[0].header.frame_id = self._body_name
        if INTERFACE == Interface.ROS1:
            self._msgs[0].header.stamp = rospy.Time.now()
            self._msgs[0].header.seq = 0
        elif INTERFACE == Interface.ROS2:
            self._msgs[0].header.stamp = self.get_clock().now().to_msg()

        def bind_request_meta_data() -> None:
            self.request_meta_data["receive"][self._body_name] = [
                "quaternion",
                "angular_velocity",
                "linear_acceleration",
            ]
        self.bind_request_meta_data_callback = bind_request_meta_data

        def bind_response_meta_data() -> None:
            response_meta_data = self.response_meta_data
            if response_meta_data.get("receive") is None:
                return
            self._msgs[0].orientation_covariance[0] = 1e-9
            self._msgs[0].orientation_covariance[4] = 1e-9
            self._msgs[0].orientation_covariance[8] = 1e-9
            self._msgs[0].angular_velocity_covariance[0] = 1e-9
            self._msgs[0].angular_velocity_covariance[4] = 1e-9
            self._msgs[0].angular_velocity_covariance[8] = 1e-9
            self._msgs[0].linear_acceleration_covariance[0] = 1e-9
            self._msgs[0].linear_acceleration_covariance[4] = 1e-9
            self._msgs[0].linear_acceleration_covariance[8] = 1e-9
            quaternion = response_meta_data["receive"][self._body_name]["quaternion"]
            angular_velocity = response_meta_data["receive"][self._body_name]["angular_velocity"]
            linear_acceleration = response_meta_data["receive"][self._body_name]["linear_acceleration"]
            self._msgs[0].orientation.w = quaternion[0]
            self._msgs[0].orientation.x = quaternion[1]
            self._msgs[0].orientation.y = quaternion[2]
            self._msgs[0].orientation.z = quaternion[3]
            self._msgs[0].angular_velocity.x = angular_velocity[0]
            self._msgs[0].angular_velocity.y = angular_velocity[1]
            self._msgs[0].angular_velocity.z = angular_velocity[2]
            self._msgs[0].linear_acceleration.x = linear_acceleration[0]
            self._msgs[0].linear_acceleration.y = linear_acceleration[1]
            self._msgs[0].linear_acceleration.z = linear_acceleration[2]
        self.bind_response_meta_data_callback = bind_response_meta_data

        def bind_send_data() -> None:
            self.send_data = [self.sim_time]
        self.bind_send_data_callback = bind_send_data

        def bind_receive_data() -> None:
            receive_data = self.receive_data
            if INTERFACE == Interface.ROS1:
                self._msgs[0].header.stamp = rospy.Time.now()
                self._msgs[0].header.seq += 1
            elif INTERFACE == Interface.ROS2:
                self._msgs[0].header.stamp = self.get_clock().now().to_msg()
            self._msgs[0].angular_velocity.x = receive_data[1]
            self._msgs[0].angular_velocity.y = receive_data[2]
            self._msgs[0].angular_velocity.z = receive_data[3]
            self._msgs[0].linear_acceleration.x = receive_data[4]
            self._msgs[0].linear_acceleration.y = receive_data[5]
            self._msgs[0].linear_acceleration.z = receive_data[6]
            self._msgs[0].orientation.w = receive_data[7]
            self._msgs[0].orientation.x = receive_data[8]
            self._msgs[0].orientation.y = receive_data[9]
            self._msgs[0].orientation.z = receive_data[10]
        self.bind_receive_data_callback = bind_receive_data
