#!/usr/bin/env python3

import rospy
from typing import Dict
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from multiverse_socket.multiverse_publishers import MultiverseRosPublisher
import numpy


class tf_publisher(MultiverseRosPublisher):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self._use_meta_data = True
        self.__tf_broadcaster = TransformBroadcaster()
        self.__refresh_time = rospy.Time.now()
        self.__tf_msgs = []
        self.__root_frame_id = rospy.get_param("multiverse/root_frame_id") if rospy.has_param("multiverse/root_frame_id") else "map"
        self.__seq = 0

    def _init_request_meta_data(self) -> None:
        super()._init_request_meta_data()
        self._request_meta_data_dict["receive"][""] = ["position", "quaternion"]

    def _construct_rosmsg(self, response_meta_data_dict: Dict) -> None:
        self.object_names = []
        if response_meta_data_dict.get("receive") is None:
            return

        self.object_names = response_meta_data_dict["receive"].keys()
        self.__tf_msgs = []

        for object_name in self.object_names:
            tf_data = response_meta_data_dict["receive"][object_name]
            tf_msg = TransformStamped()
            tf_msg.header.frame_id = self.__root_frame_id
            tf_msg.header.stamp = rospy.Time.now()
            tf_msg.header.seq = self.__seq
            tf_msg.child_frame_id = object_name

            tf_msg.transform.translation.x = tf_data["position"][0]
            tf_msg.transform.translation.y = tf_data["position"][1]
            tf_msg.transform.translation.z = tf_data["position"][2]
            quat = numpy.array([tf_data["quaternion"][0], tf_data["quaternion"][1], tf_data["quaternion"][2], tf_data["quaternion"][3]])
            try:
                quat = quat / numpy.linalg.norm(quat)
            except TypeError as e:
                rospy.logwarn(e)
                continue
            tf_msg.transform.rotation.w = quat[0]
            tf_msg.transform.rotation.x = quat[1]
            tf_msg.transform.rotation.y = quat[2]
            tf_msg.transform.rotation.z = quat[3]
            self.__tf_msgs.append(tf_msg)

        self.__seq += 1

    def _publish(self) -> None:
        self.__tf_broadcaster.sendTransform(self.__tf_msgs)
