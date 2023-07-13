#!/usr/bin/env python3

import rospy
from typing import List, Dict
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Transform
from .ros_publisher import MultiverseRosPublisher


class tf_publisher(MultiverseRosPublisher):
    def __init__(self, host: str, port: str) -> None:
        super().__init__(host, port)
        self.tf_msgs = []
        self.tf_broadcaster = TransformBroadcaster()

    def _prepare_send_meta_data(self) -> None:
        self._send_meta_data_dict["receive"][""] = ["position", "quaternion"]

    def _construct_rosmsg(self) -> None:
        self.object_names: List[str]
        if self._receive_meta_data_dict.get("receive") is None:
            return
        
        self.object_names = self._receive_meta_data_dict["receive"].keys()
        root_frame_id = rospy.get_param(
            'multiverse/root_frame_id') if rospy.has_param('multiverse/root_frame_id') else "map"
        
        for object_name in self.object_names:
            tf_msg = TransformStamped()
            tf_msg.header.frame_id = root_frame_id
            tf_msg.child_frame_id = object_name
            self.tf_msgs.append(tf_msg)

    def _bind_rosmsg(self, receive_data: List[float]) -> None:
        for i, _ in enumerate(self.object_names):
            self.tf_msgs[i].header.stamp = rospy.Time.from_sec(receive_data[0])
            self.tf_msgs[i].transform.translation.x = receive_data[7 * i + 1]
            self.tf_msgs[i].transform.translation.y = receive_data[7 * i + 2]
            self.tf_msgs[i].transform.translation.z = receive_data[7 * i + 3]
            self.tf_msgs[i].transform.rotation.w = receive_data[7 * i + 4]
            self.tf_msgs[i].transform.rotation.x = receive_data[7 * i + 5]
            self.tf_msgs[i].transform.rotation.y = receive_data[7 * i + 6]
            self.tf_msgs[i].transform.rotation.z = receive_data[7 * i + 7]

    def _publish(self) -> None:
        self.tf_broadcaster.sendTransform(self.tf_msgs)
