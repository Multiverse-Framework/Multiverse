#!/usr/bin/env python3

from typing import Dict
import numpy

from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

from .multiverse_publisher import Interface, INTERFACE

if INTERFACE == Interface.ROS1:
    import rospy

from .multiverse_publisher import MultiversePublisher, SocketAddress, MultiverseMetaData


class TfPublisher(MultiversePublisher):
    _use_meta_data = True
    _msg_type = TFMessage
    _root_frame_id: str
    _seq: int = 0

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
        self._root_frame_id = kwargs.get("root_frame_id", "map")
        self.request_meta_data["receive"][""] = ["position", "quaternion"]
        if INTERFACE == Interface.ROS1:
            self._seq = 0

    def _bind_response_meta_data(self, response_meta_data: Dict) -> None:
        objects = response_meta_data.get("receive")

        if objects is None:
            return

        self._msg.transforms.clear()

        for object_name, tf_data in objects.items():
            tf_data = response_meta_data["receive"][object_name]
            if any([p is None for p in tf_data["position"]]) or any(
                    [q is None for q in tf_data["quaternion"]]
            ):
                continue
            tf_msg = TransformStamped()
            tf_msg.header.frame_id = self._root_frame_id

            if INTERFACE == Interface.ROS1:
                tf_msg.header.stamp = rospy.Time.now()
                tf_msg.header.seq = self._seq
            elif INTERFACE == Interface.ROS2:
                tf_msg.header.stamp = self.get_clock().now().to_msg()

            tf_msg.child_frame_id = object_name
            tf_msg.transform.translation.x = float(tf_data["position"][0])
            tf_msg.transform.translation.y = float(tf_data["position"][1])
            tf_msg.transform.translation.z = float(tf_data["position"][2])
            quaternion = numpy.array(
                [float(tf_data["quaternion"][i]) for i in range(4)]
            )
            quaternion = quaternion / numpy.linalg.norm(quaternion)
            tf_msg.transform.rotation.w = quaternion[0]
            tf_msg.transform.rotation.x = quaternion[1]
            tf_msg.transform.rotation.y = quaternion[2]
            tf_msg.transform.rotation.z = quaternion[3]
            self._msg.transforms.append(tf_msg)

        # if INTERFACE == Interface.ROS1:
        #     self._seq += 1
