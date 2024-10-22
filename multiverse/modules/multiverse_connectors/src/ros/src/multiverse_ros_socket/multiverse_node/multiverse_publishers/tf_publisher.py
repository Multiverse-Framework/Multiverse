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
    _msg_types = [TFMessage]
    _root_frame_id: str
    _seq: int = 0

    def __init__(
            self,
            client_addr: SocketAddress,
            topic_name: str = "/tf",
            rate: float = 60.0,
            multiverse_meta_data: MultiverseMetaData = MultiverseMetaData(),
            **kwargs: Dict
    ) -> None:
        self._root_frame_id = kwargs.get("root_frame_id", "map")
        if INTERFACE == Interface.ROS1:
            self._seq = 0
        super().__init__(
            topic_name=topic_name,
            rate=rate,
            client_addr=client_addr,
            multiverse_meta_data=multiverse_meta_data,
        )

        def bind_request_meta_data() -> None:
            request_meta_data = self.request_meta_data
            request_meta_data["receive"] = {"": ["position", "quaternion"]}
        self.bind_request_meta_data_callback = bind_request_meta_data

        def bind_response_meta_data() -> None:
            response_meta_data = self.response_meta_data
            objects = response_meta_data.get("receive")

            if objects is None:
                return

            self._msgs[0].transforms.clear()

            for object_name, tf_data in objects.items():
                tf_data = response_meta_data["receive"][object_name]
                if "position" not in tf_data or "quaternion" not in tf_data:
                    continue
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
                self._msgs[0].transforms.append(tf_msg)

            if INTERFACE == Interface.ROS1:
                self._seq += 1
        self.bind_response_meta_data_callback = bind_response_meta_data