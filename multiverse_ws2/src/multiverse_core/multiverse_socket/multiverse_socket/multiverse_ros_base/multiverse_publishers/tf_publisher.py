#!/usr/bin/env python3

from typing import Dict, List

import numpy
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

from .ros_publisher import MultiverseRosPublisher
from ..multiverse_ros_base import SocketMetaData, SimulationMetaData


class TfPublisher(MultiverseRosPublisher):
    _tf_msgs: List[TransformStamped] = []
    _topic_name: str = "/tf"
    _root_frame_id: str = "map"

    def __init__(
        self,
        root_frame_id: str,
        topic_name: str,
        node_name: str,
        rate: float = 60.0,
        socket_metadata: SocketMetaData = SocketMetaData(),
        simulation_metadata: SimulationMetaData = SimulationMetaData(),
    ) -> None:
        super().__init__(topic_name=topic_name, node_name=node_name, rate=rate, socket_metadata=socket_metadata, simulation_metadata=simulation_metadata)
        self._use_meta_data = True
        self._publisher = self.create_publisher(TFMessage, self._topic_name, 100)
        self._root_frame_id = root_frame_id

    def _init_request_meta_data(self) -> None:
        super()._init_request_meta_data()
        self._request_meta_data_dict["receive"][""] = ["position", "quaternion"]

    def _construct_ros_message(self, response_meta_data_dict: Dict) -> None:
        self.object_names = []
        if response_meta_data_dict.get("receive") is None:
            return

        self.object_names = response_meta_data_dict["receive"].keys()
        self._tf_msgs = []

        for object_name in self.object_names:
            tf_data = response_meta_data_dict["receive"][object_name]
            tf_msg = TransformStamped()
            tf_msg.header.frame_id = self._root_frame_id
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.child_frame_id = object_name

            tf_msg.transform.translation.x = float(tf_data["position"][0])
            tf_msg.transform.translation.y = float(tf_data["position"][1])
            tf_msg.transform.translation.z = float(tf_data["position"][2])
            quaternion = numpy.array([float(tf_data["quaternion"][0]), float(tf_data["quaternion"][1]), float(tf_data["quaternion"][2]), float(tf_data["quaternion"][3])])
            if any([q is None for q in quaternion]):
                continue
            quaternion = quaternion / numpy.linalg.norm(quaternion)
            tf_msg.transform.rotation.w = quaternion[0]
            tf_msg.transform.rotation.x = quaternion[1]
            tf_msg.transform.rotation.y = quaternion[2]
            tf_msg.transform.rotation.z = quaternion[3]
            self._tf_msgs.append(tf_msg)

    def _publish(self) -> None:
        self._publisher.publish(TFMessage(transforms=self._tf_msgs))
