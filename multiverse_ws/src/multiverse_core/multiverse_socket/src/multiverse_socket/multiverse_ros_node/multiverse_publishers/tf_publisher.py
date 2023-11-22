#!/usr/bin/env python3

from typing import Dict, List

import numpy
import rospy
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

from .ros_publisher import MultiverseRosPublisher
from ..multiverse_ros_node import SimulationMetaData


class TfPublisher(MultiverseRosPublisher):
    _use_meta_data = True
    _msg_type = TFMessage
    _tf_msgs: List[TransformStamped] = []
    _root_frame_id: str

    def __init__(
            self,
            topic_name: str = "/tf",
            rate: float = 60.0,
            client_host: str = "tcp://127.0.0.1",
            client_port: str = "",
            simulation_metadata: SimulationMetaData = SimulationMetaData(),
            **kwargs: Dict
    ) -> None:
        super().__init__(topic_name=topic_name, rate=rate, client_host=client_host,
                         client_port=client_port, simulation_metadata=simulation_metadata)
        self._root_frame_id = kwargs.get("root_frame_id", "map")
        self.request_meta_data["receive"][""] = ["position", "quaternion"]

    def _bind_response_meta_data(self, response_meta_data: Dict) -> None:
        objects = response_meta_data.get("receive")

        if objects is None:
            return

        self._tf_msgs = []

        for object_name, tf_data in objects.items():
            tf_data = response_meta_data["receive"][object_name]
            if any([p is None for p in tf_data["position"]]) or any([q is None for q in tf_data["quaternion"]]):
                continue
            tf_msg = TransformStamped()
            tf_msg.header.frame_id = self._root_frame_id
            tf_msg.header.stamp = rospy.Time.now()
            tf_msg.child_frame_id = object_name
            tf_msg.transform.translation.x = float(tf_data["position"][0])
            tf_msg.transform.translation.y = float(tf_data["position"][1])
            tf_msg.transform.translation.z = float(tf_data["position"][2])
            quaternion = numpy.array([float(tf_data["quaternion"][i]) for i in range(4)])
            quaternion = quaternion / numpy.linalg.norm(quaternion)
            tf_msg.transform.rotation.w = quaternion[0]
            tf_msg.transform.rotation.x = quaternion[1]
            tf_msg.transform.rotation.y = quaternion[2]
            tf_msg.transform.rotation.z = quaternion[3]
            self._tf_msgs.append(tf_msg)

    def _publish(self) -> None:
        self._publisher.publish(TFMessage(transforms=self._tf_msgs))
