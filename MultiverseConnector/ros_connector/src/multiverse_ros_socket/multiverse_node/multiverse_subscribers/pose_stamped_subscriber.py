#!/usr/bin/env python3

from typing import Dict

from geometry_msgs.msg import PoseStamped

from .multiverse_subscriber import MultiverseSubscriber, MultiverseMetaData


class PoseStampedSubscriber(MultiverseSubscriber):
    _use_meta_data = False
    _body_name: str
    _msg_type = PoseStamped
    _init_pose = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]

    def __init__(
            self,
            port: str,
            topic_name: str = "/move_base_simple/goal",
            multiverse_meta_data: MultiverseMetaData = MultiverseMetaData(),
            **kwargs: Dict
    ) -> None:
        if "body" not in kwargs:
            raise Exception("Body not found.")
        self._body_name = str(kwargs["body"])
        if "init_pose" in kwargs:
            self._init_pose = [*kwargs["init_pose"]]
        super().__init__(
            port=port,
            topic_name=topic_name,
            multiverse_meta_data=multiverse_meta_data
        )

        def bind_request_meta_data() -> None:
            self.request_meta_data["send"][self._body_name] = ["position", "quaternion"]

        self.bind_request_meta_data_callback = bind_request_meta_data

    def _init_send_data(self) -> None:
        self.send_data = [self.sim_time] + self._init_pose

    def _bind_send_data(self, pose_stamped_msg: PoseStamped) -> None:
        self.send_data = [
            self.sim_time,
            pose_stamped_msg.pose.position.x,
            pose_stamped_msg.pose.position.y,
            pose_stamped_msg.pose.position.z,
            pose_stamped_msg.pose.orientation.w,
            pose_stamped_msg.pose.orientation.x,
            pose_stamped_msg.pose.orientation.y,
            pose_stamped_msg.pose.orientation.z,
        ]
