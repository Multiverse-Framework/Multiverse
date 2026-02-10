#!/usr/bin/env python3

from typing import Dict, List

from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

from .multiverse_subscriber import Interface, INTERFACE

if INTERFACE == Interface.ROS2:
    import rclpy

from .multiverse_subscriber import MultiverseSubscriber, MultiverseMetaData


class TfSubscriber(MultiverseSubscriber):
    _use_meta_data = True
    _root_frame_id: str
    _msg_type = [TFMessage]

    def __init__(
        self,
        port: str,
        topic_name: str = "/tf",
        multiverse_meta_data: MultiverseMetaData = MultiverseMetaData(),
        **kwargs: Dict,
    ) -> None:
        self._root_frame_id = str(kwargs.get("root_frame_id", "map"))
        self._rate = float(kwargs.get("rate", 60.0))
        super().__init__(
            port=port, topic_name=topic_name, multiverse_meta_data=multiverse_meta_data
        )

    def _create_subscriber(self, topic_name: str) -> None:
        assert topic_name == "/tf", "TfSubscriber only supports /tf topic"
        if INTERFACE == Interface.ROS2:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.timer = self.create_timer(1.0 / self._rate, self._subscriber_callback)
        else:
            raise NotImplementedError("TfSubscriber not implemented for ROS1 yet")

    def _subscriber_callback(self) -> None:
        now = rclpy.time.Time()
        frame_ids = []

        try:
            all_frames = self.tf_buffer.all_frames_as_yaml().splitlines()
            frame_ids = [frame[:-1] for frame in all_frames if frame.endswith(":")]
            for frame in frame_ids:
                self.get_logger().warn(frame)
        except Exception as e:
            self.get_logger().warn(f"Failed to retrieve frame list: {str(e)}")
            return

        if "send" not in self.response_meta_data or any(
            body_name not in self.response_meta_data["send"] for body_name in frame_ids
        ):
            self.__bind_request_meta_data(frame_ids)
            self._communicate(True)

        send_data = [self.sim_time] + [0.0] * len(frame_ids) * 7
        for i, to_frame in enumerate(frame_ids):
            if to_frame == self._root_frame_id:
                continue
            try:
                transform: TransformStamped = self.tf_buffer.lookup_transform(
                    to_frame,
                    self._root_frame_id,
                    now,
                    timeout=rclpy.duration.Duration(seconds=0.5),
                )
                send_data[i * 7 + 1] = transform.transform.translation.x
                send_data[i * 7 + 2] = transform.transform.translation.y
                send_data[i * 7 + 3] = transform.transform.translation.z
                send_data[i * 7 + 4] = transform.transform.rotation.w
                send_data[i * 7 + 5] = transform.transform.rotation.x
                send_data[i * 7 + 6] = transform.transform.rotation.y
                send_data[i * 7 + 7] = transform.transform.rotation.z
            except Exception as e:
                self.get_logger().warn(
                    f"No transform from {self._root_frame_id} to {to_frame}: {str(e)}"
                )

        self.send_data = send_data
        self._communicate(False)

    def __bind_request_meta_data(self, frame_ids: List[str]) -> None:
        self.request_meta_data["send"] = {}
        for frame_id in frame_ids:
            self.request_meta_data["send"][frame_id] = ["position", "quaternion"]
