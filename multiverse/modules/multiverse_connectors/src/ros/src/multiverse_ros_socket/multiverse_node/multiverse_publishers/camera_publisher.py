#!/usr/bin/env python3

from typing import List, Dict
import numpy

from sensor_msgs.msg import Image

from .multiverse_publisher import Interface, INTERFACE

if INTERFACE == Interface.ROS1:
    import rospy

from .multiverse_publisher import MultiversePublisher, SocketAddress, MultiverseMetaData


class CameraPublisher(MultiversePublisher):
    _use_meta_data = False
    _msg_type = Image
    _body_name: str
    _frame_id: str

    def __init__(
        self,
        client_addr: SocketAddress,
        topic_name: str = "/camera",
        rate: float = 60.0,
        multiverse_meta_data: MultiverseMetaData = MultiverseMetaData(),
        **kwargs: Dict,
    ) -> None:
        super().__init__(
            client_addr=client_addr,
            topic_name=topic_name,
            rate=rate,
            multiverse_meta_data=multiverse_meta_data,
        )
        if "camera" not in kwargs:
            raise Exception("Camera not found.")
        if "image_res" not in kwargs:
            raise Exception("Image resolution not found.")
        self._camera_name = str(kwargs["camera"])
        self._frame_id = str(kwargs.get("frame_id", "map"))
        image_res = str(kwargs["image_res"])
        if image_res == "3840_2160":
            self._msg.width = 3840
            self._msg.height = 2160
            self.request_meta_data["receive"][self._camera_name] = ["rgb_3840_2160"]
        elif image_res == "1280_1024":
            self._msg.width = 1280
            self._msg.height = 1024
            self.request_meta_data["receive"][self._camera_name] = ["rgb_1280_1024"]
        elif image_res == "640_480":
            self._msg.width = 640
            self._msg.height = 480
            self.request_meta_data["receive"][self._camera_name] = ["rgb_640_480"]
        elif image_res == "128_128":
            self._msg.width = 128
            self._msg.height = 128
            self.request_meta_data["receive"][self._camera_name] = ["rgb_128_128"]
        else:
            raise Exception(f"Invalid image resolution {image_res}.")
        
        if INTERFACE == Interface.ROS1:
            self._msg.header.stamp = rospy.Time.now()
            self._msg.header.seq = 0
        elif INTERFACE == Interface.ROS2:
            self._msg.header.stamp = self.get_clock().now().to_msg()
        self._msg.header.frame_id = self._frame_id
        
        self._msg.encoding = "rgb8"
        self._msg.is_bigendian = False
        self._msg.step = 3

    def _bind_response_meta_data(self, response_meta_data) -> None:
        if response_meta_data.get("receive") is None:
            return

    def _bind_receive_data(self, receive_data: List[float]) -> None:
        if len(receive_data) != self._msg.height * self._msg.width * 3 + 1:
            self.logwarn(
                f"Invalid data length {len(receive_data)} for camera {self._camera_name}."
            )
            return

        if INTERFACE == Interface.ROS1:
            self._msg.header.stamp = rospy.Time.now()
            self._msg.header.seq += 1
        elif INTERFACE == Interface.ROS2:
            self._msg.header.stamp = self.get_clock().now().to_msg()
        
        data_array = numpy.array(receive_data[1:], dtype=numpy.uint8)
        self._msg.data = data_array.tobytes()
