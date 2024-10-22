#!/usr/bin/env python3

from typing import List, Dict
import numpy

from sensor_msgs.msg import Image

from .multiverse_publisher import Interface, INTERFACE

if INTERFACE == Interface.ROS1:
    import rospy

from .multiverse_publisher import MultiversePublisher, SocketAddress, MultiverseMetaData


class ImageRgbPublisher(MultiversePublisher):
    _use_meta_data = False
    _msg_types = [Image]
    _body_name: str
    _frame_id: str

    def __init__(
        self,
        client_addr: SocketAddress,
        topic_name: str = "/camera/rgb/image_raw",
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
        
        if INTERFACE == Interface.ROS1:
            self._msgs[0].header.stamp = rospy.Time.now()
            self._msgs[0].header.seq = 0
        elif INTERFACE == Interface.ROS2:
            self._msgs[0].header.stamp = self.get_clock().now().to_msg()
        self._msgs[0].header.frame_id = self._frame_id
        
        self._msgs[0].encoding = "rgb8"
        self._msgs[0].is_bigendian = False
        self._msgs[0].step = 3

        def bind_request_meta_data() -> None:
            image_res = str(kwargs["image_res"])
            if image_res == "3840_2160":
                self._msgs[0].width = 3840
                self._msgs[0].height = 2160
                self.request_meta_data["receive"][self._camera_name] = ["rgb_3840_2160"]
            elif image_res == "1280_1024":
                self._msgs[0].width = 1280
                self._msgs[0].height = 1024
                self.request_meta_data["receive"][self._camera_name] = ["rgb_1280_1024"]
            elif image_res == "640_480":
                self._msgs[0].width = 640
                self._msgs[0].height = 480
                self.request_meta_data["receive"][self._camera_name] = ["rgb_640_480"]
            elif image_res == "128_128":
                self._msgs[0].width = 128
                self._msgs[0].height = 128
                self.request_meta_data["receive"][self._camera_name] = ["rgb_128_128"]
            else:
                raise Exception(f"Invalid image resolution {image_res}.")
        self.bind_request_meta_data_callback = bind_request_meta_data

        def bind_send_data() -> None:
            self.send_data = [self.world_time + self.sim_time]
        self.bind_send_data_callback = bind_send_data

        def bind_receive_data() -> None:
            receive_data = self.receive_data
            if len(receive_data) != self._msgs[0].height * self._msgs[0].width * 3 + 1:
                self.logwarn(
                    f"Invalid data length {len(receive_data)} for camera {self._camera_name}."
                )
                return

            if INTERFACE == Interface.ROS1:
                self._msgs[0].header.stamp = rospy.Time.now()
                self._msgs[0].header.seq += 1
            elif INTERFACE == Interface.ROS2:
                self._msgs[0].header.stamp = self.get_clock().now().to_msg()
            
            data_array = numpy.array(receive_data[1:], dtype=numpy.uint8)
            self._msgs[0].data = data_array.tobytes()
        self.bind_receive_data_callback = bind_receive_data
