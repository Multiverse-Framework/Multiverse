#!/usr/bin/env python3

from typing import List, Dict
import numpy

from std_msgs.msg import Header
from sensor_msgs.msg import RegionOfInterest, CameraInfo, Image

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
            **kwargs: Dict
    ) -> None:
        super().__init__(
            client_addr=client_addr,
            topic_name=topic_name,
            rate=rate,
            multiverse_meta_data=multiverse_meta_data,
        )
        if "camera" not in kwargs:
            raise Exception("Camera not found.")
        self._camera_name = str(kwargs["camera"])
        self._frame_id = str(kwargs.get("frame_id", "map"))
        self.request_meta_data["receive"][self._camera_name] = ["rgb_640_480"]
        self._publish_camera_info(f"{topic_name}_info", rate)

    def _publish_camera_info(self, topic_name: str, rate: float) -> None:
        self._camera_info_msg = CameraInfo()

        header = Header()
        header.frame_id = self._frame_id
        self._camera_info_msg.header = header

        self._camera_info_msg.height = 640
        self._camera_info_msg.width = 480
        self._camera_info_msg.distortion_model = "plumb_bob"
        # self._camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        # self._camera_info_msg.k = [1.0, 0.0, 1920.0, 0.0, 1.0, 1080.0, 0.0, 0.0, 1.0]
        # self._camera_info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        # self._camera_info_msg.p = [1.0, 0.0, 1920.0, 0.0, 0.0, 1080.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self._camera_info_msg.binning_x = 0
        self._camera_info_msg.binning_y = 0

        roi = RegionOfInterest()
        roi.x_offset = 0
        roi.y_offset = 0
        roi.height = 0
        roi.width = 0
        roi.do_rectify = False

        self._camera_info_msg.roi = roi

        if INTERFACE == Interface.ROS1:
            self._seq = 0
            self._camera_info_publisher = rospy.Publisher(topic_name, CameraInfo, queue_size=100)
            duration_in_seconds = 1.0 / rate
            secs = int(duration_in_seconds)
            nsecs = int((duration_in_seconds - secs) * 1e9)
            r = rospy.Rate(1)
            r.sleep()
            rospy.Timer(
                period=rospy.Duration(secs=secs, nsecs=nsecs),
                callback=self._publisher_camera_info_callback
            )
        elif INTERFACE == Interface.ROS2:
            self._camera_info_publisher = self.create_publisher(CameraInfo, topic_name, 100)
            self.create_timer(
                timer_period_sec=1.0 / rate,
                callback=self._publisher_camera_info_callback
            )

    def _publisher_camera_info_callback(self, _=None) -> None:
        if INTERFACE == Interface.ROS1:
            self._camera_info_msg.header.stamp = rospy.Time.now()
            self._camera_info_msg.header.seq = self._seq
            self._seq += 1
        elif INTERFACE == Interface.ROS2:
            self._camera_info_msg.header.stamp = self.get_clock().now().to_msg()
        self._camera_info_publisher.publish(self._camera_info_msg)

    def _bind_response_meta_data(self, response_meta_data) -> None:
        if response_meta_data.get("receive") is None:
            return
        self._msg.height = 640
        self._msg.width = 480
        self._msg.encoding = "rgb8"
        self._msg.is_bigendian = False
        self._msg.step = 3 * 480

    def _bind_receive_data(self, receive_data: List[float]) -> None:
        if len (receive_data) != 640 * 480 + 1:
            return
            
        self._msg.header = self._camera_info_msg.header
        self._msg.data = [0] * 640 * 480 * 3

        receive_data[len(receive_data) - 1] = 255190012.0

        for i, data in enumerate(receive_data[1:]):
            self._msg.data[3 * i] = data // 1000000
            self._msg.data[3 * i + 1] = (data % 1000000) // 1000
            self._msg.data[3 * i + 2] = data % 1000

        self._msg.data = numpy.array(self._msg.data, dtype=numpy.uint8).tobytes()
