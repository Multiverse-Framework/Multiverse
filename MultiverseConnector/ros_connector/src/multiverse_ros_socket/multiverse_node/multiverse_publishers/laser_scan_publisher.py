#!/usr/bin/env python3

from typing import Dict, List
import numpy

from sensor_msgs.msg import LaserScan

from .multiverse_publisher import Interface, INTERFACE

if INTERFACE == Interface.ROS1:
    import rospy

from .multiverse_publisher import MultiversePublisher, MultiverseMetaData


class LaserScanPublisher(MultiversePublisher):
    _use_meta_data = False
    _msg_types = [LaserScan]
    _laser_names: List[str]
    _frame_id: str
    _angle_min: float
    _angle_max: float
    _angle_increment: float
    _range_min: float
    _range_max: float

    def __init__(
        self,
        port: str,
        topic_name: str = "/laser_scan",
        rate: float = 60.0,
        multiverse_meta_data: MultiverseMetaData = MultiverseMetaData(),
        **kwargs: Dict,
    ) -> None:
        super().__init__(
            port=port,
            topic_name=topic_name,
            rate=rate,
            multiverse_meta_data=multiverse_meta_data,
        )
        assert "angle_min" in kwargs, "Angle min not found."
        self._angle_min = float(kwargs["angle_min"])
        assert "angle_max" in kwargs, "Angle max not found."
        self._angle_max = float(kwargs["angle_max"])
        assert "angle_increment" in kwargs, "Angle increment not found."
        self._angle_increment = float(kwargs["angle_increment"])
        assert "range_min" in kwargs, "Range min not found."
        self._range_min = float(kwargs["range_min"])
        assert "range_max" in kwargs, "Range max not found."
        self._range_max = float(kwargs["range_max"])
        assert "laser_name" in kwargs, "Laser name not found."
        laser_base_name = str(kwargs["laser_name"])
        laser_length = int((self._angle_max - self._angle_min) / self._angle_increment) + 1
        self._msgs[0].ranges = [0.0] * laser_length

        self._frame_id = str(kwargs.get("frame_id", "map"))
        
        if INTERFACE == Interface.ROS1:
            self._msgs[0].header.stamp = rospy.Time.now()
            self._msgs[0].header.seq = 0
        elif INTERFACE == Interface.ROS2:
            self._msgs[0].header.stamp = self.get_clock().now().to_msg()
        self._msgs[0].header.frame_id = self._frame_id
        
        self._msgs[0].angle_min = self._angle_min
        self._msgs[0].angle_max = self._angle_max
        self._msgs[0].angle_increment = self._angle_increment
        self._msgs[0].scan_time = 1.0 / rate
        self._msgs[0].range_min = self._range_min
        self._msgs[0].range_max = self._range_max
        
        laser_ids = [0.0] * laser_length
        laser_map = {}

        def bind_request_meta_data() -> None:
            for i in range(laser_length):
                laser_name = f"{laser_base_name}_{i}"
                self.request_meta_data["receive"][laser_name] = ["scalar"]
                laser_map[laser_name] = i
        self.bind_request_meta_data_callback = bind_request_meta_data

        def bind_response_meta_data() -> None:
            response_meta_data = self.response_meta_data
            for i, laser_name in enumerate(response_meta_data["receive"].keys()):
                laser_id = laser_map[laser_name]
                laser_ids[laser_id] = i
        self.bind_response_meta_data_callback = bind_response_meta_data

        def bind_send_data() -> None:
            self.send_data = [self.sim_time]
        self.bind_send_data_callback = bind_send_data

        def bind_receive_data() -> None:
            if INTERFACE == Interface.ROS1:
                self._msgs[0].header.stamp = rospy.Time.now()
                self._msgs[0].header.seq += 1
            elif INTERFACE == Interface.ROS2:
                self._msgs[0].header.stamp = self.get_clock().now().to_msg()

            receive_data = numpy.array(self.receive_data[1:], dtype=float)
            self._msgs[0].ranges = receive_data[laser_ids].tolist()
        self.bind_receive_data_callback = bind_receive_data
