#!/usr/bin/env python3

import rclpy
from typing import List, Dict
from multiverse_socket.multiverse_ros_base import MultiverseRosBase


class MultiverseRosPublisher(MultiverseRosBase):
    _use_meta_data = False

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.topic = str(kwargs.get("topic"))
        self.timer = self.create_timer(float(kwargs.get("rate", 60)), self._publisher_callback)


    def start(self) -> None:
        self._init_multiverse_socket()
        self._set_request_meta_data()
        self._connect()
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            self._disconnect()
        finally:
            self.destroy_node()
        

    def _publisher_callback(self):
        if self._use_meta_data:
            while rclpy.ok():
                self._communicate(True)
                self._construct_rosmsg(self._get_response_meta_data())
                if rclpy.ok():
                    self._publish()
                self.rate.sleep()

        else:
            self._construct_rosmsg(self._get_response_meta_data())
            while rclpy.ok():
                self._communicate()
                self._bind_rosmsg(self._get_receive_data())
                if rclpy.ok():
                    self._publish()
                self.rate.sleep()
        pass

    def _construct_rosmsg(self, response_meta_data_dict: Dict) -> None:
        pass

    def _bind_rosmsg(self, receive_data: List[float]) -> None:
        pass

    def _publish(self) -> None:
        pass
