#!/usr/bin/env python3

import rospy
from typing import List
from multiverse_client.multiverse_ros_base import MultiverseRosBase

class MultiverseRosPublisher(MultiverseRosBase):
    def __init__(self, host: str, port: str, **kwargs) -> None:
        super().__init__(host, port)
        self.use_thread = True
        self.rate = rospy.Rate(float(kwargs.get('rate', 60)))
        self._prepare_send_meta_data()

    def start(self) -> None:
        self._init_multiverse_socket()
        self._assign_send_meta_data()
        self._connect()

        if not self._retrieve_receive_meta_data():
            return

        self._construct_rosmsg()

        while not rospy.is_shutdown():
            self._send_data([rospy.Time.now().to_sec()])
            receive_data = self._receive_data()
            self._bind_rosmsg(receive_data)
            self._publish()
            self.rate.sleep()
        
        self._disconnect()

    def _prepare_send_meta_data(self) -> None:
        pass

    def _construct_rosmsg(self) -> None:
        pass

    def _bind_rosmsg(self, receive_data: List[float]) -> None:
        pass

    def _publish(self) -> None:
        pass