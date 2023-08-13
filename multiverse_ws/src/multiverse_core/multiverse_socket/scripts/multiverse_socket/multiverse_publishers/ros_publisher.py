#!/usr/bin/env python3

import rospy
from typing import List, Dict
from multiverse_socket import MultiverseRosBase


class MultiverseRosPublisher(MultiverseRosBase):
    _use_meta_data = False

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.rate = rospy.Rate(float(kwargs.get("rate", 60)))

    def start(self) -> None:
        self._init_multiverse_socket()
        self._set_request_meta_data()
        self._connect()
        if self._use_meta_data:
            while not rospy.is_shutdown():
                self._communicate(True)
                self._construct_rosmsg(self._get_response_meta_data())
                if not rospy.is_shutdown():
                    self._publish()
                self.rate.sleep()

        else:
            self._construct_rosmsg(self._get_response_meta_data())

            while not rospy.is_shutdown():
                self._communicate()
                self._bind_rosmsg(self._get_receive_data())
                if not rospy.is_shutdown():
                    self._publish()
                self.rate.sleep()

        self._disconnect()

    def _construct_rosmsg(self, response_meta_data_dict: Dict) -> None:
        pass

    def _bind_rosmsg(self, receive_data: List[float]) -> None:
        pass

    def _publish(self) -> None:
        pass
