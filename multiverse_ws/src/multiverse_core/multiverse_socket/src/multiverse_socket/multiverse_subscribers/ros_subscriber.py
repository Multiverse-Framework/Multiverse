#!/usr/bin/env python3

import rospy
from typing import Any
from multiverse_socket import MultiverseRosBase


class MultiverseRosSubscriber(MultiverseRosBase):
    _data_class = None
    _send_data = []
    _receive_data = []

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.topic = str(kwargs.get("topic"))

    def start(self) -> None:
        self._init_multiverse_socket()
        self._set_request_meta_data()
        self._connect()
        self._get_response_meta_data()
        self._init_send_data()
        self._set_send_data(self._send_data)
        self._communicate()
        while not rospy.is_shutdown():
            self._receive_data = self._get_receive_data()
            if len(self._receive_data) > 0:
                break
        rospy.Subscriber(self.topic, self._data_class, self._subscriber_callback)
        rospy.loginfo(f"Start subscriber {self.topic}")
        rospy.spin()
        self._disconnect()

    def _subscriber_callback(self, data: Any) -> None:
        self._bind_send_data(data)
        self._set_send_data(self._send_data)
        self._communicate()
        self._receive_data = self._get_receive_data()

    def _init_send_data(self) -> None:
        pass

    def _bind_send_data(self, data: Any):
        pass
