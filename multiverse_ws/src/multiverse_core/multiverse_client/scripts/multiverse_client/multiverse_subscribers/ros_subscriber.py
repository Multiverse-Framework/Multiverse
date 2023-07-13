#!/usr/bin/env python3

import rospy
from typing import Any
from multiverse_client.multiverse_ros_base import MultiverseRosBase


class MultiverseRosSubscriber(MultiverseRosBase):
    _topic_name = ""
    _data_class = None
    _data = []

    def __init__(self, host: str, port: str, **kwargs) -> None:
        super().__init__(host, port)
        self.use_thread = True

    def start(self) -> None:
        self._init_multiverse_socket()
        self._construct_send_meta_data()
        self._assign_send_meta_data()
        self._connect()
        self._retrieve_receive_meta_data()
        self._init_send_data()
        rospy.Subscriber(self._topic_name, self._data_class, self._subscriber_callback)
        rospy.loginfo(f"Start subscriber {self._topic_name}")
        rospy.spin()
        self._disconnect()

    def _subscriber_callback(self, data: Any) -> None:
        self._bind_send_data(data)
        self._send_data(self._data)
        self._receive_data()

    def _construct_send_meta_data(self, request):
        pass

    def _init_send_data(self) -> None:
        pass

    def _bind_send_data(self, data: Any) -> Any:
        pass
