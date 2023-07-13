#!/usr/bin/env python3

import rospy
from typing import Any
from multiverse_client.multiverse_ros_base import MultiverseRosBase


class MultiverseRosServiceServer(MultiverseRosBase):
    _service_name = ""
    _service_class = None

    def __init__(self, host: str, port: str, **kwargs) -> None:
        super().__init__(host, port)
        self.use_thread = False

    def start(self) -> None:
        rospy.Service(self._service_name, self._service_class,
                      self._service_handle)
        rospy.loginfo(f"Start service {self._service_name}")
        rospy.spin()

    def _service_handle(self, request) -> Any:
        self._init_multiverse_socket()
        self._bind_send_meta_data(request)
        self._assign_send_meta_data()
        self._connect()
        self._retrieve_receive_meta_data()
        self._disconnect()
        return self._bind_response()

    def _bind_send_meta_data(self, request):
        pass

    def _bind_response(self) -> Any:
        pass
