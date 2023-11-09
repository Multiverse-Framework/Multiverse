#!/usr/bin/env python3

import rospy
from typing import Any
from multiverse_socket import MultiverseRosBase


class MultiverseRosServiceServer(MultiverseRosBase):
    _service_name = ""
    _service_class = None

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

    def start(self) -> None:
        self._init_multiverse_socket()
        self._init_request_meta_data()
        self._set_request_meta_data()
        self._connect()
        rospy.Service(self._service_name, self._service_class, self._service_handle)
        rospy.loginfo(f"Start service at {self._service_name}")
        rospy.spin()
        self._disconnect()

    def _service_handle(self, request) -> Any:
        self._bind_request_meta_data(request)
        self._set_request_meta_data()
        self._communicate(True)
        return self._bind_response()

    def _bind_request_meta_data(self, request):
        pass

    def _bind_response(self) -> Any:
        pass
