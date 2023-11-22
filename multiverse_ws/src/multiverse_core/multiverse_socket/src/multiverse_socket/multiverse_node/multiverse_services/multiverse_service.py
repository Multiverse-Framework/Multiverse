#!/usr/bin/env python3

import sys
from typing import Dict, Any

import rospy

from ..multiverse_node import MultiverseNode, MultiverseMetaData, SocketAddress


class MultiverseService(MultiverseNode):
    _srv_name = ""
    _srv_class: Any = None
    _srv_request_class: Any = None
    _srv_response_class: Any = None

    def __init__(
        self,
        client_addr: SocketAddress = SocketAddress(),
        multiverse_meta_data: MultiverseMetaData = MultiverseMetaData(),
        **kwargs: Dict
    ) -> None:
        MultiverseNode.__init__(
            self, client_addr=client_addr, multiverse_meta_data=multiverse_meta_data
        )
        rospy.Service(
            name=self._srv_name,
            service_class=self._srv_class,
            handler=self._service_handler,
        )

    @property
    def response(self) -> Any:
        return self._srv_response_class()

    def _run(self) -> None:
        self._connect()
        rospy.spin()
        self._disconnect()

    def _service_handler(self, request: Any) -> Any:
        self._bind_request_meta_data(request)
        self._communicate(True)
        return self._bind_response_meta_data(self.response)
