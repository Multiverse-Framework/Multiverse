#!/usr/bin/env python3

from typing import Dict

from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node, MsgType, SrvTypeRequest, SrvTypeResponse

from ..multiverse_node import MultiverseNode, MultiverseMetaData, SocketAddress


class MultiverseService(MultiverseNode, Node):
    _srv_name = ""
    _srv_type: MsgType = None

    def __init__(
        self,
        node_name: str,
        client_addr: SocketAddress = SocketAddress(),
        multiverse_meta_data: MultiverseMetaData = MultiverseMetaData(),
        **kwargs: Dict
    ) -> None:
        MultiverseNode.__init__(
            self, client_addr=client_addr, multiverse_meta_data=multiverse_meta_data
        )
        Node.__init__(self, node_name=node_name)
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self)
        self.create_service(
            srv_type=self._srv_type,
            srv_name=self._srv_name,
            callback=self._service_callback,
        )

    def _run(self) -> None:
        self._connect()
        self._executor.spin()
        self._disconnect()
        self.destroy_node()

    def _service_callback(
        self, request: SrvTypeRequest, response: SrvTypeResponse
    ) -> SrvTypeResponse:
        self._bind_request_meta_data(request)
        self._communicate(True)
        return self._bind_response_meta_data(response)
