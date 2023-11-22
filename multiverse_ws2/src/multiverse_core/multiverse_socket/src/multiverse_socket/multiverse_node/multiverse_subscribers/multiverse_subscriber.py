#!/usr/bin/env python3

from typing import List, Dict, Any

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from ..multiverse_node import MultiverseNode, MultiverseMetaData, SocketAddress


class MultiverseSubscriber(MultiverseNode, Node):
    _msg_type = None
    _send_data: List[float] = []
    _receive_data: List[float] = []
    _executor: MultiThreadedExecutor

    def __init__(
        self,
        topic_name: str,
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
        self.create_subscription(
            msg_type=self._msg_type,
            topic=topic_name,
            callback=self._subscriber_callback,
            qos_profile=1,
        )

    def _run(self) -> None:
        self._connect()
        self._init_send_data()
        self._communicate()
        while rclpy.ok():
            if len(self.receive_data) > 0:
                break
        self._executor.spin()
        self._disconnect()
        self.destroy_node()

    def _subscriber_callback(self, data: Any) -> None:
        self._bind_send_data(data)
        self._communicate()

    def _init_send_data(self) -> None:
        pass
