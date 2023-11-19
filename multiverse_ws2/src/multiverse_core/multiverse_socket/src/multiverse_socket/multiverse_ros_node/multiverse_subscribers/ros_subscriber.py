#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from typing import List, Any
from ..multiverse_ros_node import MultiverseRosNode, SimulationMetaData


class MultiverseRosSubscriber(MultiverseRosNode, Node):
    _msg_type = None
    _send_data: List[float] = []
    _receive_data: List[float] = []

    def __init__(
            self,
            topic_name: str,
            node_name: str,
            client_host: str = "tcp://127.0.0.1",
            client_port: str = "",
            simulation_metadata: SimulationMetaData = SimulationMetaData()
    ) -> None:
        MultiverseRosNode.__init__(self, client_host=client_host, client_port=client_port,
                                   simulation_metadata=simulation_metadata)
        Node.__init__(self, node_name=node_name)
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self)
        self.create_subscription(msg_type=self._msg_type, topic=topic_name, callback=self._subscriber_callback, qos_profile=1)

    def start(self) -> None:
        self._init_multiverse_socket()
        self._set_request_meta_data()
        self._connect()
        self._get_response_meta_data()
        self._init_send_data()
        self._set_send_data(self._send_data)
        self._communicate()
        while rclpy.ok():
            self._receive_data = self._get_receive_data()
            if len(self._receive_data) > 0:
                break
        self._executor.spin()
        self._disconnect()
        self.destroy_node()

    def _subscriber_callback(self, data: Any) -> None:
        self._bind_send_data(data)
        self._set_send_data(self._send_data)
        self._communicate()
        self._receive_data = self._get_receive_data()

    def _init_send_data(self) -> None:
        pass

    def _bind_send_data(self, data: Any):
        pass
