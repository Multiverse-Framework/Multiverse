#!/usr/bin/env python3

from typing import List, Any

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from ..multiverse_ros_node import MultiverseRosNode, SimulationMetaData


class MultiverseRosSubscriber(MultiverseRosNode, Node):
    _msg_type = None
    _send_data: List[float] = []
    _receive_data: List[float] = []
    _executor: MultiThreadedExecutor

    def __init__(
            self,
            topic_name: str,
            node_name: str,
            client_host: str = "tcp://127.0.0.1",
            client_port: str = "",
            simulation_metadata: SimulationMetaData = SimulationMetaData(),
            **kwargs: object
    ) -> None:
        MultiverseRosNode.__init__(self, client_host=client_host, client_port=client_port,
                                   simulation_metadata=simulation_metadata)
        Node.__init__(self, node_name=node_name)
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self)
        self.create_subscription(msg_type=self._msg_type, topic=topic_name, callback=self._subscriber_callback, qos_profile=1)

    def run(self) -> None:
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
