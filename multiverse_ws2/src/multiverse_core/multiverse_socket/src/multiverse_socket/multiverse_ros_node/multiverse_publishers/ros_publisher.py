#!/usr/bin/env python3

from typing import Dict

from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher

from ..multiverse_ros_node import MultiverseRosNode, SimulationMetaData


class MultiverseRosPublisher(MultiverseRosNode, Node):
    _publisher: Publisher
    _msg_type = None
    _use_meta_data: bool = False
    _executor: MultiThreadedExecutor

    def __init__(
            self,
            topic_name: str,
            node_name: str,
            rate: float = 60.0,
            client_host: str = "tcp://127.0.0.1",
            client_port: str = "",
            simulation_metadata: SimulationMetaData = SimulationMetaData(),
            **kwargs: Dict
    ) -> None:
        MultiverseRosNode.__init__(self, client_host=client_host, client_port=client_port,
                                   simulation_metadata=simulation_metadata)
        Node.__init__(self, node_name=node_name)
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self)
        self._publisher = self.create_publisher(self._msg_type, topic_name, 100)
        self.create_timer(timer_period_sec=1.0 / rate, callback=self._publisher_callback)

    def _run(self) -> None:
        self._connect()
        if not self._use_meta_data:
            self._bind_response_meta_data(self.response_meta_data)
        self._executor.spin()
        self._disconnect()
        self.destroy_node()

    def _publisher_callback(self):
        self._communicate(self._use_meta_data)
        if self._use_meta_data:
            self._bind_response_meta_data(self.response_meta_data)
        else:
            self._bind_receive_data(self.receive_data)
        self._publish()

    def _publish(self) -> None:
        pass
