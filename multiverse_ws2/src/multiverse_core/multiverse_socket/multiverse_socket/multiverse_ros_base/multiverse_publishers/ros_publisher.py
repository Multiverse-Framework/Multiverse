#!/usr/bin/env python3

from rclpy.executors import MultiThreadedExecutor
from rclpy.publisher import Publisher
from rclpy.node import Node
from typing import List, Dict
from ..multiverse_ros_base import MultiverseRosBase, SimulationMetaData, SocketMetaData


class MultiverseRosPublisher(MultiverseRosBase, Node):
    _publisher: Publisher
    _use_meta_data: bool = False
    _topic_name: str = ""
    _executor: MultiThreadedExecutor

    def __init__(
        self,
        topic_name: str,
        node_name: str,
        rate: float = 60.0,
        socket_metadata: SocketMetaData = SocketMetaData(),
        simulation_metadata: SimulationMetaData = SimulationMetaData(),
    ):
        MultiverseRosBase.__init__(self, socket_metadata=socket_metadata, simulation_metadata=simulation_metadata)
        Node.__init__(self, node_name=node_name)
        self._topic_name = topic_name
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self)
        self.create_timer(1.0/rate, self._publisher_callback)

    def start(self) -> None:
        self._init_multiverse_socket()
        self._set_request_meta_data()
        self._connect()
        if not self._use_meta_data:
            self._construct_ros_message(self._get_response_meta_data())
        self._executor.spin()
        self._disconnect()
        self.destroy_node()

    def _publisher_callback(self):
        if self._use_meta_data:
            self._communicate(True)
            self._construct_ros_message(self._get_response_meta_data())
            self._publish()
        else:
            self._communicate()
            self._bind_ros_message(self._get_receive_data())
            self._publish()

    def _construct_ros_message(self, response_meta_data_dict: Dict) -> None:
        pass

    def _bind_ros_message(self, receive_data: List[float]) -> None:
        pass

    def _publish(self) -> None:
        pass
