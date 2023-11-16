#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from typing import List, Dict
from ..multiverse_ros_base import MultiverseRosBase, SimulationMetaData, SocketMetaData


class MultiverseRosPublisher(MultiverseRosBase, Node):
    _use_meta_data: bool = False
    _topic_name: str = ""

    def __init__(self, topic_name: str, node_name: str, socket_metadata: SocketMetaData = SocketMetaData(),
                 simulation_metadata: SimulationMetaData = SimulationMetaData()):
        MultiverseRosBase.__init__(self, socket_metadata=socket_metadata, simulation_metadata=simulation_metadata)
        Node.__init__(self, node_name=node_name)
        self._topic_name = topic_name

    def start(self) -> None:
        self._init_multiverse_socket()
        self._set_request_meta_data()
        self._connect()
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            self._disconnect()
        finally:
            self.destroy_node()

    def _publisher_callback(self):
        if self._use_meta_data:
            while rclpy.ok():
                self._communicate(True)
                self._construct_ros_message(self._get_response_meta_data())
                if rclpy.ok():
                    self._publish()

        else:
            self._construct_ros_message(self._get_response_meta_data())
            while rclpy.ok():
                self._communicate()
                self._bind_ros_message(self._get_receive_data())
                if rclpy.ok():
                    self._publish()
        pass

    def _construct_ros_message(self, response_meta_data_dict: Dict) -> None:
        pass

    def _bind_ros_message(self, receive_data: List[float]) -> None:
        pass

    def _publish(self) -> None:
        pass
