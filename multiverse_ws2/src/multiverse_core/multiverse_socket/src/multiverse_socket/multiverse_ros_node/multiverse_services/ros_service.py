#!/usr/bin/env python3

from typing import Dict

from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node, MsgType, SrvTypeRequest, SrvTypeResponse

from ..multiverse_ros_node import MultiverseRosNode, SimulationMetaData


class MultiverseRosService(MultiverseRosNode, Node):
    _srv_name = ""
    _srv_type: MsgType = None

    def __init__(
            self,
            node_name: str,
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
        self.create_service(srv_type=self._srv_type, srv_name=self._srv_name, callback=self._service_callback)

    def _run(self) -> None:
        self._connect()
        self._executor.spin()
        self._disconnect()
        self.destroy_node()

    def _service_callback(self, request: SrvTypeRequest, response: SrvTypeResponse) -> SrvTypeResponse:
        self._bind_request_meta_data(request)
        self._communicate(True)
        return self._bind_response_meta_data(response)
