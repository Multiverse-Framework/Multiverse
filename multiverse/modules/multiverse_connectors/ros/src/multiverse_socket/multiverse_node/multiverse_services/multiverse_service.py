#!/usr/bin/env python3

from typing import Dict, Any, TypeVar

from ..multiverse_nodes.config import USING_ROS1

if USING_ROS1:
    import rospy
    from ..multiverse_nodes.multiverse_ros1_node import MultiverseNode
else:
    from ..multiverse_nodes.multiverse_ros2_node import MultiverseNode

from ..multiverse_meta_node import SocketAddress, MultiverseMetaData

Request = TypeVar("Request")
Response = TypeVar("Response")


class MultiverseService(MultiverseNode):
    _srv_name = ""
    _srv_class: Any
    _srv_request_class: Request
    _srv_response_class: Response

    def __init__(
            self,
            node_name: str,
            client_addr: SocketAddress = SocketAddress(),
            multiverse_meta_data: MultiverseMetaData = MultiverseMetaData(),
            **kwargs: Dict
    ) -> None:
        if USING_ROS1:
            super().__init__(
                client_addr=client_addr,
                multiverse_meta_data=multiverse_meta_data
            )
            rospy.Service(
                name=self._srv_name,
                service_class=self._srv_class,
                handler=self._service_handler,
            )
        else:
            super().__init__(
                node_name=node_name,
                client_addr=client_addr,
                multiverse_meta_data=multiverse_meta_data
            )
            self.create_service(
                srv_type=self._srv_class,
                srv_name=self._srv_name,
                callback=self._service_handler,
            )

    def _run(self) -> None:
        self._connect_and_start()

    def _service_callback(self, request: Request, response: Response) -> Response:
        self._bind_request_meta_data(request)
        self._communicate(True)
        return self._bind_response_meta_data(response)

    def _service_handler(self, request: Request) -> Response:
        return self._service_callback(request, self._srv_response_class())
