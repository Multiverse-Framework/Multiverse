#!/usr/bin/env python3

from typing import Dict, TypeVar

from ... import Interface, INTERFACE

if INTERFACE == Interface.ROS1:
    import rospy
elif INTERFACE == Interface.ROS2:
    pass
else:
    raise ValueError(f"Invalid interface {INTERFACE}")

from ..multiverse_node import MultiverseNode, MultiverseMetaData

SrvType = TypeVar("SrvType")
SrvTypeRequest = TypeVar(f"SrvTypeRequest")
SrvTypeResponse = TypeVar("SrvTypeResponse")


class MultiverseService(MultiverseNode):
    _srv_name = ""
    _srv_type: SrvType
    _srv_request_class: SrvTypeRequest
    _srv_response_class: SrvTypeResponse

    def __init__(
            self,
            port: str,
            multiverse_meta_data: MultiverseMetaData = MultiverseMetaData(),
            **kwargs: Dict
    ) -> None:
        super().__init__(
            port=port,
            multiverse_meta_data=multiverse_meta_data
        )
        if INTERFACE == Interface.ROS1:
            rospy.Service(
                name=self._srv_name,
                service_class=self._srv_type,
                handler=self._service_handler,
            )
        elif INTERFACE == Interface.ROS2:
            self.create_service(
                srv_type=self._srv_type,
                srv_name=self._srv_name,
                callback=self._service_callback,
            )

    def _run(self) -> None:
        self._connect_and_start()

    def _service_callback(self, request: SrvTypeRequest, response: SrvTypeResponse) -> SrvTypeResponse:
        self._bind_request_meta_data(request)
        self._communicate(True)
        return self._bind_response_meta_data(response)

    def _service_handler(self, request: SrvTypeRequest) -> SrvTypeResponse:
        return self._service_callback(request, self._srv_response_class())

    def _bind_request_meta_data(self, request: SrvTypeRequest) -> SrvTypeRequest:
        raise NotImplementedError("Method not implemented.")

    def _bind_response_meta_data(self, response: SrvTypeResponse) -> SrvTypeResponse:
        raise NotImplementedError("Method not implemented.")
