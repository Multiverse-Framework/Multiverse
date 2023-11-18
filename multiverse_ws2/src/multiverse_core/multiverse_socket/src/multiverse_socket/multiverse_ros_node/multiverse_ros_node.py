#!/usr/bin/env python3

import dataclasses
from typing import List, Dict
from multiverse_client_pybind import MultiverseClientPybind  # noqa


@dataclasses.dataclass
class SimulationMetaData:
    world_name: str = "world"
    simulation_name: str = "ros2"
    length_unit: str = "m"
    angle_unit: str = "rad"
    mass_unit: str = "kg"
    time_unit: str = "s"
    handedness: str = "rhs"


class MultiverseRosNode:
    _server_host: str = "tcp://127.0.0.1"
    _server_port: str = "7000"

    def __init__(self, client_host: str = "tcp://127.0.0.1", client_port: str = "",
                 simulation_metadata: SimulationMetaData = SimulationMetaData()) -> None:
        if client_port == "":
            raise ValueError(f"Must specify client port for {self.__class__.__name__}")
        self._client_host = client_host
        self._client_port = client_port
        self._simulation_metadata = simulation_metadata
        self._init_request_meta_data()

    def start(self) -> None:
        pass

    def _init_multiverse_socket(self) -> None:
        server_socket_addr = f"{self._server_host}:{self._server_port}"
        self.__multiverse_socket = MultiverseClientPybind(server_socket_addr)

    def _init_request_meta_data(self) -> None:
        self._request_meta_data_dict = {"meta_data": self._simulation_metadata.__dict__, "send": {}, "receive": {}}

    def _connect(self) -> None:
        self.__multiverse_socket.connect(self._client_host, self._client_port)
        self.__multiverse_socket.start()

    def _disconnect(self) -> None:
        self.__multiverse_socket.disconnect()

    def _set_request_meta_data(self) -> None:
        self.__multiverse_socket.set_request_meta_data(self._request_meta_data_dict)

    def _get_response_meta_data(self) -> Dict:
        response_meta_data = self.__multiverse_socket.get_response_meta_data()
        if not response_meta_data:
            print(f"[Client {self._client_port}] Receive empty response meta data.")
        return response_meta_data

    def _set_send_data(self, send_data: List[float]) -> None:
        self.__multiverse_socket.set_send_data(send_data)

    def _communicate(self, resend_request_meta_data: bool = False) -> None:
        self.__multiverse_socket.communicate(resend_request_meta_data)

    def _get_receive_data(self) -> List[float]:
        receive_data = self.__multiverse_socket.get_receive_data()
        if not receive_data:
            print(f"[Client {self._client_port}] Receive empty data.")
        return receive_data

    def _restart(self) -> None:
        self._disconnect()
        self._connect()
