#!/usr/bin/env python3

import dataclasses
from typing import List, Dict, TypeVar

from multiverse_client_pybind import MultiverseClientPybind  # noqa

T = TypeVar("T")


@dataclasses.dataclass
class SimulationMetaData:
    world_name: str = "world"
    simulation_name: str = "ros"
    length_unit: str = "m"
    angle_unit: str = "rad"
    mass_unit: str = "kg"
    time_unit: str = "s"
    handedness: str = "rhs"


class MultiverseRosNode:
    _server_host: str = "tcp://127.0.0.1"
    _server_port: str = "7000"
    _client_host: str
    _client_port: str
    _simulation_metadata: SimulationMetaData
    _multiverse_socket: MultiverseClientPybind
    _send_data: List[float]

    def __init__(self, client_host: str = "tcp://127.0.0.1", client_port: str = "",
                 simulation_metadata: SimulationMetaData = SimulationMetaData()) -> None:
        if client_port == "":
            raise ValueError(f"Must specify client port for {self.__class__.__name__}")
        self._client_host = client_host
        self._client_port = client_port
        self._simulation_metadata = simulation_metadata
        self._multiverse_socket = MultiverseClientPybind(f"{self._server_host}:{self._server_port}")
        self.request_meta_data = {"meta_data": self._simulation_metadata.__dict__, "send": {}, "receive": {}}

    def run(self) -> None:
        print(f"[Client {self._client_port}] Start {self.__class__.__name__}")
        self._run()

    def _run(self) -> None:
        raise NotImplementedError(f"Must implement _run() for {self.__class__.__name__}")

    @property
    def request_meta_data(self) -> Dict:
        return self._request_meta_data

    @request_meta_data.setter
    def request_meta_data(self, request_meta_data: Dict) -> None:
        self._request_meta_data = request_meta_data
        self._multiverse_socket.set_request_meta_data(self._request_meta_data)

    @property
    def response_meta_data(self) -> Dict:
        response_meta_data = self._multiverse_socket.get_response_meta_data()
        if not response_meta_data:
            print(f"[Client {self._client_port}] Receive empty response meta data.")
        return response_meta_data

    @property
    def send_data(self) -> List[float]:
        return self._send_data

    @send_data.setter
    def send_data(self, send_data: List[float]) -> None:
        self._send_data = send_data
        self._multiverse_socket.set_send_data(self._send_data)

    @property
    def receive_data(self) -> List[float]:
        receive_data = self._multiverse_socket.get_receive_data()
        if not receive_data:
            print(f"[Client {self._client_port}] Receive empty data.")
        return receive_data

    def _bind_request_meta_data(self, request_meta_data: T) -> T:
        pass

    def _bind_response_meta_data(self, response_meta_data: T) -> T:
        pass

    def _bind_send_data(self, send_data: T) -> T:
        pass

    def _bind_receive_data(self, receive_data: T) -> T:
        pass

    def _connect(self) -> None:
        self._multiverse_socket.connect(self._client_host, self._client_port)
        self._multiverse_socket.start()

    def _disconnect(self) -> None:
        self._multiverse_socket.disconnect()

    def _communicate(self, resend_request_meta_data: bool = False) -> None:
        self._multiverse_socket.communicate(resend_request_meta_data)

    def _restart(self) -> None:
        self._disconnect()
        self._connect()
