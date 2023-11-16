#!/usr/bin/env python3
import dataclasses
from typing import List, Dict, Optional

from multiverse_client_pybind import MultiverseClientPybind  # noqa


@dataclasses.dataclass
class SocketMetaData:
    server_host: str = "tcp://127.0.0.1"
    server_port: str = "7000"
    client_host: str = "tcp://127.0.0.1"
    client_port: Optional[str] = None


@dataclasses.dataclass
class SimulationMetaData:
    world_name: str = "world"
    simulation_name: str = "ros"
    length_unit: str = "m"
    angle_unit: str = "rad"
    mass_unit: str = "kg"
    time_unit: str = "s"
    handedness: str = "rhs"


class MultiverseRosBase:
    _request_meta_data_dict = {}

    _send_data: List[float]

    def __init__(self, socket_metadata: SocketMetaData = SocketMetaData(),
                 simulation_metadata: SimulationMetaData = SimulationMetaData()) -> None:
        if socket_metadata.client_port is None:
            raise ValueError(f"Must specify client port for {self.__class__.__name__}")
        self.server_host = socket_metadata.server_host
        self.server_port = socket_metadata.server_port
        self.client_host = socket_metadata.client_host
        self.client_port = socket_metadata.client_port
        self.simulation_metadata = simulation_metadata
        self._init_request_meta_data()

    def start(self) -> None:
        pass

    @property
    def send_data(self) -> List[float]:
        return self._send_data

    @send_data.setter
    def send_data(self, data):
        self._send_data = data

    def _init_multiverse_socket(self) -> None:
        server_socket_addr = f"{self.server_host}:{self.server_port}"
        self.__multiverse_socket = MultiverseClientPybind(server_socket_addr)

    def _init_request_meta_data(self) -> None:
        self._request_meta_data_dict = self.simulation_metadata.__dict__
        self._request_meta_data_dict["send"] = {}
        self._request_meta_data_dict["receive"] = {}

    def _connect(self) -> None:
        self.__multiverse_socket.connect(self.client_host, self.client_port)
        self.__multiverse_socket.start()

    def _disconnect(self) -> None:
        self.__multiverse_socket.disconnect()

    def _set_request_meta_data(self) -> None:
        self.__multiverse_socket.set_request_meta_data(self._request_meta_data_dict)

    def _get_response_meta_data(self) -> Dict:
        response_meta_data = self.__multiverse_socket.get_response_meta_data()
        if not response_meta_data:
            print(f"[Client {self.client_port}] Receive empty response meta data.")
        return response_meta_data

    def _set_send_data(self, send_data: List[float]) -> None:
        self.__multiverse_socket.set_send_data(send_data)

    def _communicate(self, resend_request_meta_data: bool = False) -> None:
        self.__multiverse_socket.communicate(resend_request_meta_data)

    def _get_receive_data(self) -> List[float]:
        receive_data = self.__multiverse_socket.get_receive_data()
        if not receive_data:
            print(f"[Client {self.client_port}] Receive empty data.")
        return receive_data

    def _restart(self) -> None:
        self._disconnect()
        self._connect()
