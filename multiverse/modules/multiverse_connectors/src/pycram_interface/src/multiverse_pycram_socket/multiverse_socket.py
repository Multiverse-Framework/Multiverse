#!/usr/bin/env python3

import dataclasses
from typing import List, Dict, TypeVar
import logging

from multiverse_client_pybind import MultiverseClientPybind  # noqa

T = TypeVar("T")


@dataclasses.dataclass
class MultiverseMetaData:
    world_name: str = "world"
    simulation_name: str = "cram"
    length_unit: str = "m"
    angle_unit: str = "rad"
    mass_unit: str = "kg"
    time_unit: str = "s"
    handedness: str = "rhs"


class SocketAddress:
    host: str = "tcp://127.0.0.1"
    port: str = ""

    def __init__(self, port: str) -> None:
        self.port = port


class MultiverseSocket:
    _server_addr: SocketAddress = SocketAddress(port="7000")
    _client_addr: SocketAddress
    _meta_data: MultiverseMetaData
    _multiverse_socket: MultiverseClientPybind

    def __init__(
            self,
            client_addr: SocketAddress,
            multiverse_meta_data: MultiverseMetaData = MultiverseMetaData(),
    ) -> None:
        if not isinstance(client_addr.port, str) or client_addr.port == "":
            raise ValueError(f"Must specify client port for {self.__class__.__name__}")
        self._send_data = None
        self._client_addr = client_addr
        self._meta_data = multiverse_meta_data
        self._multiverse_socket = MultiverseClientPybind(
            f"{self._server_addr.host}:{self._server_addr.port}"
        )
        self.request_meta_data = {
            "meta_data": self._meta_data.__dict__,
            "send": {},
            "receive": {},
        }

    def run(self) -> None:
        message = f"[Client {self._client_addr.port}] Start {self.__class__.__name__}{self._client_addr.port}"
        logging.info(message)
        self._connect_and_start()

    def stop(self) -> None:
        self._disconnect()

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
            message = f"[Client {self._client_addr.port}] Receive empty response meta data."
            logging.warning(message)
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
            message = f"[Client {self._client_addr.port}] Receive empty data."
            logging.warning(message)
        return receive_data

    def _connect_and_start(self) -> None:
        self._multiverse_socket.connect(self._client_addr.host, self._client_addr.port)
        self._multiverse_socket.start()

    def _disconnect(self) -> None:
        self._multiverse_socket.disconnect()

    def _communicate(self, resend_request_meta_data: bool = False) -> None:
        self._multiverse_socket.communicate(resend_request_meta_data)

    def _restart(self) -> None:
        self._disconnect()
        self._connect_and_start()
