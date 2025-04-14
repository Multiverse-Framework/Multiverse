#!/usr/bin/env python3

"""Multiverse Client base class."""

import dataclasses
from typing import List, Dict, Callable, TypeVar

from multiverse_client_pybind import MultiverseClientPybind  # noqa

T = TypeVar("T")


@dataclasses.dataclass
class MultiverseMetaData:
    """Meta data for the Multiverse Client, the simulation_name should be non-empty and unique for each simulation"""

    world_name: str = "world"
    simulation_name: str = ""
    length_unit: str = "m"
    angle_unit: str = "rad"
    mass_unit: str = "kg"
    time_unit: str = "s"
    handedness: str = "rhs"


class MultiverseClient:
    """Base class for the Multiverse Client"""

    _host: str = "tcp://127.0.0.1"
    _server_port: str = "7000"
    _client_port: str
    _meta_data: MultiverseMetaData
    _multiverse_socket: MultiverseClientPybind
    _start_time: float
    _api_callbacks: Dict[str, Callable]
    _api_callbacks_response: Dict[str, Callable]
    _bind_request_meta_data_callback: Callable
    _bind_response_meta_data_callback: Callable
    _bind_send_data_callback: Callable
    _bind_receive_data_callback: Callable
    _init_objects_callback: Callable
    _reset_callback: Callable

    def __init__(
        self,
        port: str,
        multiverse_meta_data: MultiverseMetaData,
    ) -> None:
        """

        Args:
            port: The client port.
            multiverse_meta_data: The meta data for the Multiverse Client.
        """
        self._client_port = port
        if multiverse_meta_data.simulation_name == "":
            raise ValueError(f"Must specify simulation name.")
        self._send_data = None
        self._meta_data = multiverse_meta_data
        self._multiverse_socket = MultiverseClientPybind()
        self.request_meta_data = {
            "meta_data": self._meta_data.__dict__,
            "send": {},
            "receive": {},
        }
        self._start_time = 0.0

    def loginfo(self, message: str) -> None:
        """Log information.

        Args:
            message: The message to log.
        """
        raise NotImplementedError(
            f"Must implement loginfo() for {self.__class__.__name__}."
        )

    def logwarn(self, message: str) -> None:
        """Warn the user.

        Args:
            message: The message to warn about.
        """
        raise NotImplementedError(
            f"Must implement logwarn() for {self.__class__.__name__}."
        )

    def run(self) -> None:
        """Run the client. This method will call the _run() method."""
        message = f"[Client {self._client_port}] Start {self.__class__.__name__}{self._client_port}."
        self.loginfo(message)
        self._run()

    def _run(self) -> None:
        """Run the client, should call the _connect_and_start() method. It's left to the user to implement this method
        in threaded or non-threaded fashion.
        """
        raise NotImplementedError(
            f"Must implement _run() for {self.__class__.__name__}."
        )

    def stop(self) -> None:
        """Stop the client."""
        self._disconnect()

    @property
    def request_meta_data(self) -> Dict:
        """Get the request meta data, which is set by the user."""
        return self._request_meta_data

    @request_meta_data.setter
    def request_meta_data(self, request_meta_data: Dict) -> None:
        """Set the request_meta_data, make sure to clear the `send` and `receive` field before setting the request"""
        self._request_meta_data = request_meta_data
        self._multiverse_socket.set_request_meta_data(self._request_meta_data)

    @property
    def response_meta_data(self) -> Dict:
        """Get the response_meta_data, which is received from the server."""
        response_meta_data = self._multiverse_socket.get_response_meta_data()
        assert isinstance(response_meta_data, dict)
        if response_meta_data == {}:
            message = (
                f"[Client {self._client_port}] Receive empty response meta data."
            )
            self.logwarn(message)
        return response_meta_data

    @property
    def send_data(self) -> List[float]:
        """Get the send_data, which is set by the user.
        The first element should be the current simulation time,
        the rest should be the data to send with the following order:
        double -> uint8_t -> uint16_t"""
        return self._send_data

    @send_data.setter
    def send_data(self, send_data: List[float]) -> None:
        """Set the send_data, which is received from the server.
        The first element should be the current simulation time,
        the rest should be the data to send with the following order:
        double -> uint8_t -> uint16_t"""
        assert isinstance(send_data, list)
        self._send_data = send_data
        self._multiverse_socket.set_send_data(self._send_data)

    @property
    def receive_data(self) -> List[float]:
        """Get the receive_data, the first element should be the current world time,
        the rest should be the received data with the following order:
        double -> uint8_t -> uint16_t"""
        receive_data = self._multiverse_socket.get_receive_data()
        assert isinstance(receive_data, list)
        return receive_data

    @property
    def api_callbacks(self) -> Dict[str, Callable]:
        """Get the api_callbacks."""
        return self._api_callbacks

    @api_callbacks.setter
    def api_callbacks(
        self, api_callbacks: Dict[str, Callable]
    ) -> None:
        """Set the api_callbacks."""
        self._multiverse_socket.set_api_callbacks(api_callbacks)
        self._api_callbacks = api_callbacks

    @property
    def api_callbacks_response(self) -> Dict[str, Callable]:
        """Get the api_callbacks_response."""
        return self._api_callbacks_response
    
    @api_callbacks_response.setter
    def api_callbacks_response(self, api_callbacks_response: Dict[str, Callable]) -> None:
        """Set the api_callbacks_response."""
        self._multiverse_socket.set_api_callbacks_response(api_callbacks_response)
        self._api_callbacks_response = api_callbacks_response

    @property
    def bind_request_meta_data_callback(self) -> Callable:
        """Get the bind_request_meta_data_callback."""
        return self._bind_request_meta_data_callback

    @bind_request_meta_data_callback.setter
    def bind_request_meta_data_callback(self, bind_request_meta_data_callback: Callable) -> None:
        """Set the bind_request_meta_data_callback."""
        self._multiverse_socket.set_bind_request_meta_data_callback(bind_request_meta_data_callback)
        self._bind_request_meta_data_callback = bind_request_meta_data_callback

    @property
    def bind_response_meta_data_callback(self) -> Callable:
        """Get the bind_response_meta_data_callback."""
        return self._bind_response_meta_data_callback

    @bind_response_meta_data_callback.setter
    def bind_response_meta_data_callback(self, bind_response_meta_data_callback: Callable) -> None:
        """Set the bind_response_meta_data_callback."""
        self._multiverse_socket.set_bind_response_meta_data_callback(bind_response_meta_data_callback)
        self._bind_response_meta_data_callback = bind_response_meta_data_callback

    @property
    def bind_send_data_callback(self) -> Callable:
        """Get the bind_send_data_callback."""
        return self._bind_send_data_callback

    @bind_send_data_callback.setter
    def bind_send_data_callback(self, bind_send_data_callback: Callable) -> None:
        """Set the bind_send_data_callback."""
        self._multiverse_socket.set_bind_send_data_callback(bind_send_data_callback)
        self._bind_send_data_callback = bind_send_data_callback

    @property
    def bind_receive_data_callback(self) -> Callable:
        """Get the bind_receive_data_callback."""
        return self._bind_receive_data_callback
    
    @bind_receive_data_callback.setter
    def bind_receive_data_callback(self, bind_receive_data_callback: Callable) -> None:
        """Set the bind_receive_data_callback."""
        self._multiverse_socket.set_bind_receive_data_callback(bind_receive_data_callback)
        self._bind_receive_data_callback = bind_receive_data_callback

    @property
    def init_objects_callback(self) -> Callable:
        """Get the init_objects_callback."""
        return self._init_objects_callback
    
    @init_objects_callback.setter
    def init_objects_callback(self, init_objects_callback: Callable) -> None:
        """Set the init_objects_callback."""
        self._multiverse_socket.set_init_objects_callback(init_objects_callback)
        self._init_objects_callback = init_objects_callback

    @property
    def reset_callback(self) -> Callable:
        """Get the reset_callback."""
        return self._reset_callback

    @reset_callback.setter
    def reset_callback(self, reset_callback: Callable) -> None:
        """Set the reset_callback."""
        self._multiverse_socket.set_reset_callback(reset_callback)
        self._reset_callback = reset_callback

    def _connect_and_start(self) -> None:
        """Connect to the server and start the client."""
        self._multiverse_socket.connect(self._host, self._server_port, self._client_port)
        self._multiverse_socket.start()
        self._start_time = self._multiverse_socket.get_time_now()

    def _disconnect(self) -> None:
        """Disconnect from the server."""
        self._multiverse_socket.disconnect()

    def _communicate(self, resend_request_meta_data: bool = False) -> bool:
        """Communicate with the server. Return True if successful, False otherwise.

        Args:
            resend_request_meta_data: Resend the request meta data.
        """
        return self._multiverse_socket.communicate(resend_request_meta_data)

    def _restart(self) -> None:
        """Restart the client."""
        self._disconnect()
        self._connect_and_start()

    @property
    def world_time(self) -> float:
        """Get the current world time from the server in the time unit specified in
        multiverse_client_py.multiverse_client.MultiverseMetaData.time_unit."""
        return self._multiverse_socket.get_world_time()

    @property
    def sim_time(self) -> float:
        """Get the current simulation time in the time unit specified in
        multiverse_client_py.multiverse_client.MultiverseMetaData.time_unit."""
        return self._multiverse_socket.get_time_now() - self._start_time
