#!/usr/bin/env python3

from typing import Dict, Any

from std_msgs.msg import Float64

from .multiverse_subscriber import MultiverseSubscriber, SocketAddress, MultiverseMetaData


class DataSubscriber(MultiverseSubscriber):
    _body_name: str

    def __init__(
            self,
            topic_name: str = "/cmd_vel",
            client_addr: SocketAddress = SocketAddress(),
            multiverse_meta_data: MultiverseMetaData = MultiverseMetaData(),
            **kwargs: Dict
    ) -> None:
        if "send" not in kwargs:
            raise ValueError("Send not found.")
        if not isinstance(kwargs["send"], dict):
            raise ValueError("Send must be dict.")
        msg_type = str(kwargs["msg_type"])
        if msg_type == "std_msgs/Float64":
            self._msg_type = Float64
        else:
            raise NotImplementedError(f"msg_type {msg_type} not implemented.")
        super().__init__(
            topic_name=topic_name,
            client_addr=client_addr,
            multiverse_meta_data=multiverse_meta_data
        )
        self.request_meta_data["send"] = kwargs["send"]

    def _init_send_data(self) -> None:
        if self._msg_type == Float64:
            self.send_data = [0.0] * 2
        else:
            raise NotImplementedError(f"msg_type {self._msg_type} not implemented.")

    def _bind_send_data(self, data_msg: Any) -> Any:
        if self._msg_type == Float64:
            self.send_data[1:] = [data_msg.data]
        else:
            raise NotImplementedError(f"msg_type {self._msg_type} not implemented.")
        return data_msg
