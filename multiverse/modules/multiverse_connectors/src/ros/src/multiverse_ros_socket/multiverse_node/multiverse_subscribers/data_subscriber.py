#!/usr/bin/env python3

from typing import Dict, Any

from std_msgs.msg import Float64

from .multiverse_subscriber import MultiverseSubscriber, SocketAddress, MultiverseMetaData


class DataSubscriber(MultiverseSubscriber):
    _body_name: str

    def __init__(
            self,
            client_addr: SocketAddress,
            topic_name: str,
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
            client_addr=client_addr,
            topic_name=topic_name,
            multiverse_meta_data=multiverse_meta_data
        )
        def bind_request_meta_data() -> None:
            self.request_meta_data["send"] = kwargs["send"]
        self.bind_request_meta_data_callback = bind_request_meta_data

    def _init_send_data(self) -> None:
        if self._msg_type == Float64:
            self.send_data = [self.world_time + self.sim_time, 0.0]
        else:
            raise NotImplementedError(f"msg_type {self._msg_type} not implemented.")

    def _bind_send_data(self, data_msg: Any) -> Any:
        if self._msg_type == Float64:
            self.send_data = [self.world_time + self.sim_time, data_msg.data]
        else:
            raise NotImplementedError(f"msg_type {self._msg_type} not implemented.")
        return data_msg
