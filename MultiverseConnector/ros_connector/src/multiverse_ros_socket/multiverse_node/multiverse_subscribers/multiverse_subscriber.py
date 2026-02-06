#!/usr/bin/env python3

from typing import List, Dict, Any

from ... import Interface, INTERFACE

if INTERFACE == Interface.ROS1:
    import rospy
elif INTERFACE == Interface.ROS2:
    pass
else:
    raise ValueError(f"Invalid interface {INTERFACE}")

from ..multiverse_node import MultiverseNode, MultiverseMetaData


class MultiverseSubscriber(MultiverseNode):
    _use_meta_data: bool = False
    _msg_type = None
    _send_data: List[float] = []
    _receive_data: List[float] = []

    def __init__(
            self,
            port: str,
            topic_name: str,
            multiverse_meta_data: MultiverseMetaData = MultiverseMetaData(),
            **kwargs: Dict
    ) -> None:
        super().__init__(
            port=port,
            multiverse_meta_data=multiverse_meta_data
        )
        self._create_subscriber(topic_name)

    def _create_subscriber(self, topic_name: str) -> None:
        if INTERFACE == Interface.ROS1:
            self._subscriber = rospy.Subscriber(
                name=topic_name,
                data_class=self._msg_type,
                callback=self._subscriber_callback,
                queue_size=100
            )
        elif INTERFACE == Interface.ROS2:
            self.create_subscription(
                msg_type=self._msg_type,
                topic=topic_name,
                callback=self._subscriber_callback,
                qos_profile=1,
            )

    def _run(self) -> None:
        self._connect_and_start()
        self._init_send_data()
        self._communicate()

    def _subscriber_callback(self, data: Any) -> None:
        self._bind_send_data(data)
        self._communicate(False)

    def _init_send_data(self) -> None:
        pass

    def _bind_send_data(self, data_msg: Any) -> None:
        raise NotImplementedError("Method not implemented.")
