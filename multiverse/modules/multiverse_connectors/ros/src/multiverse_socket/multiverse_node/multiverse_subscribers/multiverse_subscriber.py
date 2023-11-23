#!/usr/bin/env python3

from typing import List, Dict, Any

from ..multiverse_nodes.config import USING_ROS1

if USING_ROS1:
    import rospy
    from ..multiverse_nodes.multiverse_ros1_node import MultiverseNode
else:
    from ..multiverse_nodes.multiverse_ros2_node import MultiverseNode

from ..multiverse_meta_node import SocketAddress, MultiverseMetaData


class MultiverseSubscriber(MultiverseNode):
    _msg_type = None
    _send_data: List[float] = []
    _receive_data: List[float] = []

    def __init__(
            self,
            node_name: str,
            topic_name: str,
            client_addr: SocketAddress = SocketAddress(),
            multiverse_meta_data: MultiverseMetaData = MultiverseMetaData(),
            **kwargs: Dict
    ) -> None:
        if USING_ROS1:
            super().__init__(
                client_addr=client_addr,
                multiverse_meta_data=multiverse_meta_data
            )
            self._subscriber = rospy.Subscriber(
                topic_name,
                self._msg_type,
                self._subscriber_callback,
                queue_size=100
            )
        else:
            super().__init__(
                node_name=node_name,
                client_addr=client_addr,
                multiverse_meta_data=multiverse_meta_data
            )
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
        self._communicate()

    def _init_send_data(self) -> None:
        pass
