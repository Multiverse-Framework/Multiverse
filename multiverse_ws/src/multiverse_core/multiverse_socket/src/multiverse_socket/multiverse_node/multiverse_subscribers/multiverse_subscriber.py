#!/usr/bin/env python3

from typing import List, Dict, Any

import rospy

from ..multiverse_node import MultiverseNode, MultiverseMetaData, SocketAddress


class MultiverseSubscriber(MultiverseNode):
    _msg_type = None
    _send_data: List[float] = []
    _receive_data: List[float] = []

    def __init__(
        self,
        topic_name: str,
        client_addr: SocketAddress = SocketAddress(),
        multiverse_meta_data: MultiverseMetaData = MultiverseMetaData(),
        **kwargs: Dict
    ) -> None:
        MultiverseNode.__init__(
            self, client_addr=client_addr, multiverse_meta_data=multiverse_meta_data
        )
        self._subscriber = rospy.Subscriber(
            topic_name, self._msg_type, self._subscriber_callback, queue_size=100
        )

    def _run(self) -> None:
        self._connect()
        self._init_send_data()
        self._communicate()
        while not rospy.is_shutdown():
            if len(self.receive_data) > 0:
                break
        rospy.spin()
        self._disconnect()

    def _subscriber_callback(self, data: Any) -> None:
        self._bind_send_data(data)
        self._communicate()

    def _init_send_data(self) -> None:
        pass
