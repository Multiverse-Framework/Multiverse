#!/usr/bin/env python3

from typing import Dict

import rospy

from ..multiverse_node import MultiverseNode, MultiverseMetaData, SocketAddress


class MultiversePublisher(MultiverseNode):
    _use_meta_data: bool = False
    _publisher: rospy.Publisher
    _msg_type = None
    _msg = None

    def __init__(
            self,
            topic_name: str,
            rate: float = 60.0,
            client_addr: SocketAddress = SocketAddress(),
            multiverse_meta_data: MultiverseMetaData = MultiverseMetaData(),
            **kwargs: Dict
    ) -> None:
        super().__init__(
            client_addr=client_addr,
            multiverse_meta_data=multiverse_meta_data
        )
        self._msg = self._msg_type()
        self._publisher = rospy.Publisher(topic_name, self._msg_type, queue_size=100)
        duration_in_seconds = 1.0 / rate
        secs = int(duration_in_seconds)
        nsecs = int((duration_in_seconds - secs) * 1e9)
        rospy.Timer(
            period=rospy.Duration(secs=secs, nsecs=nsecs),
            callback=self._publisher_callback
        )

    def _run(self) -> None:
        self._connect_and_start()
        if not self._use_meta_data:
            self._bind_response_meta_data(self.response_meta_data)

    def _publisher_callback(self, _=None) -> None:
        self._communicate(self._use_meta_data)
        if self._use_meta_data:
            self._bind_response_meta_data(self.response_meta_data)
        else:
            self._bind_receive_data(self.receive_data)
        self._publish()

    def _publish(self) -> None:
        try:
            self._publisher.publish(self._msg)
        except KeyboardInterrupt:
            return
