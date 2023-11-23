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
        MultiverseNode.__init__(
            self, client_addr=client_addr, multiverse_meta_data=multiverse_meta_data
        )
        self.rate = rospy.Rate(rate)
        self._msg = self._msg_type()
        self._publisher = rospy.Publisher(topic_name, self._msg_type, queue_size=100)

    def _run(self) -> None:
        self._connect_and_start()
        if self._use_meta_data:
            while not rospy.is_shutdown():
                self._communicate(True)
                self._bind_response_meta_data(self.response_meta_data)
                self._publish()
                self.rate.sleep()
        else:
            self._bind_response_meta_data(self.response_meta_data)
            while not rospy.is_shutdown():
                self._communicate()
                self._bind_receive_data(self.receive_data)
                self._publish()
                self.rate.sleep()
        self._disconnect()

    def _publish(self) -> None:
        try:
            self._publisher.publish(self._msg)
        except KeyboardInterrupt:
            return
