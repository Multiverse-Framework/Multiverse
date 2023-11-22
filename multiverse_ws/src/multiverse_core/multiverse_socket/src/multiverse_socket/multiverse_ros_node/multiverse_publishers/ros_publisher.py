#!/usr/bin/env python3

from typing import Dict

import rospy

from ..multiverse_ros_node import MultiverseRosNode, SimulationMetaData


class MultiverseRosPublisher(MultiverseRosNode):
    _publisher: rospy.Publisher
    _msg_type = None
    _use_meta_data: bool = False

    def __init__(
            self,
            topic_name: str,
            rate: float = 60.0,
            client_host: str = "tcp://127.0.0.1",
            client_port: str = "",
            simulation_metadata: SimulationMetaData = SimulationMetaData(),
            **kwargs: Dict
    ) -> None:
        MultiverseRosNode.__init__(self, client_host=client_host, client_port=client_port,
                                   simulation_metadata=simulation_metadata)
        self.rate = rospy.Rate(rate)
        self._publisher = rospy.Publisher(topic_name, self._msg_type, queue_size=100)

    def _run(self) -> None:
        self._connect()
        if self._use_meta_data:
            while not rospy.is_shutdown():
                self._communicate(True)
                self._bind_response_meta_data(self.response_meta_data)
                if not rospy.is_shutdown():
                    self._publish()
                self.rate.sleep()
        else:
            self._bind_response_meta_data(self.response_meta_data)
            while not rospy.is_shutdown():
                self._communicate()
                self._bind_receive_data(self.receive_data)
                if not rospy.is_shutdown():
                    self._publish()
                self.rate.sleep()
        self._disconnect()

    def _publish(self) -> None:
        pass
