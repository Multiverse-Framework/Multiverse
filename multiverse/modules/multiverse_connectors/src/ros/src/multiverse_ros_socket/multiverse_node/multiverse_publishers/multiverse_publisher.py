#!/usr/bin/env python3

from typing import Dict, Any, List, Optional

from ... import Interface, INTERFACE

if INTERFACE == Interface.ROS1:
    import rospy
    from rospy import Publisher
elif INTERFACE == Interface.ROS2:
    from rclpy.publisher import Publisher
else:
    raise ValueError(f"Invalid interface {INTERFACE}")

from ..multiverse_node import MultiverseNode, SocketAddress, MultiverseMetaData

class MultiversePublisher(MultiverseNode):
    _use_meta_data: bool = False
    _msg_types: List[Any] = []
    _msgs: List[Any] = []
    __publishers: List[Publisher] = []

    def __init__(
            self,
            client_addr: SocketAddress,
            topic_name: str | List[str],
            rate: float = 60.0,
            multiverse_meta_data: MultiverseMetaData = MultiverseMetaData(),
            **kwargs: Dict
    ) -> None:
        super().__init__(
            client_addr=client_addr,
            multiverse_meta_data=multiverse_meta_data
        )
        if isinstance(topic_name, str):
            self.create_publishers([topic_name], [self._msg_type], rate)
        elif isinstance(topic_name, list):
            self.create_publishers(topic_name, self._msg_types, rate)

    def create_publishers(self, topic_names: List[str], msg_types: List[Any], rate: float) -> None:
        if INTERFACE == Interface.ROS1:
            for topic_name, msg_type in zip(topic_names, msg_types):
                self._msgs.append(msg_type())
                self.__publishers.append(rospy.Publisher(topic_name, msg_type, queue_size=100))
            duration_in_seconds = 1.0 / rate
            secs = int(duration_in_seconds)
            nsecs = int((duration_in_seconds - secs) * 1e9)
            rospy.Timer(
                period=rospy.Duration(secs=secs, nsecs=nsecs),
                callback=self._publisher_callback
            )
        elif INTERFACE == Interface.ROS2:
            for topic_name, msg_type in zip(topic_names, msg_types):
                self._msgs.append(msg_type())
                self.__publishers.append(self.create_publisher(msg_type, topic_name, 100))
            self.create_timer(
                timer_period_sec=1.0 / rate,
                callback=self._publisher_callback
            )
        else:
            raise ValueError(f"Invalid interface {INTERFACE}")

    def _run(self) -> None:
        self._connect_and_start()
        if not self._use_meta_data:
            self._bind_response_meta_data(self.response_meta_data)

    def _publisher_callback(self, _=None) -> None:
        try:
            self._communicate(self._use_meta_data)
            if self._use_meta_data:
                self._bind_response_meta_data(self.response_meta_data)
            else:
                self.send_data = [self.world_time + self.sim_time]
                self._bind_receive_data(self.receive_data)
            self._publish()
        except:
            return

    def _publish(self) -> None:
        for msg, publisher in zip(self._msgs, self.__publishers):
            publisher.publish(msg)
