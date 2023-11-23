#!/usr/bin/env python3

import argparse
import dataclasses
import json
import yaml
from typing import Dict, List

import rospy

from multiverse_socket.multiverse_node.multiverse_node import (
    MultiverseNode,
    MultiverseMetaData,
    SocketAddress,
)
from multiverse_socket.multiverse_node.multiverse_publishers.multiverse_publisher import (
    MultiversePublisher,
)
from multiverse_socket.multiverse_node.multiverse_services.multiverse_service import (
    MultiverseService,
)
from multiverse_socket.multiverse_node.multiverse_subscribers.multiverse_subscriber import (
    MultiverseSubscriber,
)


class MultiverseNodeProperties:
    ros_node_name: str
    ros_node_prop: Dict
    multiverse_meta_data: MultiverseMetaData = MultiverseMetaData()
    client_addr: SocketAddress

    def __init__(self, ros_node_name: str, ros_name_prop: Dict):
        self.ros_node_name = ros_node_name
        self.ros_node_prop = ros_name_prop
        if "meta_data" in self.ros_node_prop:
            meta_data = self.ros_node_prop.pop("meta_data")
            self.multiverse_meta_data = MultiverseMetaData(
                world_name=meta_data.get("world_name", "world"),
                length_unit=meta_data.get("length_unit", "m"),
                angle_unit=meta_data.get("angle_unit", "rad"),
                mass_unit=meta_data.get("mass_unit", "kg"),
                time_unit=meta_data.get("time_unit", "s"),
                handedness=meta_data.get("handedness", "rhs")
            )
        self.client_addr = SocketAddress(
            host=self.ros_node_prop.pop("host"),
            port=str(self.ros_node_prop.pop("port")),
        )

    @property
    def ros_node_name(self):
        return self._ros_node_name

    @ros_node_name.setter
    def ros_node_name(self, ros_node_name: str):
        ros_node_name = ros_node_name.split("_")
        ros_node_name = "".join(part.capitalize() for part in ros_node_name)
        self._ros_node_name = ros_node_name

    def create_publisher(self):
        topic_name = self.ros_node_prop.pop("topic")
        rate = self.ros_node_prop.pop("rate")
        publisher_name = f"{self.ros_node_name}Publisher"
        for subclass in MultiversePublisher.__subclasses__():
            if subclass.__name__ == publisher_name:
                return subclass(
                    topic_name=topic_name,
                    rate=rate,
                    client_addr=self.client_addr,
                    multiverse_meta_data=self.multiverse_meta_data,
                    **self.ros_node_prop,
                )

        raise TypeError(f"Class {publisher_name} not found.")

    def create_subscriber(self):
        topic_name = self.ros_node_prop.pop("topic")
        subscriber_name = f"{self.ros_node_name}Subscriber"
        for subclass in MultiverseSubscriber.__subclasses__():
            if subclass.__name__ == subscriber_name:
                return subclass(
                    topic_name=topic_name,
                    client_addr=self.client_addr,
                    multiverse_meta_data=self.multiverse_meta_data,
                    **self.ros_node_prop,
                )

        raise TypeError(f"Class {subscriber_name} not found.")

    def create_service(self):
        service_name = f"{self.ros_node_name}Service"
        for subclass in MultiverseService.__subclasses__():
            if subclass.__name__ == service_name:
                return subclass(
                    client_addr=self.client_addr,
                    multiverse_meta_data=self.multiverse_meta_data,
                    **self.ros_node_prop,
                )

        raise TypeError(f"Class {service_name} not found.")


@dataclasses.dataclass
class RosNode:
    publishers: Dict[str, Dict]
    subscribers: Dict[str, Dict]
    services: Dict[str, Dict]


class MultiverseRosSocket:
    ros_node: RosNode

    def __init__(
            self,
            server_addr: SocketAddress,
            ros_node: RosNode,
    ) -> None:
        MultiverseNode._server_addr = server_addr
        self.ros_node = ros_node

    def start(self):
        rospy.init_node(name="multiverse_ros_socket", anonymous=False)

        subscriber_list: List[MultiverseSubscriber] = []
        publisher_list: List[MultiversePublisher] = []
        service_list: List[MultiverseService] = []

        for subscriber_name, subscriber_props in self.ros_node.subscribers.items():
            for subscriber_prop in subscriber_props:
                print(f"Start subscriber [{subscriber_name}] with {subscriber_prop}")
                subscriber = MultiverseNodeProperties(
                    ros_node_name=subscriber_name, ros_name_prop=subscriber_prop
                ).create_subscriber()
                subscriber.run()
                subscriber_list.append(subscriber)

        for publisher_name, publisher_props in self.ros_node.publishers.items():
            for publisher_prop in publisher_props:
                print(f"Start publisher [{publisher_name}] with {publisher_prop}")
                publisher = MultiverseNodeProperties(
                    ros_node_name=publisher_name, ros_name_prop=publisher_prop
                ).create_publisher()
                publisher.run()
                publisher_list.append(publisher)

        for service_name, service_props in self.ros_node.services.items():
            for service_prop in service_props:
                print(f"Start service [{service_name}] with {service_prop}")
                service = MultiverseNodeProperties(
                    ros_node_name=service_name, ros_name_prop=service_prop
                ).create_service()
                service.run()
                service_list.append(service)

        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.signal_shutdown(
                f"[{self.__class__.__name__}] Caught SIGINT (Ctrl+C), exiting..."
            )
        finally:
            for subscriber in subscriber_list:
                subscriber.stop()
            for publisher in publisher_list:
                publisher.stop()
            for service in service_list:
                service.stop()


def parse_ros_node_from_str(ros_node_str: str) -> dict:
    return json.loads(ros_node_str.replace("'", '"')) if isinstance(ros_node_str, str) else {}


def main():
    parser = argparse.ArgumentParser(description="Multiverse parser")
    parser.add_argument(
        "--multiverse_server",
        type=yaml.safe_load,
        required=False,
        help="Parameters for multiverse server",
    )
    parser.add_argument(
        "--services",
        type=yaml.safe_load,
        required=False,
        help="Parameters for ROS services",
    )
    parser.add_argument(
        "--publishers",
        type=yaml.safe_load,
        required=False,
        help="Parameters for ROS publishers",
    )
    parser.add_argument(
        "--subscribers",
        type=yaml.safe_load,
        required=False,
        help="Parameters for ROS subscribers",
    )
    args = parser.parse_args()

    multiverse_server = (
        json.loads(args.multiverse_server.replace("'", '"'))
        if isinstance(args.multiverse_server, str)
        else {"host": "tcp://127.0.0.1", "port": 7000}
    )
    publishers = parse_ros_node_from_str(args.publishers)
    subscribers = parse_ros_node_from_str(args.subscribers)
    services = parse_ros_node_from_str(args.services)

    socket_addr = SocketAddress(
        host=multiverse_server["host"], port=str(multiverse_server["port"])
    )
    ros_node = RosNode(
        publishers=publishers, subscribers=subscribers, services=services
    )

    multiverse_ros_socket = MultiverseRosSocket(
        server_addr=socket_addr,
        ros_node=ros_node,
    )
    multiverse_ros_socket.start()
    if not rospy.is_shutdown():
        rospy.signal_shutdown("Multiverse socket shutdown")


if __name__ == "__main__":
    main()
