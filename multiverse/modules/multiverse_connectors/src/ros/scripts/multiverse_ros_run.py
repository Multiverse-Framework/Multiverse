#!/usr/bin/env python3

import argparse
import dataclasses
import json
import yaml
from typing import Dict, List

from multiverse_ros_socket.multiverse_node.multiverse_node import MultiverseNode
from multiverse_ros_socket.multiverse_node.multiverse_node import Interface, INTERFACE
from multiverse_ros_socket.multiverse_node.multiverse_node_properties import (MultiverseNodeProperties,
                                                                              MultiverseSubscriber, MultiversePublisher,
                                                                              MultiverseService)

if INTERFACE == Interface.ROS1:
    import rospy
elif INTERFACE == Interface.ROS2:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
else:
    raise ValueError(f"Invalid interface {INTERFACE}")


@dataclasses.dataclass
class RosNode:
    publishers: Dict[str, Dict]
    subscribers: Dict[str, Dict]
    services: Dict[str, Dict]


class MultiverseRosSocket:
    ros_node: RosNode

    def __init__(
            self,
            host: str,
            server_port: str,
            ros_node: RosNode,
    ) -> None:
        MultiverseNode._host = host
        MultiverseNode._server_port = server_port
        self.ros_node = ros_node

    def start(self):
        if INTERFACE == Interface.ROS1:
            rospy.init_node(name="multiverse_ros_socket", anonymous=False)
        elif INTERFACE == Interface.ROS2:
            rclpy.init()

        subscriber_list: List[MultiverseSubscriber] = []
        publisher_list: List[MultiversePublisher] = []
        service_list: List[MultiverseService] = []

        if INTERFACE == Interface.ROS2:
            executor = SingleThreadedExecutor()

        for subscriber_name, subscriber_props in self.ros_node.subscribers.items():
            for subscriber_prop in subscriber_props:
                print(f"Start subscriber [{subscriber_name}] with {subscriber_prop}")
                subscriber = MultiverseNodeProperties(
                    ros_node_name=subscriber_name, ros_name_prop=subscriber_prop
                ).create_subscriber()
                if INTERFACE == Interface.ROS2:
                    executor.add_node(subscriber)
                subscriber.run()
                subscriber_list.append(subscriber)

        for publisher_name, publisher_props in self.ros_node.publishers.items():
            for publisher_prop in publisher_props:
                print(f"Start publisher [{publisher_name}] with {publisher_prop}")
                publisher = MultiverseNodeProperties(
                    ros_node_name=publisher_name, ros_name_prop=publisher_prop
                ).create_publisher()
                if INTERFACE == Interface.ROS2:
                    executor.add_node(publisher)
                publisher.run()
                publisher_list.append(publisher)

        for service_name, service_props in self.ros_node.services.items():
            for service_prop in service_props:
                print(f"Start service [{service_name}] with {service_prop}")
                service = MultiverseNodeProperties(
                    ros_node_name=service_name, ros_name_prop=service_prop
                ).create_service()
                if INTERFACE == Interface.ROS2:
                    executor.add_node(service)
                service.run()
                service_list.append(service)

        try:
            if INTERFACE == Interface.ROS1:
                rospy.spin()
            elif INTERFACE == Interface.ROS2:
                executor.spin()
        except KeyboardInterrupt:
            exit_str = f"[{self.__class__.__name__}] Caught SIGINT (Ctrl+C), exiting..."
            print(f"[ROS] {exit_str}")
            if INTERFACE == Interface.ROS1:
                if not rospy.is_shutdown():
                    rospy.signal_shutdown(exit_str)
            elif INTERFACE == Interface.ROS2:
                if rclpy.ok():
                    rclpy.shutdown()
        finally:
            for subscriber in subscriber_list:
                subscriber.stop()
            for publisher in publisher_list:
                publisher.stop()
            for service in service_list:
                service.stop()


def parse_ros_node(ros_node) -> dict:
    if isinstance(ros_node, str):
        return json.loads(ros_node.replace("'", '"'))
    elif isinstance(ros_node, dict):
        return ros_node
    else:
        return {}


def main():
    parser = argparse.ArgumentParser(description="Multiverse Ros Socket")
    parser.add_argument(
        "--host",
        type=str,
        required=False,
        default="tcp://127.0.0.1",
        help="Multiverse host",
    )
    parser.add_argument(
        "--server_port",
        type=str,
        required=False,
        default="7000",
        help="Multiverse server port",
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

    publishers = parse_ros_node(args.publishers)
    subscribers = parse_ros_node(args.subscribers)
    services = parse_ros_node(args.services)

    ros_node = RosNode(
        publishers=publishers, subscribers=subscribers, services=services
    )

    multiverse_ros_socket = MultiverseRosSocket(
        host=args.host,
        server_port=args.server_port,
        ros_node=ros_node,
    )
    multiverse_ros_socket.start()
    if INTERFACE == Interface.ROS1:
        if not rospy.is_shutdown():
            rospy.signal_shutdown("Multiverse socket shutdown")
    elif INTERFACE == Interface.ROS2:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
