#!/usr/bin/env python3
import argparse
import threading
from typing import Dict

import rclpy
import yaml

from .multiverse_ros_base.multiverse_publishers.ros_publisher import SocketMetaData, \
    SimulationMetaData
from .multiverse_ros_base.multiverse_publishers.tf_publisher import TfPublisher


class MultiverseRosSocket():
    server_host: str
    server_port: str
    publishers: Dict
    subscribers: Dict
    services: Dict

    def __init__(self, publishers=None, subscribers=None, services=None) -> None:
        if isinstance(services, dict):
            self.services = services
        if isinstance(subscribers, dict):
            self.subscribers = subscribers
        if isinstance(publishers, dict):
            self.publishers = publishers

    def start(self):
        threads = {}
        publisher_list = []

        for publisher_name, publisher_props in self.publishers.items():
            for publisher_prop in publisher_props:
                self.get_logger().info(f"Start publisher [{publisher_name}] with {publisher_prop}")
                socket_metadata = SocketMetaData(server_host=publisher_prop["server_host"],
                                                 server_port=publisher_prop["server_port"],
                                                 client_host=publisher_prop["host"], client_port=publisher_prop["port"])
                simulation_metadata = SimulationMetaData(world_name=publisher_prop["world_name"],
                                                         simulation_name=publisher_prop["simulation_name"],
                                                         length_unit=publisher_prop["length_unit"],
                                                         angle_unit=publisher_prop["angle_unit"],
                                                         mass_unit=publisher_prop["mass_unit"],
                                                         time_unit=publisher_prop["time_unit"],
                                                         handedness=publisher_prop["handedness"]
                                                         )
                node_name = f"{publisher_name}_publisher_{socket_metadata.client_port}"
                topic_name = publisher_prop["topic"]

                if publisher_name == "tf":
                    publisher = TfPublisher(root_frame_id=publisher_prop.get("root_frame_id", "map"),
                                            node_name=node_name, topic_name=topic_name, socket_metadata=socket_metadata,
                                            simulation_metadata=simulation_metadata)
                    publisher_list.append(publisher)

        for publisher in publisher_list:
            publisher_thread = threading.Thread(target=publisher.start)
            publisher_thread.start()
            threads[publisher] = publisher_thread

        for publisher in publisher_list:
            threads[publisher].join()


def main():
    parser = argparse.ArgumentParser(description="Multiverse parser")
    parser.add_argument(
        "--multiverse_server",
        type=yaml.safe_load,
        required=False,
        help="Parameters for multiverse server",
    )
    parser.set_defaults(multiverse_server={"host": "tcp://127.0.0.1", "port": "7000"})
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
    
    rclpy.init()
    multiverse_ros_socket = MultiverseRosSocket(publishers=args.publishers, subscribers=args.subscribers,
                                                services=args.services)
    multiverse_ros_socket.start()
    multiverse_ros_socket.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
