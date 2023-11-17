#!/usr/bin/env python3

import argparse
import threading
from time import sleep
from typing import Optional, Dict, List

import json
import rclpy
import yaml

from .multiverse_ros_base.multiverse_publishers.ros_publisher import MultiverseRosPublisher
from .multiverse_ros_base.multiverse_publishers.tf_publisher import TfPublisher
from .multiverse_ros_base.multiverse_ros_base import MultiverseRosBase, SimulationMetaData


class MultiverseRosSocket:
    publishers: Optional[Dict[str, Dict]] = None
    subscribers: Optional[Dict[str, Dict]] = None
    services: Optional[Dict[str, Dict]] = None

    def __init__(
            self,
            server_host: str = "tcp://127.0.0.1",
            server_port: int = 7000,
            publishers: Optional[Dict] = None,
            subscribers: Optional[Dict] = None,
            services: Optional[Dict] = None,
    ) -> None:
        MultiverseRosBase._server_host = server_host
        MultiverseRosBase._server_port = str(server_port)
        if isinstance(services, dict):
            self.services = services
        if isinstance(subscribers, dict):
            self.subscribers = subscribers
        if isinstance(publishers, dict):
            self.publishers = publishers

    def start(self):
        threads = {}
        publisher_list: List[MultiverseRosPublisher] = []

        for publisher_name, publisher_props in self.publishers.items():
            for publisher_prop in publisher_props:
                print(f"Start publisher [{publisher_name}] with {publisher_prop}")
                simulation_metadata = SimulationMetaData(
                    world_name=publisher_prop["meta_data"]["world_name"],
                    length_unit=publisher_prop["meta_data"]["length_unit"],
                    angle_unit=publisher_prop["meta_data"]["angle_unit"],
                    mass_unit=publisher_prop["meta_data"]["mass_unit"],
                    time_unit=publisher_prop["meta_data"]["time_unit"],
                    handedness=publisher_prop["meta_data"]["handedness"],
                )
                client_host = publisher_prop["host"]
                client_port = str(publisher_prop["port"])
                node_name = f"{publisher_name}_publisher_{client_port}"
                topic_name = publisher_prop["topic"]
                rate = publisher_prop["rate"]

                if publisher_name == "tf":
                    publisher = TfPublisher(
                        root_frame_id=publisher_prop.get("root_frame_id", "map"),
                        node_name=node_name,
                        topic_name=topic_name,
                        rate=rate,
                        client_host=client_host,
                        client_port=client_port,
                        simulation_metadata=simulation_metadata,
                    )
                    publisher_list.append(publisher)

        for publisher in publisher_list:
            publisher_thread = threading.Thread(target=publisher.start)
            publisher_thread.start()
            threads[publisher] = publisher_thread

        try:
            while rclpy.ok():
                sleep(1)
        except KeyboardInterrupt:
            print(f"[{self.__class__.__name__}] Caught SIGINT (Ctrl+C), exiting...")
            rclpy.shutdown()
        finally:
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

    multiverse_server = json.loads(args.multiverse_server.replace("'", '"'))
    publishers = json.loads(args.publishers.replace("'", '"')) if args.publishers is not None else None
    subscribers = json.loads(args.subscribers.replace("'", '"')) if args.subscribers is not None else None
    services = json.loads(args.services.replace("'", '"')) if args.services is not None else None

    rclpy.init()
    multiverse_ros_socket = MultiverseRosSocket(
        server_host=multiverse_server["host"], server_port=multiverse_server["port"], publishers=publishers,
        subscribers=subscribers, services=services
    )
    multiverse_ros_socket.start()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
