#!/usr/bin/env python3

import argparse
import json
import threading
from time import sleep
from typing import Optional, Dict, List

import rclpy
import yaml

from multiverse_socket.multiverse_ros_node.multiverse_ros_node import MultiverseRosNode, SimulationMetaData
from multiverse_socket.multiverse_ros_node.multiverse_publishers.ros_publisher import MultiverseRosPublisher
from multiverse_socket.multiverse_ros_node.multiverse_subscribers.ros_subscriber import MultiverseRosSubscriber
from multiverse_socket.multiverse_ros_node.multiverse_services.ros_service import MultiverseRosService


class MultiverseRosNodeProperties:
    ros_base_name: str
    ros_base_prop: Dict
    simulation_metadata: SimulationMetaData = SimulationMetaData()
    client_host: str
    client_port: str

    def __init__(self, ros_base_name: str, ros_base_prop: Dict):
        self.ros_base_name = ros_base_name
        self.ros_base_prop = ros_base_prop
        if "meta_data" in self.ros_base_prop:
            simulation_metadata = self.ros_base_prop.pop("meta_data")
            self.simulation_metadata = SimulationMetaData(
                world_name=simulation_metadata["world_name"] if "world_name" in simulation_metadata else "world",
                length_unit=simulation_metadata["length_unit"] if "length_unit" in simulation_metadata else "m",
                angle_unit=simulation_metadata["angle_unit"] if "angle_unit" in simulation_metadata else "rad",
                mass_unit=simulation_metadata["mass_unit"] if "mass_unit" in simulation_metadata else "kg",
                time_unit=simulation_metadata["time_unit"] if "time_unit" in simulation_metadata else "s",
                handedness=simulation_metadata["handedness"] if "handedness" in simulation_metadata else "rhs",
            )
        self.client_host = self.ros_base_prop.pop("host")
        self.client_port = str(self.ros_base_prop.pop("port"))

    @property
    def ros_base_name(self):
        return self._ros_base_name

    @ros_base_name.setter
    def ros_base_name(self, ros_base_name: str):
        ros_base_name = ros_base_name.split("_")
        ros_base_name = ''.join(part.capitalize() for part in ros_base_name)
        self._ros_base_name = ros_base_name

    def create_publisher(self):
        node_name = f"{self.ros_base_name}_publisher_{self.client_port}"
        topic_name = self.ros_base_prop.pop("topic")
        rate = self.ros_base_prop.pop("rate")
        publisher_name = f"{self.ros_base_name}Publisher"
        for subclass in MultiverseRosPublisher.__subclasses__():
            if subclass.__name__ == publisher_name:
                return subclass(node_name=node_name,
                                topic_name=topic_name,
                                rate=rate,
                                client_host=self.client_host,
                                client_port=self.client_port,
                                simulation_metadata=self.simulation_metadata,
                                **self.ros_base_prop)

        raise TypeError(f"Class {publisher_name} not found.")

    def create_subscriber(self):
        node_name = f"{self.ros_base_name}_subscriber_{self.client_port}"
        topic_name = self.ros_base_prop.pop("topic")
        subscriber_name = f"{self.ros_base_name}Subscriber"
        for subclass in MultiverseRosSubscriber.__subclasses__():
            if subclass.__name__ == subscriber_name:
                return subclass(node_name=node_name,
                                topic_name=topic_name,
                                client_host=self.client_host,
                                client_port=self.client_port,
                                simulation_metadata=self.simulation_metadata,
                                **self.ros_base_prop)

        raise TypeError(f"Class {subscriber_name} not found.")

    def create_service(self):
        node_name = f"{self.ros_base_name}_service_{self.client_port}"
        service_name = f"{self.ros_base_name}Service"
        for subclass in MultiverseRosService.__subclasses__():
            if subclass.__name__ == service_name:
                return subclass(node_name=node_name,
                                client_host=self.client_host,
                                client_port=self.client_port,
                                simulation_metadata=self.simulation_metadata,
                                **self.ros_base_prop)

        raise TypeError(f"Class {service_name} not found.")


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
        MultiverseRosNode._server_host = server_host
        MultiverseRosNode._server_port = str(server_port)
        if isinstance(publishers, dict):
            self.publishers = publishers
        if isinstance(subscribers, dict):
            self.subscribers = subscribers
        if isinstance(services, dict):
            self.services = services

    def start(self):
        threads = {}
        subscriber_list: List[MultiverseRosSubscriber] = []
        publisher_list: List[MultiverseRosPublisher] = []
        service_list: List[MultiverseRosService] = []

        for subscriber_name, subscriber_props in self.subscribers.items():
            for subscriber_prop in subscriber_props:
                print(f"Start subscriber [{subscriber_name}] with {subscriber_prop}")
                subscriber = MultiverseRosNodeProperties(ros_base_name=subscriber_name,
                                                         ros_base_prop=subscriber_prop).create_subscriber()
                subscriber_list.append(subscriber)

        for publisher_name, publisher_props in self.publishers.items():
            for publisher_prop in publisher_props:
                print(f"Start publisher [{publisher_name}] with {publisher_prop}")
                publisher = MultiverseRosNodeProperties(ros_base_name=publisher_name,
                                                        ros_base_prop=publisher_prop).create_publisher()
                publisher_list.append(publisher)

        for service_name, service_props in self.services.items():
            for service_prop in service_props:
                print(f"Start service [{service_name}] with {service_prop}")
                service = MultiverseRosNodeProperties(ros_base_name=service_name,
                                                      ros_base_prop=service_prop).create_service()
                service_list.append(service)

        for subscriber in subscriber_list:
            subscriber_thread = threading.Thread(target=subscriber.run)
            subscriber_thread.start()
            threads[subscriber] = subscriber_thread

        for publisher in publisher_list:
            publisher_thread = threading.Thread(target=publisher.run)
            publisher_thread.start()
            threads[publisher] = publisher_thread

        for service in service_list:
            service_thread = threading.Thread(target=service.run)
            service_thread.start()
            threads[service] = service_thread

        try:
            while rclpy.ok():
                # TODO: Add multiverse_server live checking
                sleep(1)
        except KeyboardInterrupt:
            print(f"[{self.__class__.__name__}] Caught SIGINT (Ctrl+C), exiting...")
            rclpy.shutdown()
        finally:
            for subscriber in subscriber_list:
                threads[subscriber].join()
            for publisher in publisher_list:
                threads[publisher].join()
            for service in service_list:
                threads[service].join()


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
