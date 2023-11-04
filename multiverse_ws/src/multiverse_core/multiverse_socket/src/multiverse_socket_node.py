#!/usr/bin/env python3

import rospy
import threading
import argparse
import yaml, json

class MultiverseRosSocket:
    def __init__(self) -> None:
        self.publishers = []
        self.subscribers = []
        self.services = []
        self.threads = {}

    def start(self):
        parser = argparse.ArgumentParser(description="Multiverse parser")
        parser.add_argument(
            "--multiverse_server",
            type=yaml.safe_load,
            required=False,
            help="Parameters for multiverse server",
        )
        parser.set_defaults(multiverse_server={"server_host": "tcp://127.0.0.1", "server_port": "7000"})
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
        server_host = multiverse_server["host"]
        server_port = multiverse_server["port"]

        if args.subscribers is not None:
            subscribers = json.loads(args.subscribers.replace("'", '"'))
            for subscriber_name, subscriber_props in subscribers.items():
                for subscriber_prop in subscriber_props:
                    subscriber_prop["server_host"] = server_host
                    subscriber_prop["server_port"] = server_port
                    rospy.loginfo(f"Start subscriber [{subscriber_name}] with {subscriber_prop}")

                    exec(f"from multiverse_socket.multiverse_subscribers import {subscriber_name}_subscriber")
                    subscriber = eval(f"{subscriber_name}_subscriber")(**subscriber_prop)
                    self.subscribers.append(subscriber)

                    subscriber_thread = threading.Thread(target=subscriber.start)
                    subscriber_thread.start()
                    self.threads[subscriber] = subscriber_thread

        if args.publishers is not None:
            publishers = json.loads(args.publishers.replace("'", '"'))
            for publisher_name, publisher_props in publishers.items():
                for publisher_prop in publisher_props:
                    publisher_prop["server_host"] = server_host
                    publisher_prop["server_port"] = server_port
                    rospy.loginfo(f"Start publisher [{publisher_name}] with {publisher_prop}")

                    exec(f"from multiverse_socket.multiverse_publishers import {publisher_name}_publisher")
                    publisher = eval(f"{publisher_name}_publisher")(**publisher_prop)
                    self.publishers.append(publisher)

                    publisher_thread = threading.Thread(target=publisher.start)
                    publisher_thread.start()
                    self.threads[publisher] = publisher_thread

        if args.services is not None:
            services = json.loads(args.services.replace("'", '"'))
            for service_name, service_prop in services.items():
                service_prop["server_host"] = server_host
                service_prop["server_port"] = server_port
                rospy.loginfo(f"Start service server [{service_name}] with {service_prop}")

                exec(f"from multiverse_socket.multiverse_services import {service_name}_service")
                service = eval(f"{service_name}_service")(**service_prop)
                self.services.append(service)

                service_thread = threading.Thread(target=service.start)
                service_thread.start()
                self.threads[service] = service_thread

        for subscriber in self.subscribers:
            self.threads[subscriber].join()

        for publisher in self.publishers:
            self.threads[publisher].join()

        for service in self.services:
            self.threads[service].join()


if __name__ == "__main__":
    rospy.init_node(name="multiverse_socket")
    multiverse_ros_socket = MultiverseRosSocket()
    multiverse_ros_socket.start()
