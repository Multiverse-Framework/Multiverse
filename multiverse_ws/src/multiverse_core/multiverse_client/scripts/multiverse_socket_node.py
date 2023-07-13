#!/usr/bin/env python3

import rospy
import threading


class MultiverseRosSocket:
    def __init__(self) -> None:
        self.publishers = []
        self.subscribers = []
        self.services = []
        self.threads = {}

    def start(self):
        publishers = rospy.get_param("multiverse/publishers", default={})
        for publisher_name, publisher_prop in publishers.items():
            publisher_name += '_publisher'
            publisher_prop: dict
            host = str(publisher_prop.get('host', 'tcp://127.0.0.1'))
            port = str(publisher_prop.get('port'))
            exec(
                f'from multiverse_client.multiverse_publishers.{publisher_name} import {publisher_name}')

            publisher = eval(publisher_name)(host, port)
            self.publishers.append(publisher)

            publisher_thread = threading.Thread(target=publisher.start)
            publisher_thread.start()
            self.threads[publisher] = publisher_thread

        services = rospy.get_param("multiverse/services", default={})
        for service_name, service_prop in services.items():
            service_name += '_service'
            service_prop: dict
            host = str(service_prop.get('host', 'tcp://127.0.0.1'))
            port = str(service_prop.get('port'))
            exec(
                f'from multiverse_client.multiverse_services.{service_name} import {service_name}')

            service = eval(service_name)(host, port)
            self.services.append(service)

            service_thread = threading.Thread(target=service.start)
            service_thread.start()
            self.threads[service] = service_thread

        for publisher in self.publishers:
            self.threads[publisher].join()

        for service in self.services:
            self.threads[service].join()

if __name__ == "__main__":
    rospy.init_node("multiverse_socket")
    multiverse_ros_socket = MultiverseRosSocket()
    multiverse_ros_socket.start()
