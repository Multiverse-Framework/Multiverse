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
            if publisher_prop.get('port') is None:
                rospy.logwarn(f'Ignore {publisher_name} because port is not found in rosparam')
                continue
            publisher_name += "_publisher"
            exec(f"from multiverse_client.multiverse_publishers.{publisher_name} import {publisher_name}")

            publisher = eval(publisher_name)(**publisher_prop)
            self.publishers.append(publisher)

            publisher_thread = threading.Thread(target=publisher.start)
            publisher_thread.start()
            self.threads[publisher] = publisher_thread

        subscribers = rospy.get_param("multiverse/subscribers", default={})
        for subscriber_name, subscriber_prop in subscribers.items():
            subscriber_name += "_subscriber"
            exec(f"from multiverse_client.multiverse_subscribers.{subscriber_name} import {subscriber_name}")

            subscriber = eval(subscriber_name)(**subscriber_prop)
            self.subscribers.append(subscriber)

            subscriber_thread = threading.Thread(target=subscriber.start)
            subscriber_thread.start()
            self.threads[subscriber] = subscriber_thread

        services = rospy.get_param("multiverse/services", default={})
        for service_name, service_prop in services.items():
            service_name += "_service"
            exec(f"from multiverse_client.multiverse_services.{service_name} import {service_name}")

            service = eval(service_name)(**service_prop)
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
