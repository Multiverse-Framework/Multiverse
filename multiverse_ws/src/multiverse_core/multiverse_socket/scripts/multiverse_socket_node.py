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
        subscribers = rospy.get_param("/multiverse_client/ros/subscribers", default={})
        for subscriber_name, subscriber_props in subscribers.items():
            for subscriber_prop in subscriber_props:
                rospy.loginfo(f"Start subscriber [{subscriber_name}] with {subscriber_prop}")

                exec(f"from multiverse_socket.multiverse_subscribers import {subscriber_name}_subscriber")
                subscriber = eval(f"{subscriber_name}_subscriber")(**subscriber_prop)
                self.subscribers.append(subscriber)

                subscriber_thread = threading.Thread(target=subscriber.start)
                subscriber_thread.start()
                self.threads[subscriber] = subscriber_thread

        publishers = rospy.get_param("/multiverse_client/ros/publishers", default={})
        for publisher_name, publisher_props in publishers.items():
            for publisher_prop in publisher_props:
                rospy.loginfo(f"Start publisher [{publisher_name}] with {publisher_prop}")

                exec(f"from multiverse_socket.multiverse_publishers import {publisher_name}_publisher")
                publisher = eval(f"{publisher_name}_publisher")(**publisher_prop)
                self.publishers.append(publisher)

                publisher_thread = threading.Thread(target=publisher.start)
                publisher_thread.start()
                self.threads[publisher] = publisher_thread

        services = rospy.get_param("/multiverse_client/ros/services", default={})
        for service_name, service_prop in services.items():
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
    rospy.init_node("multiverse_socket")
    multiverse_ros_socket = MultiverseRosSocket()
    multiverse_ros_socket.start()
