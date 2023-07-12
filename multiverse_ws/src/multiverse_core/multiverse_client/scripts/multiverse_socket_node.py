#!/usr/bin/env python3

import rospy
import sys
import threading
import os
sys.path.append(os.path.dirname(os.path.dirname(sys.argv[0])))
from multiverse_socket import MultiverseSocket  # noqa
from multiverse_ws.src.multiverse_core.multiverse_client.scripts.multiverse_client.multiverse_publishers import multiverse_ros_publisher

class MultiverseRosSocket:
    def __init__(self) -> None:
        self.publishers = []
        self.subscribers = []
        self.service_servers = []
        
    def start():
        publisher_names = rospy.get_param("multiverse/publisher", default=[])
        
        if isinstance(publisher_names, list):
            for publisher_name in publisher_names:
                publisher = eval(publisher_name + '_publisher')



def start_multiverse_publisher() -> None:
    rospy.init_node('multiverse_publisher')
    if not rospy.has_param('multiverse/publish/tf'):
        return

    threads = []
    if rospy.has_param('multiverse/publish/tf'):
        thread = threading.Thread(target=start_publish_tf)
        thread.start()
        threads.append(thread)

    for thread in threads:
        thread.join()


if __name__ == "__main__":
    MultiverseROSSocket()
    rospy.spin()
