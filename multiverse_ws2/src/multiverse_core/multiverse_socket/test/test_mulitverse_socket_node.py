import signal
import subprocess
import threading
import unittest
from time import sleep

import rclpy

from ...multiverse_socket.src.multiverse_socket.multiverse_ros_node.multiverse_publishers.tf_publisher import TfPublisher
from ..scripts.multiverse_socket_node import MultiverseRosNodeProperties

def start_multiverse_server() -> subprocess.Popen:
    return subprocess.Popen(["multiverse_server"])


def kill_multiverse_server(process: subprocess.Popen):
    process.send_signal(signal.SIGINT)


def kill_ros_after_second(second: float):
    sleep(second)
    rclpy.shutdown()


class MultiverseRosNodePropertiesTestCase(unittest.TestCase):
    tf_publisher_props = MultiverseRosNodeProperties("tf", {
        "meta_data": {"world_name": "world", "length_unit": "m", "angle_unit": "rad", "mass_unit": "kg",
                      "time_unit": "s", "handedness": "rhs"}, "host": "tcp://127.0.0.1", "port": "1234",
        "topic": "/tf",
        "rate": 60, "root_frame_id": "map"})

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_tf_publisher_name(self):
        self.assertEqual(self.tf_publisher_props.publisher_name, "TfPublisher")

    def test_tf_subclass_creation(self):
        publisher = self.tf_publisher_props.create_publisher()
        self.assertIsInstance(publisher, TfPublisher)


class MultiverseRosNodeTestCase(unittest.TestCase):
    tf_publisher_props = MultiverseRosNodeProperties("tf", {
        "meta_data": {"world_name": "world", "length_unit": "m", "angle_unit": "rad", "mass_unit": "kg",
                      "time_unit": "s", "handedness": "rhs"}, "host": "tcp://127.0.0.1", "port": "1234",
        "topic": "/tf",
        "rate": 50, "root_frame_id": "map"})

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    def test_tf_publisher(self):
        process = start_multiverse_server()
        kill_ros_thread = threading.Thread(target=kill_ros_after_second, args=(1,))
        kill_ros_thread.start()
        self.tf_publisher = self.tf_publisher_props.create_publisher()
        self.tf_publisher.start()
        kill_multiverse_server(process)

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
