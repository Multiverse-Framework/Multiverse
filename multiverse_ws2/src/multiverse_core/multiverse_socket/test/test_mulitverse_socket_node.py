import unittest

import rclpy

from scripts.multiverse_socket_node import MultiverseRosBaseProperties
from multiverse_socket.multiverse_ros_node.multiverse_publishers.tf_publisher import TfPublisher


class MultiverseRosBasePropertiesTestCase(unittest.TestCase):
    multiverse_ros_base = MultiverseRosBaseProperties("tf", {
        "meta_data": {"world_name": "world", "length_unit": "m", "angle_unit": "rad", "mass_unit": "kg",
                      "time_unit": "s", "handedness": "rhs"}, "host": "tcp://127.0.0.1", "port": "1234", "topic": "/tf",
        "rate": 60, "root_frame_id": "map"})

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_publisher_name(self):
        self.assertEqual(self.multiverse_ros_base.publisher_name, "TfPublisher")

    def test_subclass_creation(self):
        publisher = self.multiverse_ros_base.create_publisher()
        self.assertIsInstance(publisher, TfPublisher)


class CreationTestCase(unittest.TestCase):
    # multiverse_ros_socket: MultiverseRosSocket
    # def setUp(self):
    #    self.multiverse_ros_socket = MultiverseRosSocket()
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    @staticmethod
    def test_tf_publisher():
        client_port = "7300"
        topic_name = f"tf_publisher_{client_port}"
        publisher = TfPublisher(
            root_frame_id="map",
            node_name="tf_publisher_",
            topic_name=topic_name,
            socket_metadata=SocketMetaData(client_port=client_port),
            simulation_metadata=SimulationMetaData(),
        )
        publisher.start()


if __name__ == "__main__":
    unittest.main()
