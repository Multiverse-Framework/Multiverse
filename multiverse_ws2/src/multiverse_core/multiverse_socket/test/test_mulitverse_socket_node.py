import unittest

import rclpy

from multiverse_socket.multiverse_ros_base.multiverse_publishers.tf_publisher import TfPublisher
from multiverse_socket.multiverse_ros_base.multiverse_ros_base import SocketMetaData, SimulationMetaData


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
        publisher = TfPublisher(root_frame_id="map",
                                node_name="tf_publisher_", topic_name=topic_name,
                                socket_metadata=SocketMetaData(client_port=client_port),
                                simulation_metadata=SimulationMetaData())
        publisher.start()


if __name__ == '__main__':
    unittest.main()
