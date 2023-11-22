import signal
import subprocess
import threading
import unittest
from time import sleep

import rclpy
from rclpy.node import Node

from multiverse_interfaces.srv import Socket

from multiverse_socket_node import MultiverseNodeProperties
from multiverse_socket.multiverse_node import *
from multiverse_socket.multiverse_node.multiverse_node import MultiverseMetaData


def start_multiverse_server(server_port: str) -> subprocess.Popen:
    return subprocess.Popen(["multiverse_server", f"tcp://127.0.0.1:{server_port}"])


def kill_multiverse_server(process: subprocess.Popen):
    process.send_signal(signal.SIGINT)


def kill_ros_after_second(second: float):
    sleep(second)
    rclpy.shutdown()


class SocketClient(Node):
    def __init__(self):
        super().__init__("socket_client")
        self.client = self.create_client(srv_type=Socket, srv_name="/multiverse/socket")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for service socket...")

    def send_request(self, request: Socket.Request) -> Socket.Response:
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


class MultiverseRosNodePropertiesTestCase(unittest.TestCase):
    tf_publisher_props = MultiverseNodeProperties(
        "tf",
        {
            "meta_data": {
                "world_name": "world",
                "length_unit": "m",
                "angle_unit": "rad",
                "mass_unit": "kg",
                "time_unit": "s",
                "handedness": "rhs",
            },
            "host": "tcp://127.0.0.1",
            "port": "1234",
            "topic": "/tf",
            "rate": 60,
            "root_frame_id": "map",
        },
    )

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_ros_base_name(self):
        self.assertEqual(self.tf_publisher_props.ros_node_name, "Tf")

    def test_tf_subclass_creation(self):
        publisher = self.tf_publisher_props.create_publisher()
        self.assertIsInstance(publisher, TfPublisher)


class MultiverseRosNodeCreationTestCase(unittest.TestCase):
    tf_publisher_props = MultiverseNodeProperties(
        "tf",
        {
            "meta_data": {
                "world_name": "world",
                "length_unit": "m",
                "angle_unit": "rad",
                "mass_unit": "kg",
                "time_unit": "s",
                "handedness": "rhs",
            },
            "host": "tcp://127.0.0.1",
            "port": "1234",
            "topic": "/tf",
            "rate": 50,
            "root_frame_id": "map",
        },
    )

    socket_service_props = MultiverseNodeProperties(
        "socket", {"host": "tcp://127.0.0.1", "port": "2345"}
    )

    def test_tf_publisher(self):
        rclpy.init()
        server_port = "4321"
        process = start_multiverse_server(server_port)
        thread = threading.Thread(target=kill_ros_after_second, args=(1,))
        thread.start()
        TfPublisher._server_addr.port = server_port
        self.tf_publisher = self.tf_publisher_props.create_publisher()
        self.tf_publisher.run()
        thread.join()
        kill_multiverse_server(process)

    def test_socket_service(self):
        rclpy.init()
        server_port = "5432"
        process = start_multiverse_server(server_port)
        thread = threading.Thread(target=kill_ros_after_second, args=(1,))
        thread.start()
        SocketService._server_addr.port = server_port
        self.socket_service = self.socket_service_props.create_service()
        # self.socket_service.run()
        thread.join()
        kill_multiverse_server(process)

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()


class SocketServiceTestCase(unittest.TestCase):
    socket_service_props = MultiverseNodeProperties(
        "socket", {"host": "tcp://127.0.0.1", "port": "2346"}
    )

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    def test_socket_service(self):
        server_port = "6543"
        process = start_multiverse_server(server_port)
        ros_thread = threading.Thread(target=kill_ros_after_second, args=(1,))
        ros_thread.start()
        SocketService._server_addr.port = server_port
        self.socket_service = self.socket_service_props.create_service()
        service_thread = threading.Thread(target=self.socket_service.run)
        socket_client = SocketClient()
        service_thread.start()

        request = Socket.Request()
        multiverse_meta_data = MultiverseMetaData()
        request.meta_data.world_name = multiverse_meta_data.world_name
        request.meta_data.simulation_name = multiverse_meta_data.simulation_name
        request.meta_data.length_unit = multiverse_meta_data.length_unit
        request.meta_data.angle_unit = multiverse_meta_data.angle_unit
        request.meta_data.mass_unit = multiverse_meta_data.mass_unit
        request.meta_data.time_unit = multiverse_meta_data.time_unit
        request.meta_data.handedness = multiverse_meta_data.handedness
        request.send.clear()
        request.receive.clear()

        response = socket_client.send_request(request)

        socket_client.get_logger().info(f"Response: {response}")
        socket_client.destroy_node()

        ros_thread.join()
        service_thread.join()
        kill_multiverse_server(process)

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
