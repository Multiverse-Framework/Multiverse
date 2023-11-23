import signal
import subprocess
import threading
import unittest
from time import sleep
from typing import Any

from multiverse_socket.multiverse_node.multiverse_nodes.config import USING_ROS1

if USING_ROS1:
    import rospy
    from multiverse_msgs.srv import Socket, SocketRequest
else:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from multiverse_interfaces.srv import Socket

from multiverse_socket_node import MultiverseNodeProperties
from multiverse_socket.multiverse_node.multiverse_meta_node import MultiverseMetaData


def init_node() -> None:
    if USING_ROS1:
        rospy.init_node("test_multiverse_ros_node_properties")
    else:
        rclpy.init()


def spin_node(node: Any) -> None:
    if USING_ROS1:
        rospy.spin()
    else:
        rclpy.spin(node)


def destroy_node() -> None:
    if USING_ROS1:
        if not rospy.is_shutdown():
            rospy.signal_shutdown("Multiverse socket shutdown")
    else:
        if rclpy.ok():
            rclpy.shutdown()


def start_multiverse_server(server_port: str) -> subprocess.Popen:
    return subprocess.Popen(["multiverse_server", f"tcp://127.0.0.1:{server_port}"])


def kill_multiverse_server(process: subprocess.Popen):
    process.send_signal(signal.SIGINT)
    process.wait()


def kill_ros_after_second(second: float):
    sleep(second)
    rclpy.shutdown()


class SocketClient(object if USING_ROS1 else Node):
    def __init__(self):
        if not USING_ROS1:
            super().__init__("socket_client")
        if USING_ROS1:
            self.client = rospy.ServiceProxy("/multiverse/socket", Socket)
            while not rospy.is_shutdown() and not rospy.wait_for_service(service="/multiverse/socket"):
                rospy.loginfo("Waiting for service socket...")
        else:
            self.client = self.create_client(srv_type=Socket, srv_name="/multiverse/socket")
            while rclpy.ok() and self.client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Waiting for service socket...")

    def send_request(self, request: Any) -> Any:
        if USING_ROS1:
            return self.client(request)
        else:
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
        init_node()

    @classmethod
    def tearDownClass(cls):
        destroy_node()

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
        init_node()
        server_port = "4321"
        process = start_multiverse_server(server_port)
        thread = threading.Thread(target=kill_ros_after_second, args=(1,))
        thread.start()
        TfPublisher._server_addr.port = server_port
        self.tf_publisher = self.tf_publisher_props.create_publisher()
        self.tf_publisher.run()
        spin_node(self.tf_publisher)
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
        self.socket_service.run()
        spin_node(self.socket_service)
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
        init_node()

    def test_socket_service(self):
        server_port = "6543"
        process = start_multiverse_server(server_port)
        ros_thread = threading.Thread(target=kill_ros_after_second, args=(1,))
        ros_thread.start()
        SocketService._server_addr.port = server_port
        self.socket_service = self.socket_service_props.create_service()
        self.socket_service.run()

        if USING_ROS1:
            executor_spin_thread = threading.Thread(target=rospy.spin)
        else:
            executor = MultiThreadedExecutor()
            executor.add_node(self.socket_service)
            executor_spin_thread = threading.Thread(target=executor.spin)
        executor_spin_thread.start()

        socket_client = SocketClient()
        if USING_ROS1:
            request = SocketRequest()
        else:
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

        if USING_ROS1:
            rospy.loginfo(f"Response: {response}")
        else:
            socket_client.get_logger().info(f"Response: {response}")
            socket_client.destroy_node()

        ros_thread.join()
        executor_spin_thread.join()
        kill_multiverse_server(process)

    @classmethod
    def tearDownClass(cls):
        destroy_node()


if __name__ == "__main__":
    unittest.main()
