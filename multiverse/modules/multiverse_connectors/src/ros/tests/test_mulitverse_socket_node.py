import signal
import subprocess
import threading
import unittest
from time import sleep
from typing import Any

from multiverse_ros_socket import Interface, INTERFACE

if INTERFACE == Interface.ROS1:
    import rospy
    from multiverse_msgs.srv import Socket, SocketRequest, SocketResponse

    Node = object
elif INTERFACE == Interface.ROS2:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from multiverse_interfaces.srv import Socket

    SocketRequest = Socket.Request
    SocketResponse = Socket.Response
else:
    raise ValueError(f"Invalid interface {INTERFACE}")

from multiverse_ros_socket.multiverse_node import *
from multiverse_ros_socket.multiverse_node.multiverse_node import MultiverseMetaData
from multiverse_ros_socket.multiverse_node.multiverse_node_properties import MultiverseNodeProperties

import os
from enum import Enum

ROS_DISTRO = os.environ.get("ROS_DISTRO")
USING_ROS1 = ROS_DISTRO in {"noetic", "melodic", "kinetic"}
USING_ROS2 = ROS_DISTRO in {"foxy", "galactic", "iron", "humble"}
if not USING_ROS1 and not USING_ROS2:
    raise ImportError(f"ROS_DISTRO {ROS_DISTRO} unknown.")


class Interface(Enum):
    ROS1 = "ROS1"
    ROS2 = "ROS2"


if USING_ROS1:
    INTERFACE = Interface.ROS1
elif USING_ROS2:
    INTERFACE = Interface.ROS2
else:
    raise ValueError(f"Invalid interface")


def init() -> None:
    if INTERFACE == Interface.ROS1:
        rospy.init_node("test_multiverse_ros_node_properties")
    elif INTERFACE == Interface.ROS2:
        rclpy.init()
    else:
        raise ValueError(f"Invalid interface {INTERFACE}")


def spin(node: Any) -> None:
    if INTERFACE == Interface.ROS1:
        rospy.spin()
    elif INTERFACE == Interface.ROS2:
        rclpy.spin(node)
    else:
        raise ValueError(f"Invalid interface {INTERFACE}")


def destroy() -> None:
    if INTERFACE == Interface.ROS1:
        if not rospy.is_shutdown():
            rospy.signal_shutdown("Multiverse socket shutdown")
    elif INTERFACE == Interface.ROS2:
        if rclpy.ok():
            rclpy.shutdown()
    else:
        raise ValueError(f"Invalid interface {INTERFACE}")


def start_multiverse_server(server_port: str) -> subprocess.Popen:
    return subprocess.Popen(["multiverse_server", f"tcp://127.0.0.1:{server_port}"])


def kill_multiverse_server(process: subprocess.Popen):
    process.send_signal(signal.SIGINT)
    process.wait()


def kill_ros_after_second(second: float):
    sleep(second)
    if INTERFACE == Interface.ROS1:
        rospy.signal_shutdown("Exit.")
        sleep(0.5)
    elif INTERFACE == Interface.ROS2:
        rclpy.shutdown()
    else:
        raise ValueError(f"Invalid interface {INTERFACE}")


class SocketClient(Node):
    _srv_name = "/multiverse/socket"

    def __init__(self):
        if INTERFACE == Interface.ROS1:
            if not rospy.is_shutdown():
                rospy.wait_for_service(service=self._srv_name)
                self.client = rospy.ServiceProxy(self._srv_name, Socket)
        elif INTERFACE == Interface.ROS2:
            super().__init__("socket_client")
            self.client = self.create_client(srv_type=Socket, srv_name=self._srv_name)
            while rclpy.ok() and not self.client.wait_for_service():
                self.get_logger().info(f"Waiting for {self._srv_name}...")
        else:
            raise ValueError(f"Invalid interface {INTERFACE}")

    def send_request(self, request: SocketRequest) -> SocketResponse:
        if INTERFACE == Interface.ROS1:
            if not rospy.is_shutdown():
                return self.client(request)
        elif INTERFACE == Interface.ROS2:
            if rclpy.ok():
                future = self.client.call_async(request)
                rclpy.spin_until_future_complete(self, future)
                return future.result()
        else:
            raise ValueError(f"Invalid interface {INTERFACE}")


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
        init()

    @classmethod
    def tearDownClass(cls):
        destroy()

    def test_ros_base_name(self):
        self.assertEqual(self.tf_publisher_props.ros_node_name, "Tf")


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
        init()
        server_port = "4321"
        process = start_multiverse_server(server_port)
        thread = threading.Thread(target=kill_ros_after_second, args=(1,))
        thread.start()
        TfPublisher._server_addr.port = server_port
        self.tf_publisher = self.tf_publisher_props.create_publisher()
        self.assertIsInstance(self.tf_publisher, TfPublisher)
        self.tf_publisher.run()
        spin(self.tf_publisher)
        thread.join()
        kill_multiverse_server(process)

    def test_socket_service(self):
        init()
        server_port = "5432"
        process = start_multiverse_server(server_port)
        thread = threading.Thread(target=kill_ros_after_second, args=(1,))
        thread.start()
        SocketService._server_addr.port = server_port
        self.socket_service = self.socket_service_props.create_service()
        self.assertIsInstance(self.socket_service, SocketService)
        self.socket_service.run()
        spin(self.socket_service)
        thread.join()
        kill_multiverse_server(process)

    @classmethod
    def tearDownClass(cls):
        destroy()


class SocketServiceTestCase(unittest.TestCase):
    socket_service_props = MultiverseNodeProperties(
        "socket", {"host": "tcp://127.0.0.1", "port": "2346"}
    )

    @classmethod
    def setUpClass(cls):
        init()

    def test_socket_service(self):
        server_port = "6543"
        process = start_multiverse_server(server_port)
        ros_thread = threading.Thread(target=kill_ros_after_second, args=(1,))
        ros_thread.start()
        SocketService._server_addr.port = server_port
        self.socket_service = self.socket_service_props.create_service()
        self.socket_service.run()

        if INTERFACE == Interface.ROS1:
            spin_thread = threading.Thread(target=rospy.spin)
        elif INTERFACE == Interface.ROS2:
            executor = MultiThreadedExecutor()
            executor.add_node(self.socket_service)
            spin_thread = threading.Thread(target=executor.spin)
        else:
            raise ValueError(f"Invalid interface {INTERFACE}")

        spin_thread.start()

        socket_client = SocketClient()
        request = SocketRequest()
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

        if INTERFACE == Interface.ROS1:
            rospy.loginfo(f"Response: {response}")

        elif INTERFACE == Interface.ROS2:
            socket_client.get_logger().info(f"Response: {response}")
            socket_client.destroy_node()
        else:
            raise ValueError(f"Invalid interface {INTERFACE}")

        ros_thread.join()
        spin_thread.join()
        kill_multiverse_server(process)

    @classmethod
    def tearDownClass(cls):
        destroy()


if __name__ == "__main__":
    unittest.main()
