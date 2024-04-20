#!/usr/bin/env python3

from .. import Interface, INTERFACE

from multiverse_client_py import MultiverseClient, MultiverseMetaData, SocketAddress

if INTERFACE == Interface.ROS1:
    import rospy

    Node = object
elif INTERFACE == Interface.ROS2:
    from rclpy.node import Node
else:
    raise ValueError(f"Invalid INTERFACE {INTERFACE}.")


class MultiverseNode(MultiverseClient, Node):
    def __init__(self,
                 client_addr: SocketAddress,
                 multiverse_meta_data: MultiverseMetaData = MultiverseMetaData()) -> None:
        multiverse_meta_data.simulation_name = "ros"
        if INTERFACE == Interface.ROS2:
            Node.__init__(self, node_name=f"{self.__class__.__name__}{client_addr.port}")
        super().__init__(client_addr, multiverse_meta_data)

    def loginfo(self, message: str) -> None:
        if INTERFACE == Interface.ROS1:
            rospy.loginfo(message)
        elif INTERFACE == Interface.ROS2:
            self.get_logger().info(message)

    def logwarn(self, message: str) -> None:
        if INTERFACE == Interface.ROS1:
            rospy.logwarn(message)
        elif INTERFACE == Interface.ROS2:
            self.get_logger().warn(message)
