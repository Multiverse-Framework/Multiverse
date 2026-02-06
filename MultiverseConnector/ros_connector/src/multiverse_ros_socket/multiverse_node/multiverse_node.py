#!/usr/bin/env python3

from .. import Interface, INTERFACE

from multiverse_client_py import MultiverseClient, MultiverseMetaData

if INTERFACE == Interface.ROS1:
    import rospy

    Node = object
elif INTERFACE == Interface.ROS2:
    from rclpy.node import Node
else:
    raise ValueError(f"Invalid INTERFACE {INTERFACE}.")


class MultiverseNode(MultiverseClient, Node):
    def __init__(self,
                 port: str,
                 multiverse_meta_data: MultiverseMetaData = MultiverseMetaData()) -> None:
        if INTERFACE == Interface.ROS1:
            multiverse_meta_data.simulation_name = f"ros_{port}"
        elif INTERFACE == Interface.ROS2:
            multiverse_meta_data.simulation_name = f"ros2_{port}"
            Node.__init__(self, node_name=f"{self.__class__.__name__}{port}")
        super().__init__(port, multiverse_meta_data)

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
