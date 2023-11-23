#!/usr/bin/env python3

from rclpy.node import Node

from ..multiverse_meta_node import MultiverseMetaNode, SocketAddress, MultiverseMetaData, LoggingLevel


class MultiverseNode(MultiverseMetaNode, Node):
    def __init__(
            self,
            node_name: str,
            client_addr: SocketAddress,
            multiverse_meta_data: MultiverseMetaData = MultiverseMetaData(),
    ) -> None:
        super().__init__(client_addr=client_addr, multiverse_meta_data=multiverse_meta_data)
        super().__init__(node_name=f"{node_name}{client_addr.port}")
        self._run()

    def _logging(self, log_str: str, logging_level: LoggingLevel = LoggingLevel.INFO) -> None:
        if logging_level == LoggingLevel.INFO:
            self.get_logger().info(log_str)
        elif logging_level == LoggingLevel.WARN:
            self.get_logger().warn(log_str)
        elif logging_level == LoggingLevel.ERROR:
            self.get_logger().error(log_str)
