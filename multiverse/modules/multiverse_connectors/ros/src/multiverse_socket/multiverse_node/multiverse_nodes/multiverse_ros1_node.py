#!/usr/bin/env python3

import rospy

from ..multiverse_meta_node import MultiverseMetaNode, SocketAddress, MultiverseMetaData, LoggingLevel


class MultiverseNode(MultiverseMetaNode):
    def __init__(
            self,
            client_addr: SocketAddress,
            multiverse_meta_data: MultiverseMetaData = MultiverseMetaData(),
    ) -> None:
        super().__init__(client_addr=client_addr, multiverse_meta_data=multiverse_meta_data)
        self._run()

    def _logging(self, log_str: str, logging_level: LoggingLevel = LoggingLevel.INFO) -> None:
        if logging_level == LoggingLevel.INFO:
            rospy.loginfo(log_str)
        elif logging_level == LoggingLevel.WARN:
            rospy.logwarn(log_str)
        elif logging_level == LoggingLevel.ERROR:
            rospy.logerr(log_str)
