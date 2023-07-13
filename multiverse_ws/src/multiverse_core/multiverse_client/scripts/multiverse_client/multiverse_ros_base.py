#!/usr/bin/env python3

from multiverse_client.utils.multiverse_utils import constuct_send_meta_data_dict
import rospy
import sys
import os
from typing import List

sys.path.append(os.path.dirname(os.path.dirname(sys.argv[0])))
from multiverse_socket import MultiverseSocket  # noqa


class MultiverseRosBase:
    _send_meta_data_dict = {}
    _receive_meta_data_dict = {}

    def __init__(self, host: str, port: str, **kwargs) -> None:
        self.host = host
        self.port = port
        self.use_thread = True
        self._init_send_meta_data()

    def start(self) -> None:
        pass

    def _init_multiverse_socket(self):
        self.__multiverse_socket = MultiverseSocket(self.use_thread)
        self.__multiverse_socket.init(self.host, self.port)

    def _init_send_meta_data(self) -> None:
        self._send_meta_data_dict = constuct_send_meta_data_dict()

    def _connect(self) -> None:
        self.__multiverse_socket.connect()

    def _assign_send_meta_data(self):
        self.__multiverse_socket.set_send_meta_data(self._send_meta_data_dict)

    def _retrieve_receive_meta_data(self, time_out=1) -> bool:
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            self._receive_meta_data_dict = self.__multiverse_socket.get_receive_meta_data()
            if self._receive_meta_data_dict:
                return True
            elif (rospy.Time.now() - start).to_sec() > time_out:
                return False
        return False

    def _send_data(self, send_data: List[float]) -> None:
        self.__multiverse_socket.set_send_data(send_data)
        self.__multiverse_socket.communicate()

    def _receive_data(self) -> List[float]:
        return self.__multiverse_socket.get_receive_data()

    def _disconnect(self) -> None:
        self.__multiverse_socket.disconnect()

    def _restart(self) -> None:
        self.disconnect()
        self.connect()
