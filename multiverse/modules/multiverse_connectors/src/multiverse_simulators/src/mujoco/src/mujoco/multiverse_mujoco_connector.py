#!/usr/bin/env python3

"""Multiverse Mujoco Connector class"""

from multiverse_client_py import MultiverseMetaData
from multiverse_simulator import MultiverseSimulator


class MultiverseMujocoConnector(MultiverseSimulator):
    """Multiverse Mujoco Connector class"""

    def __init__(self, xml_path: str, host: str, server_port: str, client_port: str, meta_data: MultiverseMetaData):
        super().__init__(host, server_port, client_port, meta_data)

    def start(self):
        pass

    def stop(self):
        pass

    def pause(self):
        pass

    def unpause(self):
        pass

    def reset(self):
        pass