#!/usr/bin/env python3

"""Multiverse Simulator base class"""

from multiverse_client_py import MultiverseMetaData

class MultiverseSimulator:
    """Base class for Multiverse Simulator"""

    def __init__(self, host: str, server_port: str, client_port: str, meta_data: MultiverseMetaData):
        pass

    def start(self):
        raise NotImplementedError("start() method must be implemented")

    def stop(self):
        raise NotImplementedError("stop() method must be implemented")

    def pause(self):
        raise NotImplementedError("pause() method must be implemented")

    def unpause(self):
        raise NotImplementedError("unpause() method must be implemented")

    def reset(self):
        raise NotImplementedError("reset() method must be implemented")