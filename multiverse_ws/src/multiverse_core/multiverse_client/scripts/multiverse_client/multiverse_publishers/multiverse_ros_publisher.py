#!/usr/bin/env python3

from multiverse_socket import MultiverseSocket  # noqa


class MultiverseRosPublisher:
    def __init__(self, host: str, port: str) -> None:
        self.host = host
        self.port = port
        pass

    
