#!/usr/bin/env python3

from multiverse_socket.multiverse_publishers import MultiverseRosPublisher


class visualization_marker_array_publisher(MultiverseRosPublisher):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
