#!/usr/bin/env python3

from .ros_publisher import MultiverseRosPublisher


class visualization_marker_array_publisher(MultiverseRosPublisher):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
