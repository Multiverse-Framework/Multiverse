"""
Configuration class for the Multiverse Parser.
"""

from dataclasses import dataclass

import numpy


@dataclass
class Configuration:
    """
    Configuration class for the Multiverse Parser.
    """
    model_name: str = ""
    with_physics: bool = True
    with_visual: bool = True
    with_collision: bool = True
    default_rgba: numpy.ndarray = numpy.array([0.5, 0.5, 0.5, 1.0])
