#!/usr/bin/env python3.10

from dataclasses import dataclass
from enum import Enum

import numpy


class InertiaSource(Enum):
    FROM_SRC = 0
    FROM_MESH = 1


@dataclass
class Configuration:
    """
    Configuration class for the Multiverse Parser.
    """
    model_name: str = ""
    with_physics: bool = True
    with_visual: bool = True
    with_collision: bool = True
    inertia_source: InertiaSource = InertiaSource.FROM_SRC
    default_rgba: numpy.ndarray = numpy.array([0.5, 0.5, 0.5, 1.0])
