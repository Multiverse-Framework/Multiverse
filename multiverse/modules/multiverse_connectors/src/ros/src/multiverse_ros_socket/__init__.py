import os
from enum import Enum

ROS_DISTRO = os.environ.get("ROS_DISTRO")
USING_ROS1 = ROS_DISTRO in {"noetic", "melodic", "kinetic"}
USING_ROS2 = ROS_DISTRO in {"foxy", "galactic", "iron", "humble", "jazzy"}
if not USING_ROS1 and not USING_ROS2:
    raise ImportError(f"ROS_DISTRO {ROS_DISTRO} unknown.")


class Interface(Enum):
    ROS1 = "ROS1"
    ROS2 = "ROS2"


if USING_ROS1:
    INTERFACE = Interface.ROS1
elif USING_ROS2:
    INTERFACE = Interface.ROS2
else:
    raise ValueError(f"Invalid interface")
