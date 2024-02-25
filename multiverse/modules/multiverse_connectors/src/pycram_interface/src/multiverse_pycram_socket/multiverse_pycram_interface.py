#!/usr/bin/env python3
import logging

from typing_extensions import List, Dict, Optional

from pycram.enums import WorldMode, JointType
from .multiverse_socket import MultiverseSocket, SocketAddress
from pycram.pose import Pose
from pycram.world import World, Link, Object, Constraint, Joint
from pycram.world_dataclasses import AxisAlignedBoundingBox, Color


class Multiverse(MultiverseSocket, World):
    """
    This class implements an interface between Multiverse and PyCRAM.
    """

    _joint_type_to_attribute: Dict[JointType, str] = {
        JointType.REVOLUTE: "joint_rvalue",
        JointType.PRISMATIC: "joint_tvalue",
    }
    """
    A dictionary to map JointType to the corresponding multiverse attribute name.
    """

    def __init__(self, mode: Optional[WorldMode] = WorldMode.DIRECT,
                 is_prospection: Optional[bool] = False,
                 simulation_frequency: Optional[float] = 60.0,
                 client_addr: Optional[SocketAddress] = None):
        """
        Initialize the Multiverse Socket and the PyCram World.
        """
        MultiverseSocket.__init__(self, client_addr)
        World.__init__(self, mode, is_prospection, simulation_frequency)
        self.last_object_id = -1
        self._connect_and_start()

    def get_joint_attribute(self, joint: Joint) -> str:
        if joint.type not in self._joint_type_to_attribute:
            logging.warning(f"Invalid joint type: {joint.type}")
            return "joint_rvalue"
        return self._joint_type_to_attribute[joint.type]

    def load_description_and_get_object_id(self, path: str, pose: Pose) -> int:
        self.last_object_id += 1
        return self.last_object_id

    def get_object_joint_names(self, obj: Object) -> List[str]:
        return [joint.name for joint in obj.description.joints]

    def get_object_link_names(self, obj: Object) -> List[str]:
        return [link.name for link in obj.description.links]

    def remove_object_from_simulator(self, obj: Object) -> None:
        logging.warning("remove_object_from_simulator is not implemented in Multiverse")

    def add_constraint(self, constraint: Constraint) -> int:
        logging.warning("add_constraint is not implemented in Multiverse")
        return 0

    def remove_constraint(self, constraint_id) -> None:
        logging.warning("remove_constraint is not implemented in Multiverse")

    def get_joint_position(self, joint: Joint) -> float:
        attribute = self.get_joint_attribute(joint)
        self.request_meta_data["receive"][joint.name] = [attribute]
        self._communicate()
        self._bind_receive_data(self.receive_data)
        if len(self.receive_data) != 1:
            logging.error(f"Invalid joint position data: {self.receive_data}")
            raise ValueError
        return self.receive_data[0]

    def reset_joint_position(self, joint: Joint, joint_position: float) -> None:
        attribute = self.get_joint_attribute(joint)
        self.request_meta_data["send"][joint.name] = [attribute]
        self.send_data = [0.0, joint_position]
        # first element of send_data is for time.
        self._communicate()

    def get_link_pose(self, link: Link) -> Pose:
        return self._get_body_pose(link.name)

    def get_object_pose(self, obj: Object) -> Pose:
        return self._get_body_pose(obj.name)

    def _get_body_pose(self, body_name: str) -> Pose:
        self.request_meta_data["receive"][body_name] = ["position", "quaternion"]
        self._communicate()
        self._bind_receive_data(self.receive_data)
        if len(self.receive_data) != 6:
            logging.error(f"Invalid body pose data: {self.receive_data}")
            raise ValueError
        return Pose(*self.receive_data[:3], *self.receive_data[3:])

    def reset_object_base_pose(self, obj: Object, pose: Pose):
        self.request_meta_data["send"][obj.name] = ["position", "quaternion"]
        self.send_data = [0.0, *pose.position_as_list(), *pose.orientation_as_list()]
        # first element of send_data is for time.
        self._communicate()

    def perform_collision_detection(self) -> None:
        logging.warning("perform_collision_detection is not implemented in Multiverse")

    def get_object_contact_points(self, obj: Object) -> List:
        logging.warning("get_object_contact_points is not implemented in Multiverse")
        return []

    def get_contact_points_between_two_objects(self, obj1: Object, obj2: Object) -> List:
        logging.warning("get_contact_points_between_two_objects is not implemented in Multiverse")
        return []

    def step(self):
        logging.warning("step is not implemented in Multiverse")

    def set_link_color(self, link: Link, rgba_color: Color):
        logging.warning("set_link_color is not implemented in Multiverse")

    def get_link_color(self, link: Link) -> Color:
        logging.warning("get_link_color is not implemented in Multiverse")
        return Color()

    def get_colors_of_object_links(self, obj: Object) -> Dict[str, Color]:
        logging.warning("get_colors_of_object_links is not implemented in Multiverse")
        return {}

    def get_object_axis_aligned_bounding_box(self, obj: Object) -> AxisAlignedBoundingBox:
        logging.error("get_object_axis_aligned_bounding_box is not implemented in Multiverse")
        raise NotImplementedError

    def get_link_axis_aligned_bounding_box(self, link: Link) -> AxisAlignedBoundingBox:
        logging.error("get_link_axis_aligned_bounding_box is not implemented in Multiverse")
        raise NotImplementedError

    def set_realtime(self, real_time: bool) -> None:
        logging.warning("set_realtime is not implemented in Multiverse")

    def set_gravity(self, gravity_vector: List[float]) -> None:
        logging.warning("set_gravity is not implemented in Multiverse")

    def disconnect_from_physics_server(self) -> None:
        self.stop()

    def join_threads(self) -> None:
        pass

    def save_physics_simulator_state(self) -> int:
        logging.warning("save_physics_simulator_state is not implemented in Multiverse")
        return 0

    def remove_physics_simulator_state(self, state_id: int) -> None:
        logging.warning("remove_physics_simulator_state is not implemented in Multiverse")

    def restore_physics_simulator_state(self, state_id: int) -> None:
        logging.error("restore_physics_simulator_state is not implemented in Multiverse")
        raise NotImplementedError

    def ray_test(self, from_position: List[float], to_position: List[float]) -> int:
        logging.error("ray_test is not implemented in Multiverse")
        raise NotImplementedError

    def ray_test_batch(self, from_positions: List[List[float]], to_positions: List[List[float]],
                       num_threads: int = 1) -> List[int]:
        logging.error("ray_test_batch is not implemented in Multiverse")
        raise NotImplementedError


if __name__ == "__main__":
    # main()
    Multiverse()
