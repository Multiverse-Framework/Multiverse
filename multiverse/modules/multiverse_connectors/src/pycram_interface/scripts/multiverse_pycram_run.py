#!/usr/bin/env python3

from typing_extensions import List, Dict

from multiverse_pycram_socket.multiverse_socket import MultiverseSocket, SocketAddress
from pycram.pose import Pose
from pycram.world import World, Link, Object, Constraint
from pycram.world_dataclasses import AxisAlignedBoundingBox, Color


class Multiverse(MultiverseSocket, World):

    def __init__(self):
        SocketAddress.host = "tcp://127.0.0.1"
        socket_addr = SocketAddress(port="7000")
        super().__init__(socket_addr)

    def load_description_and_get_object_id(self, path: str, pose: Pose) -> int:
        pass

    def remove_object_from_simulator(self, obj: Object) -> None:
        pass

    def add_constraint(self, constraint: Constraint) -> int:
        pass

    def remove_constraint(self, constraint_id) -> None:
        pass

    def get_joint_position(self, obj: Object, joint_name: str) -> float:
        pass

    def get_link_pose(self, link: Link) -> Pose:
        pass

    def get_object_pose(self, obj: Object) -> Pose:
        pass

    def perform_collision_detection(self) -> None:
        pass

    def get_object_contact_points(self, obj: Object) -> List:
        pass

    def get_contact_points_between_two_objects(self, obj1: Object, obj2: Object) -> List:
        pass

    def reset_joint_position(self, obj: Object, joint_name: str, joint_pose: float) -> None:
        pass

    def reset_object_base_pose(self, obj: Object, pose: Pose):
        pass

    def step(self):
        pass

    def set_link_color(self, link: Link, rgba_color: Color):
        pass

    def get_link_color(self, link: Link) -> Color:
        pass

    def get_colors_of_object_links(self, obj: Object) -> Dict[str, Color]:
        pass

    def get_object_axis_aligned_bounding_box(self, obj: Object) -> AxisAlignedBoundingBox:
        pass

    def get_link_axis_aligned_bounding_box(self, link: Link) -> AxisAlignedBoundingBox:
        pass

    def set_realtime(self, real_time: bool) -> None:
        pass

    def set_gravity(self, gravity_vector: List[float]) -> None:
        pass

    def disconnect_from_physics_server(self) -> None:
        pass

    def join_threads(self) -> None:
        pass

    def save_physics_simulator_state(self) -> int:
        pass

    def remove_physics_simulator_state(self, state_id: int) -> None:
        pass

    def restore_physics_simulator_state(self, state_id: int) -> None:
        pass

    def ray_test(self, from_position: List[float], to_position: List[float]) -> int:
        pass

    def ray_test_batch(self, from_positions: List[List[float]], to_positions: List[List[float]],
                       num_threads: int = 1) -> List[int]:
        pass


if __name__ == "__main__":
    # main()
    Multiverse()
