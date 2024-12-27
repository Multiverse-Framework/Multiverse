#!/usr/bin/env python3

"""Multiverse Simulator Compiler base class"""

import argparse
import dataclasses
import os
import shutil
from typing import Dict, Any
from .utils import str_to_dict


@dataclasses.dataclass
class Entity:
    name = ""
    path = ""
    saved_path = None
    joint_state = {}
    apply = {}
    attach = {}
    disable_self_collision = "auto"
    prefix = {}
    suffix = {}


@dataclasses.dataclass
class Robot(Entity):
    pass


@dataclasses.dataclass
class Object(Entity):
    pass


def parse_entity(data: str, cls: type) -> Dict[str, Any]:
    root = str_to_dict(data)
    if root == {}:
        return {}

    entities = {}
    for entity_name, entity_data in root.items():
        entity = cls()
        entity.name = entity_name
        entity.path = entity_data.get("path", "")
        entity.apply = entity_data.get("apply", {})
        entity.attach = entity_data.get("attach", {})
        entity.prefix = entity_data.get("prefix", {"body": "", "joint": "", "geom": "", "mesh": "", "actuator": ""})
        entity.suffix = entity_data.get("suffix", {"body": "", "joint": "", "geom": "", "mesh": "", "actuator": ""})
        entity.joint_state = entity_data.get("joint_state", {})
        entity.disable_self_collision = entity_data.get(
            "disable_self_collision", "auto"
        )
        entities[entity_name] = entity

    return entities


class MultiverseSimulatorCompiler:
    ext: str = ""

    def __init__(self, args):
        self._world_path = args.world_path
        self._scene_name = args.name
        self._save_file_path = os.path.join(args.save_dir_path, f"{self.scene_name}.{self.ext}")
        self.create_world()

    @property
    def world_path(self) -> str:
        return self._world_path

    @property
    def scene_name(self) -> str:
        return self._scene_name

    @property
    def save_file_path(self) -> str:
        return self._save_file_path

    @property
    def save_dir_path(self) -> str:
        return os.path.dirname(self.save_file_path)

    def build_world(self,
                    robots: Dict[str, Robot],
                    objects: Dict[str, Object],
                    references: Dict[str, Dict[str, Any]] = None,
                    multiverse_params: Dict[str, Dict] = None):
        raise NotImplementedError("build_world method must be implemented")

    def create_world(self):
        if not os.path.exists(self.save_dir_path):
            os.makedirs(self.save_dir_path)
        shutil.copy(self.world_path, self.save_file_path)


def multiverse_simulator_compiler_main(Compiler=MultiverseSimulatorCompiler):
    save_dir_path_1 = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "saved")
    save_dir_path_2 = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "..", "..",
                                   "saved")
    if os.path.exists(save_dir_path_1):
        save_dir_path = save_dir_path_1
    elif os.path.exists(save_dir_path_2):
        save_dir_path = save_dir_path_2
    else:
        raise FileNotFoundError(f"Could not find default save directory")

    # Initialize argument parser
    parser = argparse.ArgumentParser(description=f"Compile {Compiler.ext} from world and robots.")

    # Define arguments
    parser.add_argument("--name", type=str, required=True, help="Name of the simulation")
    parser.add_argument("--world_path", type=str, required=True, help=f"Path to world {Compiler.ext}")
    parser.add_argument("--robots", type=str, required=False, help="JSON string with robots' data")
    parser.add_argument("--objects", type=str, required=False, help="JSON string with objects' data")
    parser.add_argument("--references", type=str, required=False, help="JSON string with references' data")
    parser.add_argument("--save_dir_path", default=save_dir_path, type=str, required=False,
                        help="Path to save directory")
    parser.add_argument("--multiverse_params", type=str, required=False, help="JSON string with multiverse' data")

    # Parse arguments
    args = parser.parse_args()

    if not os.path.exists(args.save_dir_path):
        os.makedirs(args.save_dir_path)
    assert args.world_path != "" and os.path.exists(args.world_path)

    compiler = Compiler(args)
    compiler.build_world(
        robots=parse_entity(args.robots, Robot),
        objects=parse_entity(args.objects, Object),
        references=str_to_dict(args.references),
        multiverse_params=str_to_dict(args.multiverse_params)
    )
    print(f"World: {compiler.world_path}")
    print(f"Scene: {compiler.save_file_path}", end="")
