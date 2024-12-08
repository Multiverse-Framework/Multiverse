#!/usr/bin/env python3

"""Multiverse Simulator Compiler base class"""

import argparse
import dataclasses
import json
import os
import shutil
from typing import List, Dict, Any


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
    if data is None:
        return {}
    try:
        root = json.loads(data.replace("'", '"'))
    except json.JSONDecodeError as e:
        print(f"Failed to parse {data}: {str(e)}")
        return {}

    entities = {}
    for entity_name, entity_data in root.items():
        entity = cls()
        entity.name = entity_name
        entity.path = entity_data.get("path", "")
        entity.apply = entity_data.get("apply", {})
        entity.attach = entity_data.get("attach", {})
        entity.prefix = entity_data.get("prefix", {"body": "", "joint": "", "geom": ""})
        entity.suffix = entity_data.get("suffix", {"body": "", "joint": "", "geom": ""})
        entity.joint_state = entity_data.get("joint_state", {})
        entity.disable_self_collision = entity_data.get(
            "disable_self_collision", "auto"
        )
        entities[entity_name] = entity

    return entities


def update_element_name(elements: list, element_type: str, prefix: str, suffix: str):
    for element_id, element in enumerate(elements):
        element_name = element.get("name", None)
        if element_name is None:
            element_name = f"{element_type}_{element_id}"
            element.set("name", element_name)
        element.set("name", f"{prefix}{element_name}{suffix}")


class MultiverseSimulatorCompiler:
    ext: str = ""

    def __init__(self, args):
        self._world_path = args.world_path
        self._scene_name = args.name
        self._save_file_path = os.path.join(args.save_dir_path, f"{self.scene_name}.{self.ext}")
        self._robots = []
        self._objects = []
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

    @property
    def robots(self) -> List[Robot]:
        return self._robots

    @property
    def objects(self) -> List[Object]:
        return self._objects

    def build_world(self, robots: Dict[str, Robot], objects: Dict[str, Object], multiverse_params: Dict[str, Dict]):
        raise NotImplementedError("build_world method must be implemented")

    def create_world(self):
        if not os.path.exists(self.save_dir_path):
            os.makedirs(self.save_dir_path)
        shutil.copy(self.world_path, self.save_file_path)


def multiverse_simulator_compiler_main(Compiler=MultiverseSimulatorCompiler):
    compiler_file_name = os.path.basename(__file__)
    save_dir_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "saved")
    if compiler_file_name == f"multiverse_simulator_compiler" or os.name == "nt":
        pass
    elif compiler_file_name == f"multiverse_simulator_compiler.py":
        save_dir_path = (
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "..", "..", "saved"))
    else:
        raise RuntimeError(
            f"Unknown file name multiverse_simulator_compiler, expected multiverse_simulator_compiler.py")

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

    assert os.path.exists(args.save_dir_path)
    assert args.world_path != "" and os.path.exists(args.world_path)

    if args.multiverse_params is None:
        multiverse_params = {}
    else:
        try:
            multiverse_params = json.loads(args.multiverse_params.replace("'", '"'))
        except json.JSONDecodeError as e:
            print(f"Failed to parse {args.multiverse_params}: {str(e)}")
            multiverse_params = {}

    compiler = Compiler(args)
    compiler.build_world(
        robots=parse_entity(args.robots, Robot),
        objects=parse_entity(args.objects, Object),
        multiverse_params=multiverse_params
    )
    print(f"World: {compiler.world_path}")
    print(f"Scene: {compiler.save_file_path}", end="")
