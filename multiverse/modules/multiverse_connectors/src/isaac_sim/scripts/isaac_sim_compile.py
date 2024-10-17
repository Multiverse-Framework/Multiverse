#!/usr/bin/env python3

import argparse
import dataclasses
import json
import os
import shutil
import xml.etree.ElementTree as ET
from typing import List, Dict, Set, Any, Union, Optional, Tuple


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


class IsaacSimCompiler:
    world_usd_path: str
    scene_name: str
    save_usd_path: str
    robots: List[Robot]
    objects: List[Object]

    def __init__(self, args):
        self.world_usd_path = args.world
        print(f"World: {self.world_usd_path}")
        self.scene_name = args.name
        self.save_dir_path = os.path.join(args.save_dir, self.scene_name)
        self.save_usd_dir = args.save_dir
        self.robots = []
        self.objects = []

    def build_world_usd(self, robots: Dict[str, Robot], objects: Dict[str, Object]):
        self.create_world_usd()

    def create_world_usd(self):
        if not os.path.exists(self.save_dir_path):
            os.makedirs(self.save_dir_path)
        self.save_usd_path = os.path.join(self.save_dir_path, self.scene_name + ".usda")
        shutil.copy(self.world_usd_path, self.save_usd_path)

def main():
    # Initialize argument parser
    parser = argparse.ArgumentParser(description="Compile MJCF from world and robots.")

    # Define arguments
    parser.add_argument("--name", help="Name of the simulation", required=True)
    parser.add_argument("--world", help="Path to world MJCF", required=True)
    parser.add_argument("--robots", help="JSON string with robots' data", required=False)
    parser.add_argument("--objects", help="JSON string with objects' data", required=False)
    parser.add_argument("--references", help="JSON string with references' data", required=False)
    if os.path.basename(__file__) == "isaac_sim_compile":
        save_dir = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "saved"
        )
    elif os.path.basename(__file__) == "isaac_sim_compile.py":
        if os.name == "nt":
            save_dir = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "..",
                "saved",
            )
        else:
            save_dir = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "..",
                "..",
                "..",
                "..",
                "..",
                "saved",
            )
    else:
        raise RuntimeError(f"Unknown file name {os.path.basename(__file__)}")

    parser.add_argument("--save_dir", help="Path to save directory", required=False, default=save_dir)

    # Parse arguments
    args, _ = parser.parse_known_args()

    compiler = IsaacSimCompiler(args)
    compiler.build_world_usd(
        robots=parse_entity(args.robots, Robot),
        objects=parse_entity(args.objects, Object),
    )
    print(f"Scene: {compiler.save_usd_path}", end="")


if __name__ == "__main__":
    main()
