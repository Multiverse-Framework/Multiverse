#!/usr/bin/env python3.10

import dataclasses
import json
import argparse
import shutil
import os
import mujoco
from typing import List, Dict, Set, Any, Optional, Union
import xml.etree.ElementTree as ET


@dataclasses.dataclass
class Robot:
    name = ""
    path = ""
    joint_state = {}
    apply = {}
    disable_self_collision = "auto"
    prefix = {}
    suffix = {}


@dataclasses.dataclass
class Object:
    name = ""
    path = ""
    joint_state = {}
    apply = {}
    disable_self_collision = "auto"
    prefix = {}
    suffix = {}


def get_body_collision_dict(xml_path: str) -> Dict[str, Set[str]]:
    body_collision_dict = {}
    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d)
    for geom1_id, geom2_id in zip(d.contact.geom1, d.contact.geom2):
        body1_id = m.geom(geom1_id).bodyid[0]
        body1_name = m.body(body1_id).name
        body2_id = m.geom(geom2_id).bodyid[0]
        body2_name = m.body(body2_id).name
        body_collision_dict[body1_name] = body_collision_dict.get(body1_name, set()).union({body2_name})
    return body_collision_dict


def parse_robot_or_object(data: str, cls) -> List[Any]:
    if data is None:
        return []
    try:
        root = json.loads(data.replace("'", '"'))
    except json.JSONDecodeError as e:
        print(f"Failed to parse {data}: {str(e)}")
        return []

    entities = []
    for entity_name, entity_data in root.items():
        entity = cls()
        entity.name = entity_name
        entity.path = entity_data.get("path", "")
        entity.apply = entity_data.get("apply", {})
        entity.prefix = entity_data.get("prefix", {"body": "", "joint": "", "geom": ""})
        entity.suffix = entity_data.get("suffix", {"body": "", "joint": "", "geom": ""})
        entity.joint_state = entity_data.get("joint_state", {})
        entity.disable_self_collision = entity_data.get("disable_self_collision", "auto")
        entities.append(entity)

    return entities


def update_element_name(elements: list, element_type: str, prefix: str, suffix: str):
    for element_id, element in enumerate(elements):
        element_name = element.get("name", None)
        if element_name is None:
            element_name = f"{element_type}_{element_id}"
            element.set("name", element_name)
        element.set("name", f"{prefix}{element_name}{suffix}")


def get_all_descendants(m, body_id, descendants=None):
    if descendants is None:
        descendants = []
    for i in range(m.nbody):
        if m.body_parentid[i] == body_id:
            descendants.append(i)
            get_all_descendants(m, i, descendants)
    return descendants


def remove_dirs_and_get_abs_dirs(root: ET.Element, xml_path: str) -> (str, str):
    meshdir = os.path.dirname(xml_path)
    texturedir = os.path.dirname(xml_path)
    for compiler_element in root.findall("compiler"):
        if "meshdir" in compiler_element.attrib:
            meshdir = compiler_element.get("meshdir")
            del compiler_element.attrib["meshdir"]
        if "texturedir" in compiler_element.attrib:
            texturedir = compiler_element.get("texturedir")
            del compiler_element.attrib["texturedir"]

    if not os.path.isabs(meshdir):
        meshdir = os.path.join(os.path.dirname(xml_path), meshdir)
    if not os.path.isabs(texturedir):
        texturedir = os.path.join(os.path.dirname(xml_path), texturedir)

    return meshdir, texturedir


def add_visual_element(root: ET.Element):
    visual_element = ET.Element("visual")
    root.append(visual_element)
    global_element = ET.Element(
        "global", {"fovy": "45", "azimuth": "225", "elevation": "-30"}
    )
    visual_element.append(global_element)


def add_cursor_element(root: ET.Element):
    worldbody_element = ET.Element("worldbody")
    root.append(worldbody_element)
    body_element = ET.Element(
        "body",
        {"name": "cursor", "mocap": "true", "pos": "0 0 0", "euler": "0 0 0"},
    )
    worldbody_element.append(body_element)

    body_element.append(
        ET.Element(
            "site",
            {
                "name": "cursor_x",
                "pos": "0.1 0.0 0.0",
                "size": "0.01 0.1",
                "euler": "0.0 1.5708 0.0",
                "type": "cylinder",
                "rgba": "1 0 0 0.5",
            },
        )
    )
    body_element.append(
        ET.Element(
            "site",
            {
                "name": "cursor_y",
                "pos": "0.0 0.1 0.0",
                "size": "0.01 0.1",
                "euler": "1.5708 0.0 0.0",
                "type": "cylinder",
                "rgba": "0 1 0 0.5",
            },
        )
    )
    body_element.append(
        ET.Element(
            "site",
            {
                "name": "cursor_z",
                "pos": "0.0 0.0 0.1",
                "size": "0.01 0.1",
                "euler": "0.0 0.0 0.0",
                "type": "cylinder",
                "rgba": "0 0 1 0.5",
            },
        )
    )


def exclude_collision(root: ET.Element, xml_path: str, include_collision: List[str]) -> Dict[str, Set[str]]:
    body_collision_dict = get_body_collision_dict(xml_path)
    contact_element = ET.Element("contact")
    root.append(contact_element)
    for body1_name, body2_names in body_collision_dict.items():
        if body1_name in include_collision:
            continue
        for body2_name in body2_names:
            if body2_name in include_collision:
                continue
            contact_element.append(
                ET.Element(
                    "exclude",
                    {
                        "name": f"{body1_name}_{body2_name}",
                        "body1": body1_name,
                        "body2": body2_name,
                    },
                ),
            )
    return body_collision_dict


def add_prefix_and_suffix(root: ET.Element, prefix, suffix):
    for element_type in ["body", "joint", "geom"]:
        element_prefix = prefix.get(element_type, "")
        element_suffix = suffix.get(element_type, "")
        update_element_name(
            root.findall(f".//{element_type}"), element_type, element_prefix, element_suffix
        )

    body_prefix = prefix.get("body", "")
    body_suffix = suffix.get("body", "")
    for contact_element in root.findall("contact"):
        for exclude_element in contact_element.findall("exclude"):
            body1_name = exclude_element.get("body1")
            body1_name = f"{body_prefix}{body1_name}{body_suffix}"
            exclude_element.set("body1", body1_name)
            body2_name = exclude_element.get("body2")
            body2_name = f"{body_prefix}{body2_name}{body_suffix}"
            exclude_element.set("body2", f"{body2_name}")
            exclude_element.set("name", f"{body1_name}_{body2_name}")

    joint_prefix = prefix.get("joint", "")
    joint_suffix = suffix.get("joint", "")
    for actuator_element in root.findall("actuator"):
        for actuator_type in {"general", "motor", "position", "velocity"}:
            for element in actuator_element.findall(actuator_type):
                joint_name = element.get("joint")
                joint_name = f"{joint_prefix}{joint_name}{joint_suffix}"
                element.set("joint", joint_name)
                element_name = element.get("name")
                element_name = f"{joint_prefix}{element_name}{joint_suffix}"
                element.set("name", element_name)

    for equality_element in root.findall("equality"):
        for joint_element in equality_element.findall("joint"):
            joint1_name = joint_element.get("joint1")
            joint1_name = f"{joint_prefix}{joint1_name}{joint_suffix}"
            joint_element.set("joint1", joint1_name)
            joint2_name = joint_element.get("joint2")
            joint2_name = f"{joint_prefix}{joint2_name}{joint_suffix}"
            joint_element.set("joint2", f"{joint2_name}")


def apply_properties(root: ET.Element, apply: Dict[str, Dict[str, Any]]):
    for element_type, attributes in apply.items():
        for attribute_name, attribute_props in attributes.items():
            if isinstance(attribute_props, dict):
                for attribute_child_name, attribute_child_prop in attribute_props.items():
                    for element in list(root.iter(element_type)):
                        if element.get("name") == attribute_name:
                            if isinstance(attribute_child_prop, list):
                                element.set(attribute_child_name, " ".join(map(str, attribute_child_prop)))
                            elif isinstance(attribute_child_prop, int):
                                element.set(attribute_child_name, str(attribute_child_prop))
            elif isinstance(attribute_props, list):
                for element in list(root.iter(element_type)):
                    element.set(attribute_name, " ".join(map(str, attribute_props)))
            else:
                for element in list(root.iter(element_type)):
                    element.set(attribute_name, f"{attribute_props}")


class MujocoCompiler:
    world_xml_path: str
    scene_name: str
    save_xml_path: str
    robots: List[Robot]
    objects: List[Object]
    default_dict: Dict[str, List[ET.Element]]

    def __init__(self, args):
        self.world_xml_path = args.world
        print(f"World: {self.world_xml_path}")
        self.scene_name = args.name
        self.save_dir_path = os.path.join(args.save_dir, self.scene_name)
        self.save_xml_dir = args.save_dir
        self.robots = []
        self.objects = []
        self.default_dict = {}
        self.asset_dict = {"mesh": {}, "texture": {}, "material": {}}

    def build_world_xml(self, robots=Optional[Robot], objects=Optional[Object]):
        self.create_world_xml()
        tree = ET.parse(self.save_xml_path)
        root = tree.getroot()

        self.append_default_and_remove_from_root(root)
        meshdir, texturedir = remove_dirs_and_get_abs_dirs(root, self.world_xml_path)
        self.append_asset_and_remove_from_root(root, meshdir, texturedir)

        include_collision: List[str] = []

        if robots is not None:
            print("Robots:")
            robot: Robot
            for robot in robots:
                if robot.disable_self_collision == "off":
                    include_collision.append(robot.name)
                robot_xml_file = self.modify_robot_or_object(robot)
                include_element = ET.Element("include", {"file": os.path.basename(robot_xml_file)})
                root.append(include_element)
                tree.write(self.save_xml_path, encoding="utf-8", xml_declaration=True)

        if objects is not None:
            print("Objects:")
            obj: Object
            for obj in objects:
                if obj.disable_self_collision == "off":
                    include_collision.append(obj.name)
                object_xml_file = self.modify_robot_or_object(obj)
                include_element = ET.Element("include", {"file": os.path.basename(object_xml_file)})
                root.append(include_element)
                tree.write(self.save_xml_path, encoding="utf-8", xml_declaration=True)

        self.set_new_default(root)
        self.set_new_asset(root)
        tree.write(self.save_xml_path, encoding="utf-8", xml_declaration=True)

        exclude_collision(root, self.save_xml_path, include_collision)
        tree.write(self.save_xml_path, encoding="utf-8", xml_declaration=True)

        self.add_visual_and_cursor_element(root)
        ET.indent(tree, space="\t", level=0)
        tree.write(self.save_xml_path, encoding="utf-8", xml_declaration=True)

    def create_world_xml(self):
        if not os.path.exists(self.save_dir_path):
            os.makedirs(self.save_dir_path)
        self.save_xml_path = os.path.join(self.save_dir_path, self.scene_name + ".xml")
        shutil.copy(self.world_xml_path, self.save_xml_path)

    def modify_robot_or_object(self, entity: Union[Robot, Object]) -> str:
        print(f"- Name: {entity.name}")
        print(f"  Path: {entity.path}")
        print(f"  Apply: {entity.apply}")

        entity_xml_file = entity.name + ".xml"
        entity_xml_path = os.path.join(os.path.dirname(self.save_xml_path), entity_xml_file)
        shutil.copy(entity.path, entity_xml_path)

        tree = ET.parse(entity_xml_path)
        root = tree.getroot()

        meshdir, texturedir = remove_dirs_and_get_abs_dirs(root, entity.path)

        self.append_default_and_remove_from_root(root)

        self.append_asset_and_remove_from_root(root, meshdir, texturedir)

        apply_properties(root, entity.apply)

        add_prefix_and_suffix(root, entity.prefix, entity.suffix)

        ET.indent(tree, space="\t", level=0)
        tree.write(entity_xml_path, encoding="utf-8", xml_declaration=True)
        return entity_xml_path

    def append_default_and_remove_from_root(self, root: ET.Element):
        for default_element in root.findall("default"):
            for default_child_element in default_element.findall("default"):
                self.default_dict[default_child_element.get("class")] = list(default_child_element)
            root.remove(default_element)

    def append_asset_and_remove_from_root(self, root: ET.Element, meshdir: str, texturedir: str):
        for asset_element in root.findall("asset"):
            for asset_type, asset_dir in {"mesh": meshdir, "texture": texturedir}.items():
                for element in asset_element.findall(asset_type):
                    if "file" not in element.attrib:
                        continue
                    file = element.get("file")
                    if not os.path.isabs(file):
                        file = os.path.join(asset_dir, file)
                        element.set("file", file)
                    asset_element.remove(element)
                    file_name = element.get("name", os.path.splitext(os.path.basename(file))[0])
                    self.asset_dict[asset_type][file_name] = element
            for material_element in asset_element.findall("material"):
                material_name = material_element.get("name")
                self.asset_dict["material"][material_name] = material_element
                asset_element.remove(material_element)

    def set_new_default(self, root: ET.Element):
        default_parent_element = ET.Element("default")
        root.append(default_parent_element)
        for class_name, default_elements in self.default_dict.items():
            default_child_element = ET.Element("default", {"class": class_name})
            default_parent_element.append(default_child_element)
            for default_element in default_elements:
                default_child_element.append(default_element)

    def set_new_asset(self, root: ET.Element):
        asset_element = ET.Element("asset")
        root.append(asset_element)
        for _, mesh_element in self.asset_dict["mesh"].items():
            asset_element.append(mesh_element)
        for _, texture_element in self.asset_dict["texture"].items():
            asset_element.append(texture_element)
        for _, material_element in self.asset_dict["material"].items():
            asset_element.append(material_element)

    def add_visual_and_cursor_element(self, root: ET.Element):
        m = mujoco.MjModel.from_xml_path(self.save_xml_path)
        if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "cursor") != -1:
            return
        visual_element = ET.Element("visual")
        root.append(visual_element)
        global_element = ET.Element(
            "global", {"fovy": "45", "azimuth": "225", "elevation": "-30"}
        )
        visual_element.append(global_element)
        add_cursor_element(root)


def main():
    # Initialize argument parser
    parser = argparse.ArgumentParser(description="Compile MJCF from world and robots.")

    # Define arguments
    parser.add_argument("--name", help="Name of the simulation", required=True)
    parser.add_argument("--world", help="Path to world MJCF", required=True)
    parser.add_argument(
        "--robots", help="JSON string with robots' data", required=False
    )
    parser.add_argument(
        "--objects", help="JSON string with objects' data", required=False
    )
    if os.path.basename(__file__) == "mujoco_compile":
        save_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "saved")
    elif os.path.basename(__file__) == "mujoco_compile.py":
        save_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "..", "saved")
    else:
        raise RuntimeError(f"Unknown file name {os.path.basename(__file__)}")
    parser.add_argument(
        "--save_dir", help="Path to save directory", required=False, default=save_dir
    )

    # Parse arguments
    args, _ = parser.parse_known_args()

    compiler = MujocoCompiler(args)
    compiler.build_world_xml(robots=parse_robot_or_object(args.robots, Robot),
                             objects=parse_robot_or_object(args.objects, Object))
    print(f"Scene: {compiler.save_xml_path}", end="")


if __name__ == "__main__":
    main()
