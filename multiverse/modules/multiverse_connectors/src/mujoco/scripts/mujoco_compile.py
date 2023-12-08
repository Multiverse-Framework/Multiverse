#!/usr/bin/env python3.10

import argparse
import dataclasses
import json
import os
import shutil
import xml.etree.ElementTree as ET
from typing import List, Dict, Set, Any, Union, Optional

import mujoco


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


def get_body_collision_dict(xml_path: str) -> Dict[str, Set[str]]:
    body_collision_dict = {}
    m = mujoco.MjModel.from_xml_path(filename=xml_path)
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d)
    for geom1_id, geom2_id in zip(d.contact.geom1, d.contact.geom2):
        body1_id = m.geom(geom1_id).bodyid[0]
        body1_name = m.body(body1_id).name
        body2_id = m.geom(geom2_id).bodyid[0]
        body2_name = m.body(body2_id).name
        body_collision_dict[body1_name] = body_collision_dict.get(body1_name, set()).union({body2_name})
    return body_collision_dict


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
        entity.disable_self_collision = entity_data.get("disable_self_collision", "auto")
        entities[entity_name] = entity

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


def exclude_collision(root: ET.Element, xml_path: str, not_exclude_collision_bodies: List[str]) -> Dict[str, Set[str]]:
    body_collision_dict = get_body_collision_dict(xml_path)
    contact_element = ET.Element("contact")
    root.append(contact_element)
    for body1_name, body2_names in body_collision_dict.items():
        if body1_name in not_exclude_collision_bodies:
            continue
        for body2_name in body2_names:
            if body2_name in not_exclude_collision_bodies:
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
        elements = [element for worldbody in root.findall(".//worldbody") for element in
                    worldbody.findall(f".//{element_type}")]
        element_prefix = prefix.get(element_type, "")
        element_suffix = suffix.get(element_type, "")
        update_element_name(elements, element_type, element_prefix, element_suffix)

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
                if joint_name is None:
                    continue
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


def add_key_frame_element(root: ET.Element, keyframe_dict: Dict[str, ET.Element]) -> None:
    keyframe_element = ET.Element("keyframe")
    root.append(keyframe_element)
    for key_name, key_element in keyframe_dict.items():
        if key_element.get("qpos") == "":
            key_element.attrib.pop("qpos")
        if key_element.get("ctrl") == "":
            key_element.attrib.pop("ctrl")
        if key_element.get("qpos") != "" or key_element.get("ctrl") != "":
            keyframe_element.append(key_element)


def get_qpos_and_ctrl(m: mujoco.MjModel, joint_state: Dict[str, float]) -> (List[float], List[float]):
    qpos_list = []
    for qpos_id in range(m.nq):
        found_list = [qpos_id == m.jnt_qposadr[jnt_id] for jnt_id in range(m.njnt)]
        if any(found_list):
            jnt_id = found_list.index(True)
            jnt_name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, jnt_id)
            if joint_state is not None and jnt_name in joint_state:
                qpos_list.append(joint_state[jnt_name])
            else:
                qpos_list.append(m.qpos0[qpos_id])
        else:
            qpos_list.append(m.qpos0[qpos_id])

    ctrl_list = []
    for ctrl_id in range(m.nu):
        jnt_id = m.actuator_trnid[ctrl_id][0]
        jnt_name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, jnt_id)
        if joint_state is not None and jnt_name in joint_state:
            ctrl_list.append(joint_state[jnt_name])
        else:
            ctrl_list.append(0)

    return qpos_list, ctrl_list


def sort_entities(robots: Dict[str, Robot], objects: Dict[str, Object]) -> List[Union[Robot, Object]]:
    entities: List[Union[Robot, Object]] = []
    for entity in list(robots.values()) + list(objects.values()):
        if any([entity.name == other_entity.name for other_entity in entities]):
            continue
        entities.append(entity)
        attach_entities = entity.attach.values()
        for attach_entity_dict in attach_entities:
            if not isinstance(attach_entity_dict, dict):
                continue
            for attach_entity_name in attach_entity_dict.keys():
                if attach_entity_name in robots:
                    attach_entity = robots[attach_entity_name]
                elif attach_entity_name in objects:
                    attach_entity = objects[attach_entity_name]
                else:
                    raise RuntimeError(f"Unknown attach body {attach_entity_name} for {entity.name}")
                if any([attach_entity.name == other_entity.name for other_entity in entities]):
                    entities.remove(attach_entity)
                entities.append(attach_entity)
    return entities


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
        self.keyframe_dict = {}
        self.asset_dict = {"mesh": {}, "texture": {}, "material": {}}

    def build_world_xml(self, robots: Dict[str, Robot], objects: Dict[str, Object]):
        self.create_world_xml()
        tree = ET.parse(self.save_xml_path)
        root = tree.getroot()

        self.append_default_and_remove_from_root(root)
        meshdir, texturedir = remove_dirs_and_get_abs_dirs(root, self.world_xml_path)
        self.append_asset_and_remove_from_root(root, meshdir, texturedir)

        self.append_key_frame_and_remove_from_root(root, self.world_xml_path)

        not_exclude_collision_bodies: List[str] = []

        entities = sort_entities(robots, objects)

        for entity in entities:
            if entity.disable_self_collision == "off":
                not_exclude_collision_bodies.append(entity.name)
            self.modify_entity(entity)
            include_element = ET.Element("include", {"file": os.path.basename(entity.saved_path)})
            root.append(include_element)
            tree.write(self.save_xml_path, encoding="utf-8", xml_declaration=True)

        for entity in entities:
            for body_name, attach_props in entity.attach.items():
                self.apply_attach(entity, body_name, robots, objects, attach_props)

        self.set_new_default(root)
        self.set_new_asset(root)
        tree.write(self.save_xml_path, encoding="utf-8", xml_declaration=True)

        exclude_collision(root, self.save_xml_path, not_exclude_collision_bodies)
        tree.write(self.save_xml_path, encoding="utf-8", xml_declaration=True)

        add_key_frame_element(root, self.keyframe_dict)
        tree.write(self.save_xml_path, encoding="utf-8", xml_declaration=True)

        self.add_visual_and_cursor_element(root)
        ET.indent(tree, space="\t", level=0)
        tree.write(self.save_xml_path, encoding="utf-8", xml_declaration=True)

    def create_world_xml(self):
        if not os.path.exists(self.save_dir_path):
            os.makedirs(self.save_dir_path)
        self.save_xml_path = os.path.join(self.save_dir_path, self.scene_name + ".xml")
        shutil.copy(self.world_xml_path, self.save_xml_path)

    def modify_entity(self, entity: Union[Robot, Object]) -> None:
        print(f"- Name: {entity.name}")
        print(f"  Path: {entity.path}")
        print(f"  Joint state: {entity.joint_state}")
        print(f"  Apply: {entity.apply}")

        entity_xml_file = entity.name + ".xml"
        entity_xml_path = os.path.join(os.path.dirname(self.save_xml_path), entity_xml_file)
        shutil.copy(entity.path, entity_xml_path)

        tree = ET.parse(entity_xml_path)
        root = tree.getroot()

        meshdir, texturedir = remove_dirs_and_get_abs_dirs(root, entity.path)

        self.append_default_and_remove_from_root(root)

        self.append_asset_and_remove_from_root(root, meshdir, texturedir)

        self.append_key_frame_and_remove_from_root(root, entity.path, entity.joint_state)

        self.apply_properties(root, entity.path, entity.apply)

        add_prefix_and_suffix(root, entity.prefix, entity.suffix)

        ET.indent(tree, space="\t", level=0)
        tree.write(entity_xml_path, encoding="utf-8", xml_declaration=True)
        entity.saved_path = entity_xml_path

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

    def set_new_default(self, root: ET.Element) -> None:
        default_parent_element = ET.Element("default")
        root.append(default_parent_element)
        for class_name, default_elements in self.default_dict.items():
            default_child_element = ET.Element("default", {"class": class_name})
            default_parent_element.append(default_child_element)
            for default_element in default_elements:
                default_child_element.append(default_element)

    def set_new_asset(self, root: ET.Element) -> None:
        asset_element = ET.Element("asset")
        root.append(asset_element)
        for _, mesh_element in self.asset_dict["mesh"].items():
            asset_element.append(mesh_element)
        for _, texture_element in self.asset_dict["texture"].items():
            asset_element.append(texture_element)
        for _, material_element in self.asset_dict["material"].items():
            asset_element.append(material_element)

    def add_visual_and_cursor_element(self, root: ET.Element) -> None:
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

    def append_key_frame_and_remove_from_root(self, root: ET.Element, entity_path: str,
                                              joint_state: Dict[str, float] = None) -> None:
        m = mujoco.MjModel.from_xml_path(entity_path)

        qpos_list, ctrl_list = get_qpos_and_ctrl(m, joint_state)

        self.append_local_key_frame(root, qpos_list, ctrl_list)

        self.append_global_key_frame(root, qpos_list, ctrl_list)

    def apply_properties(self, root: ET.Element, entity_xml_path: str, apply: Dict[str, Dict[str, Any]]) -> None:
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

        self.apply_key_frame(entity_xml_path, apply)

    def apply_key_frame(self, entity_xml_path: str, apply: Dict[str, Dict[str, Any]]) -> None:
        m = mujoco.MjModel.from_xml_path(entity_xml_path)
        for body_name, body_attributes in apply.get("body", {}).items():
            if isinstance(body_attributes, dict) and body_attributes.get("pos") is not None:
                pos = body_attributes.get("pos")
                body_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body_name)
                qpos_id = m.body_dofadr[body_id]
                if qpos_id == -1:
                    continue
                for key_element in self.keyframe_dict.values():
                    qpos_list = key_element.get("qpos").split(" ")
                    qpos_id = len(qpos_list) - m.nq + qpos_id
                    qpos_list[qpos_id:qpos_id + 3] = pos
                    key_element.set("qpos", " ".join(map(str, qpos_list)))
            if isinstance(body_attributes, dict) and body_attributes.get("quat") is not None:
                quat = body_attributes.get("quat")
                body_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body_name)
                qpos_id = m.body_dofadr[body_id]
                qpos_num = m.body_dofnum[body_id]
                if qpos_id == -1:
                    continue
                for key_element in self.keyframe_dict.values():
                    qpos_list = key_element.get("qpos").split(" ")
                    qpos_id = len(qpos_list) - m.nq + qpos_id
                    if qpos_num == 3:
                        qpos_list[qpos_id:qpos_id + 4] = quat
                    elif qpos_num == 6:
                        qpos_list[qpos_id + 3:qpos_id + 7] = quat
                    key_element.set("qpos", " ".join(map(str, qpos_list)))

    def append_local_key_frame(self, root: ET.Element, qpos_list: List[float], ctrl_list: List[float]) -> None:
        for keyframe_element in root.findall("keyframe"):
            for key_element in keyframe_element.findall("key"):
                if key_element.get("time") is None:
                    key_element.set("time", "0")
                key_name = key_element.get("name")
                if key_name in self.keyframe_dict and self.keyframe_dict[key_name].get("qpos") is not None:
                    existing_qpos_list = self.keyframe_dict[key_name].get("qpos").split(" ")
                    qpos_list = existing_qpos_list + qpos_list
                    existing_ctrl_list = self.keyframe_dict[key_name].get("ctrl").split(" ")
                    ctrl_list = existing_ctrl_list + ctrl_list
                key_element.set("qpos", " ".join(map(str, qpos_list)))
                key_element.set("ctrl", " ".join(map(str, ctrl_list)))
                self.keyframe_dict[key_element.get("name")] = key_element

    def append_global_key_frame(self, root: ET.Element, qpos_list: List[float], ctrl_list: List[float]) -> None:
        if root.find("keyframe") is not None:
            for keyframe_element in root.findall("keyframe"):
                root.remove(keyframe_element)
        else:
            if "home" in self.keyframe_dict:
                if self.keyframe_dict["home"].get("qpos") is not None:
                    existing_qpos_list = self.keyframe_dict.get("home").get("qpos").split(" ")
                    qpos_list = existing_qpos_list + qpos_list
                    self.keyframe_dict["home"].set("qpos", " ".join(map(str, qpos_list)))
                if self.keyframe_dict["home"].get("ctrl") is not None:
                    existing_ctrl_list = self.keyframe_dict.get("home").get("ctrl").split(" ")
                    ctrl_list = existing_ctrl_list + ctrl_list
                    self.keyframe_dict["home"].set("ctrl", " ".join(map(str, ctrl_list)))
            else:
                key_element = ET.Element("key", {"name": "home",
                                                 "qpos": " ".join(map(str, qpos_list)),
                                                 "ctrl": " ".join(map(str, ctrl_list))})
                self.keyframe_dict["home"] = key_element

    def apply_attach(self, entity: Union[Robot, Object], entity_body_name, robots, objects,
                     attach_props: Dict[str, Dict[str, Any]]) -> None:
        for attach_body_name, attach_attributes in attach_props.items():
            attach_body: Optional[Union[Robot, Object]] = None
            if attach_body_name in robots:
                attach_body = robots[attach_body_name]
            elif attach_body_name in objects:
                attach_body = objects[attach_body_name]
            if attach_body is None:
                raise RuntimeError(f"Unknown attach body {attach_body_name} for {entity.name}")

            entity_path = os.path.join(os.path.dirname(self.save_xml_path), os.path.basename(entity.path))
            entity_tree = ET.parse(entity_path)
            entity_root = entity_tree.getroot()
            entity_body_found = [element for worldbody in entity_root.findall(".//worldbody")
                                 for element in worldbody.findall(".//body") if element.get("name") == entity_body_name]
            if len(entity_body_found) != 1:
                raise RuntimeError(f"Failed to find one body {entity_body_name} in {entity_path}")
            entity_body_element = entity_body_found[0]

            attach_body_path = os.path.join(os.path.dirname(self.save_xml_path), attach_body.name + ".xml")
            attach_entity_tree = ET.parse(attach_body_path)
            attach_entity_root = attach_entity_tree.getroot()

            m = mujoco.MjModel.from_xml_path(attach_body.path)
            first_body_name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, 1)
            attach_body_found = [element for worldbody in attach_entity_root.findall(".//worldbody")
                                 for element in worldbody.findall(".//body") if element.get("name") == first_body_name]
            if len(attach_body_found) != 1:
                raise RuntimeError(f"Failed to find one body {first_body_name} in {attach_body_path}")
            attach_body_element = attach_body_found[0]

            for attribute_name, attribute_data in attach_attributes.items():
                attach_body_element.set(attribute_name, " ".join(map(str, attribute_data)))

            for parent in attach_entity_root.iter():
                for child_candidate in parent:
                    if child_candidate is attach_body_element:
                        parent.remove(attach_body_element)
                        break

            ET.indent(attach_entity_tree, space="\t", level=0)
            attach_entity_tree.write(attach_body_path, encoding="utf-8", xml_declaration=True)

            entity_body_element.append(attach_body_element)
            ET.indent(entity_tree, space="\t", level=0)
            entity_tree.write(entity_path, encoding="utf-8", xml_declaration=True)


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
    compiler.build_world_xml(robots=parse_entity(args.robots, Robot),
                             objects=parse_entity(args.objects, Object))
    print(f"Scene: {compiler.save_xml_path}", end="")


if __name__ == "__main__":
    main()