#!/usr/bin/env python3

import os
import xml.etree.ElementTree as ET
from typing import Dict

import mujoco
import numpy

from multiverse_simulator import MultiverseSimulatorCompiler, Robot, Object, multiverse_simulator_compiler_main


def fix_mesh_and_texture_paths(spec: mujoco.MjSpec, default_path: str):
    meshdir_abs = spec.meshdir
    if not os.path.isabs(meshdir_abs):
        meshdir_abs = os.path.join(default_path, meshdir_abs)
    texturedir_abs = spec.texturedir
    if not os.path.isabs(texturedir_abs):
        texturedir_abs = os.path.join(default_path, texturedir_abs)

    for mesh in spec.meshes:
        if not os.path.isabs(mesh.file):
            mesh.file = os.path.join(meshdir_abs, mesh.file)
    for texture in spec.meshes:
        if not os.path.isabs(texture.file):
            texture.file = os.path.join(texturedir_abs, texture.file)


def add_entity(entities: Dict[str, Robot | Object], home_key, worldbody_frame: mujoco.MjsFrame):
    for entity_name, entity in entities.items():
        entity_spec: mujoco.MjSpec = mujoco.MjSpec.from_file(filename=entity.path)

        fix_mesh_and_texture_paths(entity_spec, os.path.dirname(entity.path))

        for body_name, body_apply in entity.apply.get("body", {}).items():
            if isinstance(body_apply, dict):
                applied_body = entity_spec.find_body(body_name)
                for apply_name, apply_data in body_apply.items():
                    setattr(applied_body, apply_name, apply_data)
            else:
                for applied_body in entity_spec.bodies:
                    setattr(applied_body, body_name, body_apply)

        if not any(key.name == "home" for key in entity_spec.keys):
            entity_spec.add_key(name="home")
        entity_home_key = [key for key in entity_spec.keys if key.name == "home"][0]
        home_key.qpos = numpy.append(home_key.qpos, entity_home_key.qpos)
        home_key.ctrl = numpy.append(home_key.ctrl, entity_home_key.ctrl)

        entity_model = entity_spec.compile()

        if entity.disable_self_collision != "off":
            entity_data = mujoco.MjData(entity_model)
            mujoco.mj_step(entity_model, entity_data)
            if entity.disable_self_collision == "auto":
                body_collision_dict = {}
                for geom1_id, geom2_id in zip(entity_data.contact.geom1, entity_data.contact.geom2):
                    body1_id = entity_model.geom(geom1_id).bodyid[0]
                    body1_name = entity_model.body(body1_id).name
                    body2_id = entity_model.geom(geom2_id).bodyid[0]
                    body2_name = entity_model.body(body2_id).name
                    body_collision_dict[body1_name] = body_collision_dict.get(body1_name, set()).union({body2_name})
                for body1_name, body2_names in body_collision_dict.items():
                    for body2_name in body2_names:
                        if not any(exclude.bodyname1 in {body1_name, body2_name} or
                                   exclude.bodyname2 in {body1_name, body2_name} for exclude in entity_spec.excludes):
                            entity_spec.add_exclude(name=f"{body1_name}_{body2_name}",
                                                    bodyname1=body1_name,
                                                    bodyname2=body2_name)
            elif entity.disable_self_collision == "on":
                for body_1 in entity_spec.bodies:
                    for body_2 in entity_spec.bodies:
                        if body_1 != body_2:
                            entity_spec.add_exclude(name=f"{body_1.name}_{body_2.name}",
                                                    bodyname1=body_1.name,
                                                    bodyname2=body_2.name)
            entity_model = entity_spec.compile()

        entity_root_name = entity_model.body(1).name
        child_body = entity_spec.find_body(entity_root_name)
        worldbody_frame.attach_body(child_body, f"{entity_name}_", f"")


def fix_prefix_and_suffix_each(entity_element_name: str, entity_prefix: str, entity_suffix: str,
                               entity_name: str) -> str:
    if entity_element_name[:len(entity_name) + 1] == f"{entity_name}_":
        entity_element_name = entity_prefix + entity_element_name[len(entity_name) + 1:]
    entity_element_name = entity_element_name + entity_suffix
    return entity_element_name


class MujocoCompiler(MultiverseSimulatorCompiler):
    name: str = "mujoco"
    ext: str = "xml"
    world_spec: mujoco.MjSpec = None

    def __init__(self, args):
        super().__init__(args)

    def build_world(self, robots: Dict[str, Robot], objects: Dict[str, Object], multiverse_params: Dict[str, Dict]):
        self.world_spec = mujoco.MjSpec.from_file(filename=self.save_file_path)

        fix_mesh_and_texture_paths(self.world_spec, os.path.dirname(self.world_path))

        if not any(key.name == "home" for key in self.world_spec.keys):
            self.world_spec.add_key(name="home")
        home_key = [key for key in self.world_spec.keys if key.name == "home"][0]
        home_key.time = 0.0

        worldbody_frame = self.world_spec.worldbody.add_frame()

        add_entity(robots, home_key, worldbody_frame)
        self.fix_prefix_and_suffix(robots)
        add_entity(objects, home_key, worldbody_frame)
        self.fix_prefix_and_suffix(objects)

        self.world_spec.compile()
        xml_string = self.world_spec.to_xml()
        with open(self.save_file_path, "w") as f:
            f.write(xml_string)

        if multiverse_params != {}:
            tree = ET.parse(self.save_file_path)
            root = tree.getroot()
            include_extension = f"""
                <extension>
                    <plugin plugin="mujoco.multiverse_connector">
                          <instance name="mujoco_client">
                                <config key="host" value="{multiverse_params['host']}"/>
                                <config key="server_port" value="{multiverse_params['server_port']}"/>
                                <config key="client_port" value="{multiverse_params['client_port']}"/>
                                <config key="world_name" value="{multiverse_params['world_name']}"/>
                                <config key="simulation_name" value="{multiverse_params['simulation_name']}"/>
                                <config key="send" value="{multiverse_params['send']}" />
                                <config key="receive" value="{multiverse_params['receive']}" />
                          </instance>
                    </plugin>
                </extension>
                """
            root.append(ET.fromstring(include_extension))
            ET.indent(root)
            file_xml_string = ET.tostring(root, encoding='unicode', method='xml')
            with open(self.save_file_path, "w") as f:
                f.write(file_xml_string)

    def fix_prefix_and_suffix(self, entities: Dict[str, Robot | Object]):
        for entity_name, entity in entities.items():
            for entity_type in ["body", "joint", "geom", "actuator"]:
                entity_prefix = entity.prefix.get(entity_type, "")
                entity_suffix = entity.suffix.get(entity_type, "")
                if entity_prefix == f"{entity_name}_" and entity_suffix == "":
                    continue
                if entity_type == "body":
                    for body in self.world_spec.bodies:
                        body.name = fix_prefix_and_suffix_each(body.name, entity_prefix, entity_suffix, entity_name)
                    for exclude in self.world_spec.excludes:
                        exclude.bodyname1 = fix_prefix_and_suffix_each(exclude.bodyname1, entity_prefix, entity_suffix,
                                                                       entity_name)
                        exclude.bodyname2 = fix_prefix_and_suffix_each(exclude.bodyname2, entity_prefix, entity_suffix,
                                                                       entity_name)
                elif entity_type == "geom":
                    for geom in self.world_spec.geoms:
                        geom.name = fix_prefix_and_suffix_each(geom.name, entity_prefix, entity_suffix, entity_name)
                elif entity_type == "actuator":
                    for actuator in self.world_spec.actuators:
                        actuator.name = fix_prefix_and_suffix_each(actuator.name, entity_prefix, entity_suffix,
                                                                   entity_name)
                elif entity_type == "joint":
                    if len(self.world_spec.tendons) > 0 or len(
                            self.world_spec.actuators) > 0:  # TODO: Waiting for MuJoCo update
                        self.world_spec.compile()
                        xml_string = self.world_spec.to_xml()
                        with open(self.save_file_path, "w") as f:
                            f.write(xml_string)

                        tree = ET.parse(self.save_file_path)
                        root = tree.getroot()
                        for tendon_element in root.findall("tendon"):
                            for fixed_element in tendon_element.findall("fixed"):
                                for joint_element in fixed_element.findall("joint"):
                                    joint_name = joint_element.attrib["joint"]
                                    joint_name = fix_prefix_and_suffix_each(joint_name, entity_prefix, entity_suffix,
                                                                            entity_name)
                                    joint_element.set("joint", joint_name)
                        for actuator_element in root.findall("actuator"):
                            for general_element in actuator_element.findall("general"):
                                if "joint" not in general_element.attrib:
                                    continue
                                joint_name = general_element.attrib["joint"]
                                joint_name = fix_prefix_and_suffix_each(joint_name, entity_prefix, entity_suffix,
                                                                        entity_name)
                                general_element.set("joint", joint_name)
                        ET.indent(root)
                        file_xml_string = ET.tostring(root, encoding='unicode', method='xml')
                        with open(self.save_file_path, "w") as f:
                            f.write(file_xml_string)
                        self.world_spec = mujoco.MjSpec.from_file(filename=self.save_file_path)

                    for joint in self.world_spec.joints:
                        joint.name = fix_prefix_and_suffix_each(joint.name, entity_prefix, entity_suffix, entity_name)
                    for equality in self.world_spec.equalities:
                        if equality.type == mujoco.mjtEq.mjEQ_JOINT:
                            equality.name1 = fix_prefix_and_suffix_each(equality.name1, entity_prefix, entity_suffix,
                                                                        entity_name)
                            equality.name2 = fix_prefix_and_suffix_each(equality.name2, entity_prefix, entity_suffix,
                                                                        entity_name)


if __name__ == "__main__":
    multiverse_simulator_compiler_main(MujocoCompiler)
