#!/usr/bin/env python3

import mujoco
import os
from typing import Dict
import xml.etree.ElementTree as ET
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


def add_entity(entities: Dict[str, Robot | Object], home_key, worldbody_frame):
    for entity_name, entity in entities.items():
        entity_spec = mujoco.MjSpec.from_file(filename=entity.path)

        fix_mesh_and_texture_paths(entity_spec, os.path.dirname(entity.path))

        for body_name, body_apply in entity.apply["body"].items():
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
        entity_root_name = entity_model.body(1).name
        child_body = entity_spec.find_body(entity_root_name)
        worldbody_frame.attach_body(child_body, f"{entity_name}_", "")


class MujocoCompiler(MultiverseSimulatorCompiler):
    name: str = "mujoco"
    ext: str = "xml"

    def __init__(self, args):
        super().__init__(args)

    def build_world(self, robots: Dict[str, Robot], objects: Dict[str, Object], multiverse_params: Dict[str, Dict]):
        world_spec = mujoco.MjSpec.from_file(filename=self.save_file_path)

        fix_mesh_and_texture_paths(world_spec, os.path.dirname(self.world_path))

        if not any(key.name == "home" for key in world_spec.keys):
            world_spec.add_key(name="home")
        home_key = [key for key in world_spec.keys if key.name == "home"][0]
        home_key.time = 0.0

        worldbody_frame = world_spec.worldbody.add_frame()

        add_entity(robots, home_key, worldbody_frame)
        add_entity(objects, home_key, worldbody_frame)

        world_spec.compile()
        xml_string = world_spec.to_xml()
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


if __name__ == "__main__":
    multiverse_simulator_compiler_main(MujocoCompiler)
