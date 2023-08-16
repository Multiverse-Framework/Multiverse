#!/usr/bin/env python3.10

from multiverse_parser import WorldBuilder, GeomType, JointType
import os
from urdf_parser_py import urdf
from .mesh_importer import import_dae, import_obj, import_stl
import rospkg
import xml.etree.ElementTree as ET

material_dict = {}

rospack = rospkg.RosPack()


def import_from_urdf(urdf_file_path: str, with_physics: bool = True) -> WorldBuilder:
    for urdf_material in ET.parse(urdf_file_path).getroot().findall("material"):
        material_dict[urdf_material.get("name")] = tuple(map(float, urdf_material.find("color").get("rgba").split()))

    robot: urdf.Robot = urdf.URDF.from_xml_file(urdf_file_path)

    world_builder = WorldBuilder()

    mesh_builders = set()

    root_link = robot.get_root()

    world_builder.add_body(body_name=root_link.name)
    urdf_joint: urdf.Joint
    for urdf_joint in robot.urdf_joints:
        urdf_parent_link = robot.link_map[urdf_joint.parent]
        urdf_child_link = robot.link_map[urdf_joint.child]
        if urdf_joint.type != JointType.FIXED and with_physics:
            body_builder = world_builder.add_body(body_name=urdf_child_link.name, parent_body_name=root_link.name)
        else:
            body_builder = world_builder.add_body(body_name=urdf_child_link.name, parent_body_name=urdf_parent_link.name)
        
        if with_physics:
            body_builder.enable_collision()
            
            if urdf_joint.type == "fixed":
                continue
            
            urdf_joint_name = urdf_joint.name
            if urdf_joint.type == "revolute":
                urdf_joint_type = JointType.REVOLUTE
            elif urdf_joint.type == "continuous":
                urdf_joint_type = JointType.CONTINUOUS
            elif urdf_joint.type == "prismatic":
                urdf_joint_type = JointType.PRISMATIC
            else:
                print(f"Joint {urdf_joint_name} type {urdf_joint.type} not supported.")
                continue
            # urdf_joint.origin.x