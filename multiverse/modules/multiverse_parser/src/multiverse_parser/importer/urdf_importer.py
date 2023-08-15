#!/usr/bin/env python3.10

from multiverse_parser import WorldBuilder, GeomType
import os
from urdf_parser_py import urdf
from .mesh_importer import import_dae, import_obj, import_stl
import rospkg
import xml.etree.ElementTree as ET

material_dict = {}

rospack = rospkg.RosPack()

def import_from_urdf(urdf_file_path: str, with_physics: bool = True) -> WorldBuilder:
    for urdf_material in ET.parse(urdf_file_path).getroot().findall('material'):
        material_dict[urdf_material.get("name")] = tuple(map(float, urdf_material.find("color").get("rgba").split()))

    in_robot = urdf.URDF.from_xml_file(urdf_file_path)

