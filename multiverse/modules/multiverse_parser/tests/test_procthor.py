import unittest

import os
import re
import json
from typing import Dict, Any
import numpy
from scipy.spatial.transform import Rotation
from multiverse_parser import Configuration, InertiaSource, Factory
from multiverse_parser import (WorldBuilder,
                               BodyBuilder,
                               JointBuilder, JointType, JointProperty, get_joint_axis_and_quat,
                               GeomType, GeomProperty,
                               MeshProperty,
                               MaterialProperty)

from pxr import Usd, UsdGeom, UsdPhysics
from mujoco import MjModel, mjMINVAL
from urdf_parser_py import urdf

source_dir = "/media/giangnguyen/Storage/dataset/"


def snake_to_camel(snake_str):
    # Split the string by underscores, but keep numbers separated
    components = re.split(r'(_\d+_)|(_\d+)|(_\d+)|_', snake_str)

    # Filter out None or empty strings that might result from the split
    components = [comp for comp in components if comp]

    # Capitalize the first letter of each component except those that are purely numeric
    return ''.join(comp.title() if not comp.isdigit() else comp for comp in components)


class ProcthorImporter(Factory):
    def __init__(self, file_path: str, config: Configuration):
        super().__init__(file_path, config)
        with open(file_path) as f:
            house = json.load(f)

        self._world_builder = WorldBuilder(usd_file_path=self.tmp_usd_file_path)

        body_builder = self._world_builder.add_body(body_name="house_4")

        objects = house["objects"]
        for obj in objects:
            self.import_object("house_4", obj)

    def import_object(self, parent_body_name: str, obj: Dict[str, Any]) -> None:
        body_name = obj["id"].replace("|", "_")
        body_builder = self._world_builder.add_body(body_name=body_name, parent_body_name="house_4")

        position = obj.get("position", {"x": 0, "y": 0, "z": 0})
        position_vec = numpy.array([position["x"], position["y"], position["z"]])
        rotation = obj.get("rotation", {"x": 0, "y": 0, "z": 0})
        rotation_mat = Rotation.from_euler("xyz", [rotation["x"], rotation["y"], rotation["z"]],
                                           degrees=True)

        x_90_rotation_matrix = numpy.array([[1, 0, 0],
                                            [0, 0, -1],
                                            [0, 1, 0]])
        position_vec = numpy.dot(x_90_rotation_matrix, position_vec)
        rotation_quat = Rotation.from_matrix(numpy.dot(x_90_rotation_matrix.T, numpy.dot(rotation_mat.as_matrix(), x_90_rotation_matrix))).as_quat()

        body_builder.set_transform(pos=position_vec, quat=rotation_quat)

        if "assetId" not in obj:
            return None

        asset_id = obj["assetId"]
        self.import_asset(body_builder, asset_id)

        for child in obj.get("children", {}):
            self.import_object(body_name, child)

    def import_asset(self, body_builder: BodyBuilder, asset_name: str) -> None:
        asset_name = snake_to_camel(asset_name)
        print("Importing asset:", asset_name)
        asset_path = os.path.join(source_dir, "grp_objects", asset_name)
        if os.path.exists(asset_path):
            asset_path = os.path.join(asset_path, f"{asset_name}.stl")
            if not os.path.exists(asset_path):
                raise FileNotFoundError("File not found:", asset_path)
        else:
            for root, dirs, files in os.walk(os.path.join(source_dir, "single_objects")):
                for file in files:
                    file_name = os.path.splitext(file)[0]
                    if asset_name in file_name:
                        asset_path = os.path.join(root, file)
                        break
            if not os.path.exists(asset_path):
                raise FileNotFoundError("File not found:", asset_name)

        tmp_usd_mesh_file_path, tmp_origin_mesh_file_path = self.import_mesh(
            mesh_file_path=asset_path, merge_mesh=True)
        mesh_stage = Usd.Stage.Open(tmp_usd_mesh_file_path)
        for mesh_prim in [prim for prim in mesh_stage.Traverse() if prim.IsA(UsdGeom.Mesh)]:
            mesh_name = mesh_prim.GetName()
            mesh_path = mesh_prim.GetPath()
            mesh_property = MeshProperty.from_mesh_file_path(mesh_file_path=tmp_usd_mesh_file_path,
                                                             mesh_path=mesh_path)
            geom_property = GeomProperty(geom_type=GeomType.MESH,
                                         is_visible=True,
                                         is_collidable=True)
            geom_builder = body_builder.add_geom(geom_name=f"SM_{asset_name}",
                                                 geom_property=geom_property)
            geom_builder.add_mesh(mesh_name=mesh_name, mesh_property=mesh_property)


class TestProcthor(unittest.TestCase):
    def test_import_json(self):
        house_file_path = os.path.join(source_dir, "house_4.json")
        config = Configuration()
        factory = ProcthorImporter(file_path=house_file_path, config=config)

        house_usd_file_path = os.path.join(source_dir, "house_4", "house_4.usda")
        factory.save_tmp_model(house_usd_file_path)


if __name__ == '__main__':
    unittest.main()
