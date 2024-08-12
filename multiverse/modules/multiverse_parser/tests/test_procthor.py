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
from multiverse_parser import MjcfExporter, UrdfExporter

from pxr import Usd, UsdGeom, UsdPhysics
from mujoco import MjModel, mjMINVAL
from urdf_parser_py import urdf

house_name = "house_5"

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

        body_builder = self._world_builder.add_body(body_name=house_name)

        objects = house["objects"]
        for obj in objects:
            self.import_object(house_name, obj)

        walls = house["walls"]
        for wall_id, wall in enumerate(walls):
            self.import_wall(wall, wall_id)

        doors = house["doors"]
        for door_id, door in enumerate(doors):
            self.import_door(door, door_id)

    def import_object(self, parent_body_name: str, obj: Dict[str, Any]) -> None:
        body_name = obj["id"].replace("|", "_")
        body_builder = self._world_builder.add_body(body_name=body_name, parent_body_name=house_name)

        position = obj.get("position", {"x": 0, "y": 0, "z": 0})
        position_vec = numpy.array([position["x"], position["y"], position["z"]])
        rotation = obj.get("rotation", {"x": 0, "y": 0, "z": 0})
        rotation_mat = Rotation.from_euler("xyz", [rotation["x"], rotation["y"], rotation["z"]],
                                           degrees=True)

        x_90_rotation_matrix = numpy.array([[1, 0, 0],
                                            [0, 0, -1],
                                            [0, 1, 0]])
        position_vec = numpy.dot(x_90_rotation_matrix, position_vec)
        rotation_quat = Rotation.from_matrix(
            numpy.dot(x_90_rotation_matrix, numpy.dot(rotation_mat.as_matrix(), x_90_rotation_matrix.T))).as_quat()

        body_builder.set_transform(pos=position_vec, quat=rotation_quat)

        if "assetId" not in obj:
            return None

        asset_id = obj["assetId"]
        asset_id = re.sub(r'(\d+)x(\d+)', r'\1_X_\2', asset_id)
        self.import_asset(body_builder, asset_id)

        for child in obj.get("children", {}):
            self.import_object(body_name, child)

    def import_wall(self, wall: Dict[str, Any], wall_id: int) -> None:
        body_name = f"Wall_{wall_id}"
        body_builder = self._world_builder.add_body(body_name=body_name, parent_body_name=house_name)

        x_90_rotation_matrix = numpy.array([[1, 0, 0],
                                            [0, 0, -1],
                                            [0, 1, 0]])
        rotation_quat = Rotation.from_matrix(x_90_rotation_matrix).as_quat()

        body_builder.set_transform(quat=rotation_quat)

        point_1 = [wall["polygon"][0]["x"], wall["polygon"][0]["y"], wall["polygon"][0]["z"]]
        point_2 = [wall["polygon"][1]["x"], wall["polygon"][1]["y"], wall["polygon"][1]["z"]]
        point_3 = [wall["polygon"][2]["x"], wall["polygon"][2]["y"], wall["polygon"][2]["z"]]
        point_4 = [wall["polygon"][3]["x"], wall["polygon"][3]["y"], wall["polygon"][3]["z"]]

        points = numpy.array([point_1, point_2, point_3, point_4])

        # Compute normal from faces
        normal_1 = numpy.cross(points[1] - points[0], points[2] - points[0])
        normal_2 = numpy.cross(points[3] - points[1], points[2] - points[1])
        normal = (normal_1 + normal_2) / 2
        normal /= numpy.linalg.norm(normal)

        if normal[0] != 0:
            x = point_1[0]
            dx = -0.05 * normal[0]
            y = (point_1[1] + point_3[1]) / 2.0
            dy = (point_3[1] - point_1[1]) / 2.0
            z = (point_1[2] + point_2[2]) / 2.0
            dz = (point_2[2] - point_1[2]) / 2.0
        elif normal[2] != 0:
            x = (point_2[0] + point_3[0]) / 2.0
            dx = (point_3[0] - point_2[0]) / 2.0
            y = (point_1[1] + point_3[1]) / 2.0
            dy = (point_3[1] - point_1[1]) / 2.0
            z = point_1[2]
            dz = -0.05 * normal[2]
        else:
            raise ValueError("Invalid normal")

        points = numpy.array(
            [[x - dx, y - dy, z - dz], [x - dx, y - dy, z + dz], [x - dx, y + dy, z - dz], [x - dx, y + dy, z + dz],
             [x + dx, y - dy, z - dz], [x + dx, y - dy, z + dz], [x + dx, y + dy, z - dz],
             [x + dx, y + dy, z + dz]])
        normals = numpy.array([[-1, 0, 0], [-1, 0, 0], [-1, 0, 0],
                               [0, 1, 0], [0, 1, 0], [0, 1, 0],
                               [1, 0, 0], [1, 0, 0], [1, 0, 0],
                               [0, -1, 0], [0, -1, 0], [0, -1, 0],
                               [0, 0, -1], [0, 0, -1], [0, 0, -1],
                               [0, 0, 1], [0, 0, 1], [0, 0, 1],
                               [-1, 0, 0], [-1, 0, 0], [-1, 0, 0],
                               [0, 1, 0], [0, 1, 0], [0, 1, 0],
                               [1, 0, 0], [1, 0, 0], [1, 0, 0],
                               [0, -1, 0], [0, -1, 0], [0, -1, 0],
                               [0, 0, -1], [0, 0, -1], [0, 0, -1],
                               [0, 0, 1], [0, 0, 1], [0, 0, 1]])
        face_vertex_counts = numpy.array([3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3])
        face_vertex_indices = numpy.array(
            [[1, 2, 0], [3, 6, 2],
             [7, 4, 6], [5, 0, 4],
             [6, 0, 2], [3, 5, 7],
             [1, 3, 2], [3, 7, 6],
             [7, 5, 4], [5, 1, 0],
             [6, 4, 0], [3, 1, 5]])

        mesh_file_name = f"Wall_{wall_id}"

        mesh_property = MeshProperty(points=points,
                                     normals=normals,
                                     face_vertex_counts=face_vertex_counts,
                                     face_vertex_indices=face_vertex_indices,
                                     mesh_file_name=mesh_file_name)
        geom_property = GeomProperty(geom_type=GeomType.MESH,
                                     is_visible=True,
                                     is_collidable=True)
        geom_builder = body_builder.add_geom(geom_name=f"{body_name}",
                                             geom_property=geom_property)
        geom_builder.add_mesh(mesh_name=f"SM_{body_name}", mesh_property=mesh_property)

    def import_door(self, door: Dict[str, Any], door_id : int) -> None:
        body_name = f"Door_{door_id}"
        body_builder = self._world_builder.add_body(body_name=body_name, parent_body_name=house_name)

        position = door.get("assetPosition", {"x": 0, "y": 0, "z": 0})
        position_vec = numpy.array([position["x"], position["z"], position["y"]])

        x_90_rotation_matrix = numpy.array([[1, 0, 0],
                                            [0, 0, -1],
                                            [0, 1, 0]])
        rotation_quat = Rotation.from_matrix(x_90_rotation_matrix).as_quat()

        body_builder.set_transform(pos=position_vec)

        if "assetId" not in door:
            return None

        asset_id = door["assetId"]
        asset_id = re.sub(r'(\d+)x(\d+)', r'\1_X_\2', asset_id)
        self.import_asset(body_builder, asset_id)

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
                    if file.endswith('.stl') and asset_name in file_name:
                        asset_path = os.path.join(root, file)
                        break
            if not os.path.exists(asset_path):
                print(f"Asset not found: {asset_name}, try to remove the last numbers")
                asset_name = re.sub(r'_\d+$', '', asset_name)
                asset_path = os.path.join(source_dir, "grp_objects", asset_name)
                if os.path.exists(asset_path):
                    asset_path = os.path.join(asset_path, f"{asset_name}.stl")
                else:
                    for root, dirs, files in os.walk(os.path.join(source_dir, "single_objects")):
                        if os.path.exists(asset_path):
                            break
                        for file in files:
                            file_name = os.path.splitext(file)[0]
                            if file.endswith('.stl') and asset_name in file_name:
                                asset_path = os.path.join(root, file)
                                break
                    if not os.path.exists(asset_path):
                        print(f"Asset not found: {asset_name}, try to remove the last numbers")
                        asset_name = re.sub(r'_\d+$', '', asset_name)
                        asset_path = os.path.join(source_dir, "grp_objects", asset_name)
                        if os.path.exists(asset_path):
                            asset_path = os.path.join(asset_path, f"{asset_name}.stl")
                        else:
                            for root, dirs, files in os.walk(os.path.join(source_dir, "single_objects")):
                                if os.path.exists(asset_path):
                                    break
                                for file in files:
                                    file_name = os.path.splitext(file)[0]
                                    if file.endswith('.stl') and asset_name in file_name:
                                        asset_path = os.path.join(root, file)
                                        break
                            if not os.path.exists(asset_path):
                                for root, dirs, files in os.walk(os.path.join(source_dir, "grp_objects")):
                                    if os.path.exists(asset_path):
                                        break
                                    for file in files:
                                        file_name = os.path.splitext(file)[0]
                                        if file.endswith('.stl') and asset_name in file_name:
                                            asset_path = os.path.join(root, file)
                                            break
                                if not os.path.exists(asset_path):
                                    for root, dirs, files in os.walk(os.path.join(source_dir, "grp_objects")):
                                        if os.path.exists(asset_path):
                                            break
                                        for file in files:
                                            file_name = os.path.splitext(file)[0]
                                            if file.endswith('.stl') and file_name in asset_name:
                                                asset_path = os.path.join(root, file)
                                                break
                                    if not os.path.exists(asset_path):
                                        for root, dirs, files in os.walk(os.path.join(source_dir, "single_objects")):
                                            if os.path.exists(asset_path):
                                                break
                                            for file in files:
                                                file_name = os.path.splitext(file)[0]
                                                if file.endswith('.stl') and file_name in asset_name:
                                                    asset_path = os.path.join(root, file)
                                                    break
                                        if not os.path.exists(asset_path):
                                            print(f"Asset not found: {asset_name}")
                                            return None

        asset_dir = os.path.dirname(asset_path)
        mesh_idx = 0
        body_name = body_builder.xform.GetPrim().GetName()
        if os.path.basename(os.path.dirname(asset_dir)) == "grp_objects" or "Pen" in asset_name:
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
                geom_builder = body_builder.add_geom(geom_name=f"SM_{body_name}_{asset_name}",
                                                     geom_property=geom_property)
                geom_builder.add_mesh(mesh_name=mesh_name, mesh_property=mesh_property)
        else:
            for root, dirs, files in os.walk(asset_dir):
                for file in files:
                    file_name = os.path.splitext(file)[0]
                    if file.endswith('.stl') and asset_name in file_name:
                        asset_path = os.path.join(root, file)
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
                            geom_builder = body_builder.add_geom(geom_name=f"SM_{body_name}_{asset_name}_{mesh_idx}",
                                                                 geom_property=geom_property)
                            geom_builder.add_mesh(mesh_name=mesh_name, mesh_property=mesh_property)
                            mesh_idx += 1


class TestProcthor(unittest.TestCase):
    def test_import_json(self):
        house_file_path = os.path.join(source_dir, f"{house_name}.json")
        config = Configuration()
        factory = ProcthorImporter(file_path=house_file_path, config=config)

        house_usd_file_path = os.path.join(source_dir, house_name, f"{house_name}.usda")
        factory.save_tmp_model(house_usd_file_path)

        # Export to MJCF
        # exporter = MjcfExporter(file_path=house_usd_file_path.replace(".usda", ".xml"),
        #                         factory=factory)
        # exporter.build()
        # exporter.export(keep_usd=False)


if __name__ == '__main__':
    unittest.main()
