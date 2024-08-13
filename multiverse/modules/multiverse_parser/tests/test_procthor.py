import unittest

import os
import re
import json
from typing import Dict, Any, List, Optional
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
import random

house_name = "house_5"

source_dir = "/media/giangnguyen/Storage/dataset/"

ignore_objects = ["Painting"]


def snake_to_camel(snake_str):
    # Split the string by underscores, but keep numbers separated
    components = re.split(r'(_\d+_)|(_\d+)|(_\d+)|_', snake_str)

    # Filter out None or empty strings that might result from the split
    components = [comp for comp in components if comp]

    # Capitalize the first letter of each component except those that are purely numeric
    camel_str = ''.join(comp.title() if not comp.isdigit() else comp for comp in components)

    camel_str = re.sub(r'(\d+)', r'_\1_', camel_str)

    camel_str = camel_str.strip('_').replace('.', '_').replace(' ', '_').replace('__', '_')

    return camel_str


def get_asset_paths(asset_name: str) -> List[str]:
    asset_name = asset_name.replace("Bathroom", "").replace("Photo", "").replace("Painting", "")
    print("Importing asset:", asset_name)
    asset_path = os.path.join(source_dir, "grp_objects", asset_name)

    asset_paths = []
    if os.path.exists(asset_path):
        asset_path = os.path.join(asset_path, f"{asset_name}.stl")
        if not os.path.exists(asset_path):
            raise FileNotFoundError("File not found:", asset_path)
        asset_paths.append(asset_path)
        return asset_paths
    else:
        for root, dirs, files in os.walk(os.path.join(source_dir, "single_objects")):
            for file in files:
                file_name = os.path.splitext(file)[0]
                if file.endswith('.stl') and asset_name in file_name:
                    asset_path = os.path.join(root, file)
                    if os.path.exists(asset_path):
                        asset_paths.append(asset_path)
        if len(asset_paths) > 0:
            return asset_paths

        asset_new_name = re.sub(r'_\d+$', '', asset_name)
        if asset_new_name[-1] == '_':
            asset_new_name = asset_new_name[:-1]
        if asset_new_name != asset_name:
            print(f"Asset not found: {asset_name}, try to remove the last numbers")
            return get_asset_paths(asset_new_name)

        for root, dirs, files in os.walk(os.path.join(source_dir, "grp_objects")):
            for file in files:
                file_name = os.path.splitext(file)[0]
                if file.endswith('.stl') and asset_name in file_name:
                    asset_path = os.path.join(root, file)
                    if os.path.exists(asset_path):
                        asset_paths.append(asset_path)
        if len(asset_paths) > 0:
            return asset_paths

        for root, dirs, files in os.walk(os.path.join(source_dir, "grp_objects")):
            for file in files:
                file_name = os.path.splitext(file)[0]
                if file.endswith('.stl') and file_name in asset_name:
                    asset_path = os.path.join(root, file)
                    if os.path.exists(asset_path):
                        asset_paths.append(asset_path)
        if len(asset_paths) > 0:
            return asset_paths

        for root, dirs, files in os.walk(os.path.join(source_dir, "single_objects")):
            for file in files:
                file_name = os.path.splitext(file)[0]
                if file.endswith('.stl') and file_name in asset_name:
                    asset_path = os.path.join(root, file)
                    if os.path.exists(asset_path):
                        asset_paths.append(asset_path)
        if len(asset_paths) > 0:
            return asset_paths

    if asset_name[-1].isupper():
        print(f"Asset not found: {asset_name}, try to remove the last capital character")
        asset_new_name = asset_name[:-1]
        return get_asset_paths(asset_new_name)

    return asset_paths


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
        doors = house["doors"]
        walls_with_door = {}
        for door_id, door in enumerate(doors):
            self.import_door(door, door_id, walls, walls_with_door)

        for wall_id, wall in enumerate(walls):
            self.import_wall(wall, wall_id, walls_with_door)

    def import_object(self, parent_body_name: str, obj: Dict[str, Any]) -> None:
        body_name = obj["id"].replace("|", "_")

        if any([ignore_object in body_name for ignore_object in ignore_objects]):
            return None

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

    def import_wall(self, wall: Dict[str, Any], wall_id: int, walls_with_door: Dict[str, Any]) -> None:
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

        if wall["id"] not in walls_with_door:
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

        else:
            door = walls_with_door[wall["id"]]
            hole_height = door["holePolygon"][1]["y"]
            if point_1[1] < point_3[1]:
                y = [(point_1[1] + hole_height) / 2.0, (hole_height + point_3[1]) / 2.0,
                     (point_1[1] + hole_height) / 2.0]
                dy = [(hole_height - point_1[1]) / 2.0, (point_3[1] - hole_height) / 2.0,
                      (hole_height - point_1[1]) / 2.0]
            else:
                y = [(point_3[1] + hole_height) / 2.0, (hole_height + point_1[1]) / 2.0,
                     (point_3[1] + hole_height) / 2.0]
                dy = [(hole_height - point_3[1]) / 2.0, (point_1[1] - hole_height) / 2.0,
                      (hole_height - point_3[1]) / 2.0]
            if normal[0] != 0:
                z = [(point_1[2] + point_2[2]) / 2.0] * 3
                dz = [(point_2[2] - point_1[2]) / 2.0] * 3

                dz[0] = door["holePolygon"][0]["y"] / 2.0
                if point_1[2] > point_2[2]:
                    z[0] = point_2[2] + dz[0]
                    dz[2] = (point_1[2] - point_2[2] - door["holePolygon"][1]["y"]) / 2.0
                    z[2] = point_1[2] - dz[2]
                else:
                    z[0] = point_1[2] + dz[0]
                    dz[2] = (point_2[2] - point_1[2] - door["holePolygon"][1]["y"]) / 2.0
                    z[2] = point_2[2] - dz[2]

                x = [point_1[0]] * 3
                dx = [-0.05 * normal[0]] * 3
            elif normal[2] != 0:
                x = [(point_2[0] + point_3[0]) / 2.0] * 3
                dx = [(point_3[0] - point_2[0]) / 2.0] * 3

                dx[0] = door["holePolygon"][0]["x"] / 2.0
                if point_2[0] > point_3[0]:
                    x[0] = point_3[0] + dx[0]
                    dx[2] = (point_2[0] - point_3[0] - door["holePolygon"][1]["x"]) / 2.0
                    x[2] = point_2[0] - dx[2]
                else:
                    x[0] = point_2[0] + dx[0]
                    dx[2] = (point_3[0] - point_2[0] - door["holePolygon"][1]["x"]) / 2.0
                    x[2] = point_3[0] - dx[2]

                z = [point_1[2]] * 3
                dz = [-0.05 * normal[2]] * 3
            else:
                raise ValueError("Invalid normal")

            for idx in range(3):
                points = numpy.array(
                    [[x[idx] - dx[idx], y[idx] - dy[idx], z[idx] - dz[idx]],
                     [x[idx] - dx[idx], y[idx] - dy[idx], z[idx] + dz[idx]],
                     [x[idx] - dx[idx], y[idx] + dy[idx], z[idx] - dz[idx]],
                     [x[idx] - dx[idx], y[idx] + dy[idx], z[idx] + dz[idx]],
                     [x[idx] + dx[idx], y[idx] - dy[idx], z[idx] - dz[idx]],
                     [x[idx] + dx[idx], y[idx] - dy[idx], z[idx] + dz[idx]],
                     [x[idx] + dx[idx], y[idx] + dy[idx], z[idx] - dz[idx]],
                     [x[idx] + dx[idx], y[idx] + dy[idx], z[idx] + dz[idx]]])

                mesh_file_name = f"Wall_{wall_id}_{idx}"

                mesh_property = MeshProperty(points=points,
                                             normals=normals,
                                             face_vertex_counts=face_vertex_counts,
                                             face_vertex_indices=face_vertex_indices,
                                             mesh_file_name=mesh_file_name)
                geom_property = GeomProperty(geom_type=GeomType.MESH,
                                             is_visible=True,
                                             is_collidable=True)
                geom_builder = body_builder.add_geom(geom_name=f"{body_name}_{idx}",
                                                     geom_property=geom_property)
                geom_builder.add_mesh(mesh_name=f"SM_{body_name}_{idx}", mesh_property=mesh_property)

    def import_door(self, door: Dict[str, Any], door_id: int, walls: List[Dict[str, Any]],
                    walls_with_door: Dict) -> None:
        body_name = f"Door_{door_id}"
        body_builder = self._world_builder.add_body(body_name=body_name, parent_body_name=house_name)

        position = door.get("assetPosition", {"x": 0, "y": 0, "z": 0})
        position_vec = numpy.array([0, 0, position["y"]])

        wall0 = door.get("wall0", None)
        wall1 = door.get("wall1", None)
        if wall0 is None:
            raise ValueError(f"Door {door_id} does not have wall0")
        if wall1 is None:
            raise ValueError(f"Door {door_id} does not have wall1")

        for wall in walls:
            if wall["id"] == wall1:
                walls_with_door[wall1] = door
                break
        else:
            raise ValueError(f"Wall {wall1} not found")

        for wall in walls:
            if wall["id"] == wall0:
                walls_with_door[wall0] = door
                break
        else:
            raise ValueError(f"Wall {wall0} not found")

        if wall["polygon"][0]["x"] == wall["polygon"][1]["x"]:
            position_vec[0] = position["z"] + wall["polygon"][0]["x"]
            position_vec[1] = -position["x"] - (
                wall["polygon"][0]["z"] if wall["polygon"][0]["z"] < wall["polygon"][1]["z"] else wall["polygon"][1][
                    "z"])
            body_builder.set_transform(pos=position_vec,
                                       quat=Rotation.from_euler("xyz", [0, 0, 90], degrees=True).as_quat())
        elif wall["polygon"][0]["z"] == wall["polygon"][1]["z"]:
            position_vec[0] = position["x"] + (
                wall["polygon"][0]["x"] if wall["polygon"][0]["x"] < wall["polygon"][1]["x"] else wall["polygon"][1][
                    "x"])
            position_vec[1] = position["z"] - wall["polygon"][0]["z"]
            body_builder.set_transform(pos=position_vec)
        else:
            raise ValueError(f"Invalid wall {wall}")

        if "assetId" not in door:
            return None

        asset_id = door["assetId"]
        asset_id = re.sub(r'(\d+)x(\d+)', r'\1_X_\2', asset_id)
        self.import_asset(body_builder, asset_id)

    def import_asset(self, body_builder: BodyBuilder, asset_name: str) -> None:
        asset_name = snake_to_camel(asset_name)

        asset_paths = get_asset_paths(asset_name)
        if len(asset_paths) == 0:
            return None
        else:
            i = int(random.uniform(0, len(asset_paths)))
            asset_path = asset_paths[i]

        asset_dir = os.path.dirname(asset_path)
        mesh_idx = 0
        body_name = body_builder.xform.GetPrim().GetName()
        if (os.path.basename(os.path.dirname(asset_dir)) == "grp_objects"
                or "Pen" in asset_name
                or "Keychain" in asset_name):
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
            if "Chair" in asset_name:
                print(asset_name)
            file_text = re.sub(r'[^a-zA-Z]', '', asset_name)
            for root, dirs, files in os.walk(asset_dir):
                for file in files:
                    file_name = os.path.splitext(file)[0]
                    if file.endswith('.stl') and file_text in file_name:
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
        # exporter = UrdfExporter(file_path=house_usd_file_path.replace(".usda", ".urdf"),
        #                         factory=factory)
        # exporter.build()
        # exporter.export(keep_usd=False)


if __name__ == '__main__':
    unittest.main()
