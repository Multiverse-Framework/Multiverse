#!/usr/bin/env python3.10

import importlib.util
import os, shutil
import random, string
from pxr import Usd, UsdGeom, Sdf, Gf, Tf
from enum import Enum

multiverse_parser_path = os.path.dirname(importlib.util.find_spec("multiverse_parser").origin)

mesh_dict = {}
body_dict = {}
geom_dict = {}

TMP = "tmp"
TMP_DIR = "tmp/usd"

xform_cache = UsdGeom.XformCache()


def copy_and_overwrite(source_folder: str, destination_folder: str) -> None:
    os.makedirs(name=destination_folder, exist_ok=True)

    # Iterate through all files and folders in the source folder
    for item in os.listdir(source_folder):
        source_item = os.path.join(source_folder, item)
        destination_item = os.path.join(destination_folder, item)

        # If item is a folder, call the function recursively
        if os.path.isdir(source_item):
            if os.path.exists(destination_item):
                shutil.rmtree(destination_item)
            shutil.copytree(source_item, destination_item)
        # If item is a file, simply copy it
        else:
            shutil.copy2(source_item, destination_item)


class GeomType(Enum):
    PLANE = 0
    CUBE = 1
    SPHERE = 2
    CYLINDER = 3
    MESH = 4


class Mesh:
    def __init__(self, name: str, usd_file_path: str) -> None:
        mesh_dict[name] = self
        self.stage = Usd.Stage.CreateNew(usd_file_path)
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, UsdGeom.LinearUnits.meters)
        self.prim = UsdGeom.Mesh.Define(self.stage, "/Mesh_" + name)
        self.stage.SetDefaultPrim(self.prim.GetPrim())

    def build(self, points, normals, face_vertex_counts, face_vertex_indices):
        self.prim.CreatePointsAttr(points)
        self.prim.CreateNormalsAttr(normals)
        self.prim.CreateFaceVertexCountsAttr(face_vertex_counts)
        self.prim.CreateFaceVertexIndicesAttr(face_vertex_indices)

    def save(self):
        self.stage.Save()


class Geom:
    def __init__(self, stage: Usd.Stage, name: str, body_path: Sdf.Path, type: GeomType) -> None:
        geom_dict[name] = self
        self.stage = stage
        self.usd_file_dir = os.path.dirname(self.stage.GetRootLayer().realPath)
        self.path = body_path.AppendPath(name)
        self.set_prim(type)

    def set_prim(self, type: GeomType) -> None:
        if type == GeomType.PLANE:
            self.prim = UsdGeom.Mesh.Define(self.stage, self.path)
            self.prim.CreatePointsAttr([(-0.5, -0.5, 0), (0.5, -0.5, 0), (-0.5, 0.5, 0), (0.5, 0.5, 0)])
            self.prim.CreateExtentAttr([(-0.5, -0.5, 0), (0.5, 0.5, 0)])
            self.prim.CreateNormalsAttr([(0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1)])
            self.prim.CreateFaceVertexCountsAttr([4])
            self.prim.CreateFaceVertexIndicesAttr([0, 1, 3, 2])
        elif type == GeomType.CUBE:
            self.prim = UsdGeom.Cube.Define(self.stage, self.path)
        elif type == GeomType.SPHERE:
            self.prim = UsdGeom.Sphere.Define(self.stage, self.path)
        elif type == GeomType.CYLINDER:
            self.prim = UsdGeom.Cylinder.Define(self.stage, self.path)
        elif type == GeomType.MESH:
            self.prim = UsdGeom.Mesh.Define(self.stage, self.path)

    def set_transform(
        self,
        pos: tuple = (0.0, 0.0, 0.0),
        quat: tuple = (1.0, 0.0, 0.0, 0.0),
        size: tuple = (1.0, 1.0, 1.0),
    ):
        mat = Gf.Matrix4d()
        mat.SetTranslateOnly(Gf.Vec3d(pos))
        mat.SetRotateOnly(Gf.Quatd(quat[0], Gf.Vec3d(quat[1], quat[2], quat[3])))
        mat_scale = Gf.Matrix4d()
        mat_scale.SetScale(Gf.Vec3d(size))
        mat = mat_scale * mat
        self.prim.AddTransformOp().Set(mat)

    def set_attribute(self, prefix: str = None, **kwargs):
        for key, value in kwargs.items():
            attr = prefix + ":" + key if prefix is not None else key
            if self.prim.GetPrim().HasAttribute(attr):
                self.prim.GetPrim().GetAttribute(attr).Set(value)

    def add_mesh(self, mesh_name: str) -> Mesh:
        mesh_dir = os.path.join(TMP_DIR, mesh_name + ".usda")
        mesh_ref = "./" + mesh_dir
        if mesh_name in mesh_dict:
            mesh = mesh_dict[mesh_name]
        else:
            mesh = Mesh(mesh_name, os.path.join(self.usd_file_dir, TMP_DIR, mesh_name + ".usda"))
        self.prim.GetPrim().GetReferences().AddReference(mesh_ref)
        return mesh


class Body:
    def __init__(self, stage: Usd.Stage, name: str, parent_name: str = None) -> None:
        body_dict[name] = self
        if parent_name is not None:
            parent_prim = body_dict.get(parent_name).prim
            if parent_prim is not None:
                self.path = parent_prim.GetPath().AppendPath(name)
            else:
                print(f"Parent prim with name {parent_name} not found.")
                return
        else:
            self.path = Sdf.Path("/").AppendPath(name)
        self.stage = stage
        self.usd_file_dir = os.path.dirname(self.stage.GetRootLayer().realPath)
        self.prim = UsdGeom.Xform.Define(self.stage, self.path)

    def set_transform(
        self,
        pos: tuple = (0.0, 0.0, 0.0),
        quat: tuple = (1.0, 0.0, 0.0, 0.0),
        size: tuple = (1.0, 1.0, 1.0),
        relative_to: str = None,
    ):
        mat = Gf.Matrix4d()
        mat.SetTranslateOnly(Gf.Vec3d(pos))
        mat.SetRotateOnly(Gf.Quatd(quat[0], Gf.Vec3d(quat[1], quat[2], quat[3])))
        mat_scale = Gf.Matrix4d()
        mat_scale.SetScale(Gf.Vec3d(size))
        mat = mat_scale * mat
        if relative_to is not None:
            relative_prim = body_dict[relative_to].prim.GetPrim()
            if relative_prim:
                parent_prim = self.prim.GetPrim().GetParent()
                if parent_prim and parent_prim != relative_prim:
                    parent_to_relative_mat, _ = xform_cache.ComputeRelativeTransform(relative_prim, parent_prim)
                    mat = mat * parent_to_relative_mat
            else:
                print(f"Prim at path {relative_to} not found.")

        self.prim.AddTransformOp().Set(mat)

    def add_geom(self, geom_name: str, geom_type: GeomType) -> Geom:
        if geom_name in geom_dict:
            print(f"Geom {geom_name} already exists.")
            return geom_dict[geom_name]

        return Geom(self.stage, geom_name, self.path, geom_type)


class UsdWorld:
    def __init__(self) -> None:
        random_string = "".join(random.choices(string.ascii_letters + string.digits, k=10))
        self.usd_file_path = os.path.join(multiverse_parser_path, ".cache", random_string, TMP + ".usda")
        print(f"Create {self.usd_file_path}")
        os.makedirs(os.path.dirname(self.usd_file_path))
        self.stage = Usd.Stage.CreateNew(self.usd_file_path)
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, UsdGeom.LinearUnits.meters)

    def add_body(self, body_name: str, parent_body_name: str = None) -> Body:
        if body_name in body_dict:
            print(f"Body {body_name} already exists.")
            return body_dict[body_name]

        if parent_body_name is None:
            self.root_body = Body(self.stage, body_name)
            self.stage.SetDefaultPrim(self.root_body.prim.GetPrim())
            return self.root_body
        else:
            return Body(self.stage, body_name, parent_body_name)

    def export(self, usd_file_path: str = None) -> None:
        self.stage.Save()

        if usd_file_path is not None:
            usd_file_dir = os.path.dirname(usd_file_path)
            usd_file_name = os.path.splitext(os.path.basename(usd_file_path))[0]

            copy_and_overwrite(os.path.dirname(self.usd_file_path), usd_file_dir)

            tmp_usd_file_path = os.path.join(usd_file_dir, os.path.basename(self.usd_file_path))
            os.rename(tmp_usd_file_path, usd_file_path)

            tmp_mesh_dir = os.path.join(usd_file_dir, TMP)
            new_mesh_dir = os.path.join(usd_file_dir, usd_file_name)
            if os.path.exists(new_mesh_dir):
                shutil.rmtree(new_mesh_dir)
            os.rename(tmp_mesh_dir, new_mesh_dir)

            with open(usd_file_path, 'r', encoding='utf-8') as file:
                file_contents = file.read()
            
            tmp_path = "prepend references = @./" + TMP + "/usd/"
            new_path = "prepend references = @./" + usd_file_name + "/usd/"
            file_contents = file_contents.replace(tmp_path, new_path)
            
            with open(usd_file_path, 'w', encoding='utf-8') as file:
                file.write(file_contents)

    def clean_up(self) -> None:
        print(f"Remove {os.path.dirname(self.usd_file_path)}")
        shutil.rmtree(os.path.dirname(self.usd_file_path))
        body_dict.clear()
        geom_dict.clear()
        mesh_dict.clear()
        xform_cache.Clear()
