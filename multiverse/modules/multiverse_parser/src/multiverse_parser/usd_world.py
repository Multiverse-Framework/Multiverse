#!/usr/bin/env python3.10

import importlib.util
import os, shutil
import random, string
from pxr import Usd, UsdGeom, Sdf, Gf, Tf
from enum import Enum

multiverse_parser_path = os.path.dirname(
    importlib.util.find_spec("multiverse_parser").origin
)

mesh_dict = {}
body_dict = {}
geom_dict = {}


def copy_and_overwrite(source_folder: str, destination_folder: str) -> None:
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
    def __init__(self, stage: Usd.Stage, name: str) -> None:
        mesh_dict[name] = self


class Geom:
    def __init__(
        self, stage: Usd.Stage, name: str, body_path: Sdf.Path, type: GeomType
    ) -> None:
        geom_dict[name] = self
        self.stage = stage
        self.path = body_path.AppendPath(name)
        self.set_prim(type)

    def set_prim(self, type: GeomType) -> None:
        if type == GeomType.PLANE:
            self.prim = UsdGeom.Mesh.Define(self.stage, self.path)
            self.prim.CreatePointsAttr(
                [(-0.5, -0.5, 0), (0.5, -0.5, 0), (-0.5, 0.5, 0), (0.5, 0.5, 0)]
            )
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
        transform = self.prim.AddTransformOp()
        mat = Gf.Matrix4d()
        mat.SetTranslateOnly(Gf.Vec3d(pos))
        mat.SetRotateOnly(Gf.Quatd(quat[0], Gf.Vec3d(quat[1], quat[2], quat[3])))
        mat_scale = Gf.Matrix4d()
        mat_scale.SetScale(Gf.Vec3d(size))
        mat = mat_scale * mat
        transform.Set(mat)

    def set_attribute(self, **kwargs):
        if (
            kwargs.get("size") is not None
            and isinstance(kwargs["size"], tuple)
            and UsdGeom.Cube(self.prim)
        ):
            self.prim.AddScaleOp().Set(Gf.Vec3d(kwargs["size"][0], kwargs["size"][1], kwargs["size"][2]))
            xformOpOrder = self.prim.GetXformOpOrderAttr().Get()
            new_xformOpOrder = ["xformOp:scale"]
            for xformOp in xformOpOrder:
                new_xformOpOrder.append(xformOp)
            self.prim.CreateXformOpOrderAttr().Set(new_xformOpOrder)


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
        self.usd_file_dir = os.path.dirname(stage.GetRootLayer().realPath)
        self.prim = UsdGeom.Xform.Define(self.stage, self.path)

    def set_pose(
        self,
        pos: tuple = (0.0, 0.0, 0.0),
        quat: tuple = (1.0, 0.0, 0.0, 0.0),
        relative_to: str = None,
    ):
        transform = self.prim.AddTransformOp()
        mat = Gf.Matrix4d()
        mat.SetTranslateOnly(Gf.Vec3d(pos))
        mat.SetRotateOnly(Gf.Quatd(quat[0], Gf.Vec3d(quat[1], quat[2], quat[3])))
        if relative_to is not None:
            prim = body_dict[relative_to].prim
            if prim:
                xformable = UsdGeom.Xformable(prim)
                mat = (
                    xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default()) * mat
                )
            else:
                print(f"Prim at path {relative_to} not found.")
        transform.Set(mat)

    def add_geom(self, geom_name: str, geom_type: GeomType) -> Geom:
        if geom_name in geom_dict:
            print(f"Geom {geom_name} already exists.")
            return geom_dict[geom_name]

        return Geom(self.stage, geom_name, self.path, geom_type)


class UsdWorld:
    def __init__(self) -> None:
        random_string = "".join(
            random.choices(string.ascii_letters + string.digits, k=10)
        )
        self.usd_file_path = os.path.join(
            multiverse_parser_path, ".cache", random_string, "tmp.usda"
        )
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
        
    def add_mesh(self, mesh_name: str) -> Mesh:
        if mesh_name in mesh_dict:
            return mesh_dict[mesh_name]

        stage = Usd.Stage.CreateNew(os.path.join(os.path.dirname(self.usd_file_path), 'tmp', 'usd', mesh_name + '.usda'))
        return Mesh(stage, mesh_name)

    def export(self, usd_file_path: str) -> None:
        self.stage.Save()
        copy_and_overwrite(
            os.path.dirname(self.usd_file_path), os.path.dirname(usd_file_path)
        )
        os.rename(
            os.path.join(
                os.path.dirname(usd_file_path), os.path.basename(self.usd_file_path)
            ),
            usd_file_path,
        )

    def clean_up(self) -> None:
        print(f"Remove {os.path.dirname(self.usd_file_path)}")
        shutil.rmtree(os.path.dirname(self.usd_file_path))
        body_dict.clear()
        geom_dict.clear()
        mesh_dict.clear()
