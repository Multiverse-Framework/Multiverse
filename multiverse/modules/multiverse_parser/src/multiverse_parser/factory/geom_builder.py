#!/usr/bin/env python3.10

import os
from pxr import Usd, UsdGeom, Gf, UsdPhysics, Sdf
from enum import Enum

from multiverse_parser.factory import TMP_DIR
from .mesh_builder import MeshBuilder, VisualMeshBuilder, CollisionMeshBuilder, mesh_dict

geom_dict = {}


class GeomType(Enum):
    PLANE = 0
    CUBE = 1
    SPHERE = 2
    CYLINDER = 3
    MESH = 4


class GeomBuilder:
    def __init__(self, stage: Usd.Stage, name: str, body_path: Sdf.Path, geom_type: GeomType) -> None:
        geom_dict[name] = self
        self.stage = stage
        self.path = body_path.AppendPath(name)
        self.type = geom_type
        self.set_prim()
        self.pos = Gf.Vec3d(0.0, 0.0, 0.0)
        self.quat = Gf.Quatd(1.0, 0.0, 0.0, 0.0)
        self.scale = Gf.Vec3d(1.0, 1.0, 1.0)

    def set_prim(self) -> None:
        if self.type == GeomType.PLANE:
            self.prim = UsdGeom.Mesh.Define(self.stage, self.path)
            self.prim.CreatePointsAttr([(-0.5, -0.5, 0), (0.5, -0.5, 0), (-0.5, 0.5, 0), (0.5, 0.5, 0)])
            self.prim.CreateNormalsAttr([(0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1)])
            self.prim.CreateFaceVertexCountsAttr([4])
            self.prim.CreateFaceVertexIndicesAttr([0, 1, 3, 2])
        elif self.type == GeomType.CUBE:
            self.prim = UsdGeom.Cube.Define(self.stage, self.path)
        elif self.type == GeomType.SPHERE:
            self.prim = UsdGeom.Sphere.Define(self.stage, self.path)
        elif self.type == GeomType.CYLINDER:
            self.prim = UsdGeom.Cylinder.Define(self.stage, self.path)
        elif self.type == GeomType.MESH:
            self.prim = UsdGeom.Mesh.Define(self.stage, self.path)

    def set_transform(
        self,
        pos: tuple = (0.0, 0.0, 0.0),
        quat: tuple = (1.0, 0.0, 0.0, 0.0),
        scale: tuple = (1.0, 1.0, 1.0),
    ):
        self.pos = Gf.Vec3d(pos)
        self.quat = Gf.Quatd(quat[0], Gf.Vec3d(quat[1], quat[2], quat[3]))
        self.scale = Gf.Vec3d(scale)

        mat = Gf.Matrix4d()
        mat.SetTranslateOnly(self.pos)
        mat.SetRotateOnly(self.quat)
        mat_scale = Gf.Matrix4d()
        mat_scale.SetScale(self.scale)
        mat = mat_scale * mat
        self.prim.AddTransformOp().Set(mat)

    def set_attribute(self, prefix: str = None, **kwargs) -> None:
        for key, value in kwargs.items():
            attr = prefix + ":" + key if prefix is not None else key
            if self.prim.GetPrim().HasAttribute(attr):
                self.prim.GetPrim().GetAttribute(attr).Set(value)

    def compute_extent(self) -> None:
        if self.type == GeomType.PLANE:
            self.prim.CreateExtentAttr([(-0.5, -0.5, 0), (0.5, 0.5, 0)])
        elif self.type == GeomType.CUBE:
            self.prim.CreateExtentAttr(((-1, -1, -1), (1, 1, 1)))
        elif self.type == GeomType.SPHERE:
            radius = self.prim.GetRadiusAttr().Get()
            self.prim.CreateExtentAttr(((-radius, -radius, -radius), (radius, radius, radius)))
        elif self.type == GeomType.CYLINDER:
            radius = self.prim.GetRadiusAttr().Get()
            height = self.prim.GetHeightAttr().Get()
            self.prim.CreateExtentAttr(((-radius, -radius, -height / 2), (radius, radius, height / 2)))
        elif self.type == GeomType.MESH:
            self.prim.CreateExtentAttr(((-1, -1, -1), (1, 1, 1)))

    def add_mesh(self, mesh_name: str, collision: bool = True) -> MeshBuilder:
        mesh_path = os.path.join(TMP_DIR, "collision" if collision else "visual", mesh_name + ".usda")
        mesh_ref = "./" + mesh_path
        if mesh_name in mesh_dict:
            mesh = mesh_dict[mesh_name]
        else:
            from multiverse_parser.factory import TMP_USD_FILE_DIR

            usd_file_path = os.path.join(TMP_USD_FILE_DIR, mesh_path)
            if not collision:
                mesh = VisualMeshBuilder(mesh_name, usd_file_path)
            else:
                mesh = CollisionMeshBuilder(mesh_name, usd_file_path)

        self.prim.GetPrim().GetReferences().AddReference(mesh_ref)
        return mesh

    def enable_collision(self) -> None:
        physics_collision_api = UsdPhysics.CollisionAPI(self.prim)
        physics_collision_api.CreateCollisionEnabledAttr(True)
        physics_collision_api.Apply(self.prim.GetPrim())

        if self.type == GeomType.MESH:
            physics_mesh_collision_api = UsdPhysics.MeshCollisionAPI(self.prim)
            physics_mesh_collision_api.CreateApproximationAttr("convexHull")
            physics_mesh_collision_api.Apply(self.prim.GetPrim())
