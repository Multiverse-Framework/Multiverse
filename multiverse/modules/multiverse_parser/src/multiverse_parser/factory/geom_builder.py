#!/usr/bin/env python3.10

import os
from dataclasses import dataclass
from typing import Optional, List

from pxr import Usd, UsdGeom, Gf, UsdPhysics, Sdf, UsdShade
# import bpy, bmesh
from enum import Enum

from .mesh_builder import MeshBuilder
from ..utils import modify_name


class GeomType(Enum):
    PLANE = 0
    CUBE = 1
    SPHERE = 2
    CYLINDER = 3
    CAPSULE = 4
    MESH = 5


@dataclass
class GeomProperty:
    is_visible: bool = True
    is_collidable: bool = True
    rgba: Optional[tuple] = None


def inertia_of_triangle(v1, v2, v3, density) -> Gf.Matrix3d:
    # Compute the area of the triangle
    area = 0.5 * ((v2 - v1).cross(v3 - v1)).length
    # Compute the inertia tensor of the triangle
    inertia = Gf.Matrix3d()
    for i in range(3):
        for j in range(3):
            inertia[i][j] = (
                    density
                    * area
                    / 12
                    * (
                            (v1[i] ** 2 + v1[j] ** 2 + v2[i] ** 2 + v2[j] ** 2 + v3[i] ** 2 + v3[j] ** 2)
                            + (v1[i] * v2[i] + v1[j] * v2[j] + v2[i] * v3[i] + v2[j] * v3[j] + v1[i] * v3[i] + v1[j] *
                               v3[j])
                    )
            )
            if i == j:
                inertia[i][i] -= (
                        density
                        * area
                        * (
                                v1[(i + 1) % 3] ** 2
                                + v1[(i + 2) % 3] ** 2
                                + v2[(i + 1) % 3] ** 2
                                + v2[(i + 2) % 3] ** 2
                                + v3[(i + 1) % 3] ** 2
                                + v3[(i + 2) % 3] ** 2
                        )
                        / 12
                )
    return inertia


class GeomBuilder:
    stage: Usd.Stage
    name: str
    type: GeomType
    scale: tuple
    xform: UsdGeom.Xform
    geom_properties: GeomProperty

    def __init__(self, stage: Usd.Stage, geom_name: str, body_path: Sdf.Path, geom_type: GeomType,
                 geom_property: GeomProperty) -> None:
        self.stage = stage
        self.name = geom_name
        self.type = geom_type
        self.scale = (1.0, 1.0, 1.0)
        self.xform = UsdGeom.Xform.Define(self.stage, body_path.AppendPath(geom_name))
        self.geom_properties = geom_property

    def add_mesh(self, mesh_name: str, mesh_file_path: str) -> MeshBuilder:
        mesh_name = modify_name(in_name=mesh_name)
        if mesh_name is None or not mesh_name[0].isalpha():
            mesh_name = "SM_" + self.name

        mesh_ref = "./" + mesh_file_path

        mesh_builder = MeshBuilder(mesh_name, mesh_file_path)
        for mesh in mesh_builder.build():
            geom = self.stage.OverridePrim(self.xform.GetPath().AppendPath(mesh.GetPrim().GetName()))
            self.xform.GetPrim().GetReferences().AddReference(mesh_ref, mesh.GetPath())

        return mesh_builder

    def _create_geom(self) -> UsdGeom:
        if self.type == GeomType.PLANE:
            geom = UsdGeom.Mesh.Define(self.stage, self.xform.GetPath().AppendPath("Plane"))
            geom.CreatePointsAttr([(-0.5, -0.5, 0), (0.5, -0.5, 0), (-0.5, 0.5, 0), (0.5, 0.5, 0)])
            geom.CreateNormalsAttr([(0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1)])
            geom.CreateFaceVertexCountsAttr([4])
            geom.CreateFaceVertexIndicesAttr([0, 1, 3, 2])
        elif self.type == GeomType.CUBE:
            geom = UsdGeom.Cube.Define(self.stage, self.xform.GetPath().AppendPath("Cube"))
        elif self.type == GeomType.SPHERE:
            geom = UsdGeom.Sphere.Define(self.stage, self.xform.GetPath().AppendPath("Sphere"))
        elif self.type == GeomType.CYLINDER:
            geom = UsdGeom.Cylinder.Define(self.stage, self.xform.GetPath().AppendPath("Cylinder"))
        elif self.type == GeomType.CAPSULE:
            geom = UsdGeom.Cylinder.Define(self.stage, self.xform.GetPath().AppendPath("Capsule"))
        elif self.type == GeomType.MESH:
            geom = None
        else:
            raise ValueError(f"Geom type {self.type} not supported.")
        return geom

    def build(self) -> UsdGeom:
        geom = self._create_geom()
        if self.geom_properties.rgba is not None:
            geom.CreateDisplayColorAttr(self.geom_properties.rgba[:3])
            geom.CreateDisplayOpacityAttr(self.geom_properties.rgba[3])
        return geom

    def set_transform(
            self,
            pos: tuple = (0.0, 0.0, 0.0),
            quat: tuple = (1.0, 0.0, 0.0, 0.0),
            scale: tuple = (1.0, 1.0, 1.0),
    ) -> None:
        self.scale = scale
        mat = Gf.Matrix4d()
        mat.SetTranslateOnly(pos)
        mat.SetRotateOnly(Gf.Quatd(quat[0], Gf.Vec3d(quat[1], quat[2], quat[3])))
        mat_scale = Gf.Matrix4d()
        mat_scale.SetScale(scale)
        mat = mat_scale * mat
        self.xform.AddTransformOp().Set(mat)
        self._update_extent()

    def set_attribute(self, prefix: str = None, **kwargs) -> None:
        for geom_prim in self.geom_prims:
            for key, value in kwargs.items():
                attr = prefix + ":" + key if prefix is not None else key
                if not geom_prim.GetPrim().HasAttribute(attr):
                    raise ValueError(f"Geom {self.name} does not have attribute {attr}.")
                geom_prim.GetPrim().GetAttribute(attr).Set(value)
        self._update_extent()

    def _update_extent(self) -> None:
        for geom_prim in self.geom_prims:
            if self.type == GeomType.PLANE:
                mesh = UsdGeom.Mesh(geom_prim)
                mesh.CreateExtentAttr([(-0.5, -0.5, 0), (0.5, 0.5, 0)])
            elif self.type == GeomType.CUBE:
                cube = UsdGeom.Cube(geom_prim)
                cube.CreateExtentAttr(((-1, -1, -1), (1, 1, 1)))
            elif self.type == GeomType.SPHERE:
                sphere = UsdGeom.Sphere(geom_prim)
                radius = sphere.GetRadiusAttr().Get()
                sphere.CreateExtentAttr(((-radius, -radius, -radius), (radius, radius, radius)))
            elif self.type == GeomType.CYLINDER:
                cylinder = UsdGeom.Cylinder(geom_prim)
                radius = cylinder.GetRadiusAttr().Get()
                height = cylinder.GetHeightAttr().Get()
                cylinder.CreateExtentAttr(((-radius, -radius, -height / 2), (radius, radius, height / 2)))
            elif self.type == GeomType.CAPSULE:
                capsule = UsdGeom.Cylinder(geom_prim)
                radius = capsule.GetRadiusAttr().Get()
                height = capsule.GetHeightAttr().Get()
                capsule.CreateExtentAttr(((-radius, -radius, -height / 2), (radius, radius, height / 2)))

    @property
    def geom_prims(self) -> List[Usd.Prim]:
        return self.xform.GetPrim().GetChildren()
