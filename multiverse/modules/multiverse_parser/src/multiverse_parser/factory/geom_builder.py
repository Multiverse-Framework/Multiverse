#!/usr/bin/env python3.10

import os
from dataclasses import dataclass
from typing import Optional, List, Union, Any

import numpy
from pxr import Usd, UsdGeom, Gf, UsdPhysics, Sdf, UsdShade
# import bpy, bmesh
from enum import Enum

from .material_builder import MaterialBuilder
from .mesh_builder import MeshBuilder
from ..utils import modify_name


class GeomType(Enum):
    PLANE = 0
    CUBE = 1
    SPHERE = 2
    CYLINDER = 3
    CAPSULE = 4
    MESH = 5


class GeomProperty:
    is_visible: bool
    is_collidable: bool
    _rgba: Optional[tuple]

    def __init__(self, is_visible: bool = True, is_collidable: bool = True, rgba: Optional[tuple] = None) -> None:
        self.is_visible = is_visible
        self.is_collidable = is_collidable
        self.rgba = rgba

    @property
    def rgba(self) -> Optional[tuple]:
        return self._rgba

    @rgba.setter
    def rgba(self, rgba: Any) -> None:
        if rgba is not None:
            rgba = (float(rgba[0]), float(rgba[1]), float(rgba[2]), float(rgba[3]))
            if len(rgba) != 4:
                raise ValueError(f"RGBA must be a tuple of length 4.")
            for value in rgba:
                if not (0.0 <= value <= 1.0):
                    raise ValueError(f"RGBA values must be between 0.0 and 1.0.")
        self._rgba = rgba


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


def bind_materials(mesh_builder: MeshBuilder, local_meshes: List[UsdGeom.Mesh], reference_meshes: List[UsdGeom.Mesh]):
    paths = mesh_builder.xform.GetPrim().FindAllRelationshipTargetPaths()
    if len(paths) > 0:
        for local_mesh, reference_mesh in zip(local_meshes, reference_meshes):
            if reference_mesh.GetPrim().HasAPI(UsdShade.MaterialBindingAPI):
                material_binding_api = UsdShade.MaterialBindingAPI.Apply(local_mesh.GetPrim())
                for path in paths:
                    material = UsdShade.Material(mesh_builder.stage.GetPrimAtPath(path))
                    if material:
                        material_binding_api.Bind(material)


def reference_materials(material_builder: MaterialBuilder, stage: Usd.Stage, mesh_file_path: str):
    material_root_path = material_builder.root_prim.GetPath()
    material_root_prim = stage.GetPrimAtPath(material_root_path)
    if not material_root_prim.IsValid():
        material_root_prim = stage.DefinePrim(material_root_path)
    material_root_prim.GetReferences().AddReference(mesh_file_path, material_root_path)


class GeomBuilder:
    _stage: Usd.Stage
    _type: GeomType
    _xform: UsdGeom.Xform
    _property: GeomProperty

    def __init__(self, stage: Usd.Stage, geom_name: str, body_path: Sdf.Path, geom_type: GeomType,
                 geom_property: GeomProperty) -> None:
        self._stage = stage
        self._type = geom_type
        self._xform = UsdGeom.Xform.Define(self._stage, body_path.AppendPath(geom_name))
        self._property = geom_property

    def add_mesh(self, mesh_file_path: str) -> MeshBuilder:
        mesh_builder = MeshBuilder(mesh_file_path)
        reference_meshes = mesh_builder.build()
        local_meshes = []
        for mesh in reference_meshes:
            local_meshes.append(self._stage.OverridePrim(self.xform.GetPath().AppendPath(mesh.GetPrim().GetName())))
            self.xform.GetPrim().GetReferences().AddReference(mesh_file_path, mesh_builder.xform.GetPath())

        material_builder = MaterialBuilder(file_path=mesh_file_path)

        bind_materials(mesh_builder, local_meshes, reference_meshes)

        reference_materials(material_builder, self._stage, mesh_file_path)

        return mesh_builder

    def _create_geoms(self) -> List[UsdGeom.Gprim]:
        if self.type == GeomType.PLANE:
            geom = UsdGeom.Mesh.Define(self._stage, self.xform.GetPath().AppendPath("Plane"))
            geom.CreatePointsAttr([(-0.5, -0.5, 0), (0.5, -0.5, 0), (-0.5, 0.5, 0), (0.5, 0.5, 0)])
            geom.CreateNormalsAttr([(0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1)])
            geom.CreateFaceVertexCountsAttr([4])
            geom.CreateFaceVertexIndicesAttr([0, 1, 3, 2])
            return [UsdGeom.Mesh(geom)]
        if self.type == GeomType.CUBE:
            geom = UsdGeom.Cube.Define(self._stage, self.xform.GetPath().AppendPath("Cube"))
            return [UsdGeom.Cube(geom)]
        if self.type == GeomType.SPHERE:
            geom = UsdGeom.Sphere.Define(self._stage, self.xform.GetPath().AppendPath("Sphere"))
            return [UsdGeom.Sphere(geom)]
        if self.type == GeomType.CYLINDER:
            geom = UsdGeom.Cylinder.Define(self._stage, self.xform.GetPath().AppendPath("Cylinder"))
            return [UsdGeom.Cylinder(geom)]
        if self.type == GeomType.CAPSULE:
            geom = UsdGeom.Cylinder.Define(self._stage, self.xform.GetPath().AppendPath("Capsule"))
            return [UsdGeom.Cylinder(geom)]
        if self.type == GeomType.MESH:
            return self.geom_prims
        raise ValueError(f"Geom type {self.type} not supported.")

    def build(self) -> List[UsdGeom.Gprim]:
        geoms = self._create_geoms()
        for geom in geoms:
            if self.property.rgba is not None:
                geom.CreateDisplayColorAttr([self.property.rgba[:3]])
                geom.CreateDisplayOpacityAttr([self.property.rgba[3]])
        return geoms

    def set_transform(
            self,
            pos: tuple = (0.0, 0.0, 0.0),
            quat: tuple = (1.0, 0.0, 0.0, 0.0),
            scale: tuple = (1.0, 1.0, 1.0),
    ) -> None:
        mat = Gf.Matrix4d()
        mat.SetTranslateOnly(pos)
        mat.SetRotateOnly(Gf.Quatd(quat[0], Gf.Vec3d(quat[1], quat[2], quat[3])))
        mat_scale = Gf.Matrix4d()
        mat_scale.SetScale(scale)
        mat = mat_scale * mat
        self._xform.ClearXformOpOrder()
        self._xform.AddTransformOp().Set(mat)
        self._update_extent()

    def set_attribute(self, prefix: str = None, **kwargs) -> None:
        for geom_prim in self.geom_prims:
            for key, value in kwargs.items():
                attr = prefix + ":" + key if prefix is not None else key
                if not geom_prim.GetPrim().HasAttribute(attr):
                    raise ValueError(f"Geom {geom_prim.GetPrim().GetName()} does not have attribute {attr}.")
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
    def geom_prims(self) -> List[UsdGeom.Gprim]:
        return [UsdGeom.Gprim(prim) for prim in self.xform.GetPrim().GetChildren()]

    @property
    def xform(self) -> UsdGeom.Xform:
        return self._xform

    @property
    def type(self) -> GeomType:
        return self._type

    @property
    def property(self) -> GeomProperty:
        return self._property
