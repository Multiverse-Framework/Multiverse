#!/usr/bin/env python3.10

from dataclasses import dataclass
from typing import Optional, List, Dict, Tuple, Sequence

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


@dataclass(init=False)
class GeomProperty:
    name: str
    type: GeomType
    is_visible: bool
    is_collidable: bool
    rgba: Optional[numpy.ndarray]

    def __init__(self, geom_name: str, geom_type: GeomType, is_visible: bool = True, is_collidable: bool = True,
                 rgba: Optional[numpy.ndarray] = None) -> None:
        self.name = geom_name
        self.type = geom_type
        self.is_visible = is_visible
        self.is_collidable = is_collidable
        self.rgba = rgba

    @property
    def name(self) -> str:
        return self._name

    @name.setter
    def name(self, name: str) -> None:
        self._name = modify_name(name)

    @property
    def type(self) -> GeomType:
        return self._type

    @type.setter
    def type(self, geom_type: GeomType) -> None:
        self._type = geom_type

    @property
    def is_visible(self) -> bool:
        return self._is_visible

    @is_visible.setter
    def is_visible(self, is_visible: bool) -> None:
        self._is_visible = is_visible

    @property
    def is_collidable(self) -> bool:
        return self._is_collidable

    @is_collidable.setter
    def is_collidable(self, is_collidable: bool) -> None:
        self._is_collidable = is_collidable

    @property
    def rgba(self) -> Optional[numpy.ndarray]:
        return self._rgba

    @rgba.setter
    def rgba(self, rgba: Sequence) -> None:
        if rgba is not None:
            if len(rgba) != 4:
                raise ValueError(f"RGBA values must be a 4-element array.")
            rgba = numpy.array(rgba, dtype=numpy.float32)
            if not all(0.0 <= v <= 1.0 for v in rgba):
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
    stage: Usd.Stage
    xform: UsdGeom.Xform
    geom_prims: List[UsdGeom.Gprim]
    mesh_builders: Dict[str, MeshBuilder]
    material_builders: Dict[str, MaterialBuilder]

    def __init__(self, stage: Usd.Stage, geom_name: str, body_path: Sdf.Path, geom_property: GeomProperty) -> None:
        self._geom_property = geom_property
        self._xform = UsdGeom.Xform.Define(stage, body_path.AppendPath(geom_name))
        self._mesh_builders = {}
        self._material_builders = {}
        self._geom_prims = self._create_geoms()

    def add_mesh(self, mesh_file_path: str) -> Tuple[MeshBuilder, MaterialBuilder]:
        if mesh_file_path in self.mesh_builders:
            mesh_builder = self.mesh_builders[mesh_file_path]
        else:
            mesh_builder = MeshBuilder(mesh_file_path)

        if mesh_file_path in self.material_builders:
            material_builder = self.material_builders[mesh_file_path]
        else:
            material_builder = MaterialBuilder(file_path=mesh_file_path)

        reference_meshes = mesh_builder.meshes
        local_meshes = []
        for mesh in reference_meshes:
            local_meshes.append(self.stage.OverridePrim(self.xform.GetPath().AppendPath(mesh.GetPrim().GetName())))
            self.xform.GetPrim().GetReferences().AddReference(mesh_file_path, mesh_builder.xform.GetPath())

        bind_materials(mesh_builder, local_meshes, reference_meshes)

        reference_materials(material_builder, self.stage, mesh_file_path)

        return mesh_builder, material_builder

    def _create_geoms(self) -> List[Usd.Prim]:
        if self.type == GeomType.PLANE:
            geom = UsdGeom.Mesh.Define(self.stage, self.xform.GetPath().AppendPath("Plane"))
            geom.CreatePointsAttr([(-0.5, -0.5, 0), (0.5, -0.5, 0), (-0.5, 0.5, 0), (0.5, 0.5, 0)])
            geom.CreateNormalsAttr([(0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1)])
            geom.CreateFaceVertexCountsAttr([4])
            geom.CreateFaceVertexIndicesAttr([0, 1, 3, 2])
            return [UsdGeom.Mesh(geom)]
        if self.type == GeomType.CUBE:
            return [UsdGeom.Cube.Define(self.stage, self.xform.GetPath().AppendPath("Cube"))]
        if self.type == GeomType.SPHERE:
            return [UsdGeom.Sphere.Define(self.stage, self.xform.GetPath().AppendPath("Sphere"))]
        if self.type == GeomType.CYLINDER:
            return [UsdGeom.Cylinder.Define(self.stage, self.xform.GetPath().AppendPath("Cylinder"))]
        if self.type == GeomType.CAPSULE:
            return [UsdGeom.Cylinder.Define(self.stage, self.xform.GetPath().AppendPath("Capsule"))]
        if self.type == GeomType.MESH:
            return []
        raise ValueError(f"Geom type {self.type} not supported.")

    def build(self) -> List[UsdGeom.Gprim]:
        for geom in self.geom_prims:
            if self.rgba is not None:
                geom.CreateDisplayColorAttr(self.rgba[:3])
                geom.CreateDisplayOpacityAttr(self.rgba[3])
        return self.geom_prims

    def set_transform(
            self,
            pos: numpy.ndarray = numpy.array([0.0, 0.0, 0.0]),
            quat: numpy.ndarray = numpy.array([1.0, 0.0, 0.0, 0.0]),
            scale: numpy.ndarray = numpy.array([1.0, 1.0, 1.0]),
    ) -> None:
        """
        Set the transform of the body.
        :param pos: Array of x, y, z position.
        :param quat: Array of w, x, y, z quaternion.
        :param scale: Array of x, y, z scale.
        :return: None
        """
        mat = Gf.Matrix4d()
        mat.SetTranslateOnly(Gf.Vec3d(*pos))
        mat.SetRotateOnly(Gf.Quatd(quat[0], Gf.Vec3d(quat[1], quat[2], quat[3])))
        mat_scale = Gf.Matrix4d()
        mat_scale.SetScale(Gf.Vec3d(*scale))
        mat = mat_scale * mat
        self.xform.ClearXformOpOrder()
        self.xform.AddTransformOp().Set(mat)
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
                # TODO: Add more attributes
                capsule = UsdGeom.Cylinder(geom_prim)
                radius = capsule.GetRadiusAttr().Get()
                height = capsule.GetHeightAttr().Get()
                capsule.CreateExtentAttr(((-radius, -radius, -height / 2), (radius, radius, height / 2)))

    @property
    def stage(self) -> Usd.Stage:
        return self.xform.GetPrim().GetStage()

    @property
    def geom_prims(self) -> List[UsdGeom.Gprim]:
        return [UsdGeom.Gprim(prim) for prim in self.xform.GetPrim().GetChildren()]

    @property
    def xform(self) -> UsdGeom.Xform:
        return self._xform

    @property
    def type(self) -> GeomType:
        return self._geom_property.type

    @property
    def rgba(self) -> Optional[numpy.ndarray]:
        return self._geom_property.rgba

    @property
    def mesh_builders(self) -> Dict[str, MeshBuilder]:
        return self._mesh_builders

    @property
    def material_builders(self) -> Dict[str, MaterialBuilder]:
        return self._material_builders
