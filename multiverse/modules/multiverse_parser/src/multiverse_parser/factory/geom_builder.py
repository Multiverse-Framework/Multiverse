#!/usr/bin/env python3

import os
from dataclasses import dataclass
from typing import Optional
from enum import Enum

import numpy

from .mesh_builder import MeshBuilder, MeshProperty
from .material_builder import MaterialBuilder, MaterialProperty
from ..utils import modify_name, calculate_mesh_inertial, shift_inertia_tensor, shift_center_of_mass, get_transform

from pxr import Usd, UsdGeom, Sdf, UsdShade, UsdPhysics


class GeomType(Enum):
    PLANE = 0
    CUBE = 1
    SPHERE = 2
    CYLINDER = 3
    CAPSULE = 4
    MESH = 5


@dataclass
class GeomInertial:
    mass: float = 0.0
    inertia_tensor: numpy.ndarray = numpy.zeros((3, 3))
    center_of_mass: numpy.ndarray = numpy.zeros((1, 3))


@dataclass(init=False)
class GeomProperty:
    type: GeomType
    is_visible: bool
    is_collidable: bool
    rgba: Optional[numpy.ndarray]
    density: float

    def __init__(self,
                 geom_type: GeomType,
                 is_visible: bool = True,
                 is_collidable: bool = True,
                 rgba: Optional[numpy.ndarray] = None,
                 density: float = 1000.0) -> None:
        self._type = geom_type
        self._is_visible = is_visible
        self._is_collidable = is_collidable
        if rgba is not None:
            if len(rgba) != 4:
                raise ValueError(f"RGBA values must be a 4-element array.")
            rgba = numpy.array(rgba, dtype=numpy.float32)
            if not all(0.0 <= v <= 1.0 for v in rgba):
                raise ValueError(f"RGBA values must be between 0.0 and 1.0.")
        self._rgba = rgba
        self._density = density

    @property
    def type(self) -> GeomType:
        return self._type

    @property
    def is_visible(self) -> bool:
        return self._is_visible

    @property
    def is_collidable(self) -> bool:
        return self._is_collidable

    @property
    def rgba(self) -> Optional[numpy.ndarray]:
        return self._rgba

    @property
    def density(self) -> float:
        return self._density


class GeomBuilder:
    stage: Usd.Stage
    gprim: UsdGeom.Gprim

    def __init__(self,
                 stage: Usd.Stage,
                 geom_name: str,
                 body_path: Sdf.Path,
                 geom_property: GeomProperty,
                 geom_inertia: GeomInertial = None) -> None:
        self._property = geom_property
        geom_name = modify_name(geom_name)
        gprim_path = body_path.AppendPath(geom_name)
        self._gprim = self._create_gprim(stage=stage, gprim_path=gprim_path)
        self._inertial = geom_inertia

    def _create_gprim(self, stage: Usd.Stage, gprim_path: Sdf.Path) -> Usd.Prim:
        if self.type == GeomType.PLANE:
            geom = UsdGeom.Mesh.Define(stage, gprim_path)
            geom.CreatePointsAttr([(-0.5, -0.5, 0), (0.5, -0.5, 0), (-0.5, 0.5, 0), (0.5, 0.5, 0)])
            geom.CreateNormalsAttr([(0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1)])
            geom.CreateFaceVertexCountsAttr([4])
            geom.CreateFaceVertexIndicesAttr([0, 1, 3, 2])
            return UsdGeom.Mesh(geom)
        if self.type == GeomType.CUBE:
            return UsdGeom.Cube.Define(stage, gprim_path)
        if self.type == GeomType.SPHERE:
            return UsdGeom.Sphere.Define(stage, gprim_path)
        if self.type == GeomType.CYLINDER:
            return UsdGeom.Cylinder.Define(stage, gprim_path)
        if self.type == GeomType.CAPSULE:
            return UsdGeom.Cylinder.Define(stage, gprim_path)
        if self.type == GeomType.MESH:
            return UsdGeom.Mesh.Define(stage, gprim_path)
        raise ValueError(f"Geom type {self.type} not supported.")

    def add_mesh(self,
                 mesh_name: str,
                 mesh_property: MeshProperty) -> MeshBuilder:
        new_usd_mesh_file_path = os.path.join(self.stage.GetRootLayer().realPath.replace(".usda", ""),
                                              "meshes",
                                              "usd",
                                              f"{mesh_name}.usda")
        mesh_name = modify_name(mesh_name, "Mesh_")
        if not os.path.exists(new_usd_mesh_file_path):
            new_mesh_stage = Usd.Stage.CreateNew(new_usd_mesh_file_path)
            new_mesh = UsdGeom.Mesh.Define(new_mesh_stage, f"/{mesh_name}")
            new_mesh_stage.SetDefaultPrim(new_mesh.GetPrim())
            UsdGeom.SetStageUpAxis(new_mesh_stage, UsdGeom.Tokens.z)

            mesh_builder = MeshBuilder(stage=new_mesh_stage, mesh_property=mesh_property)
            local_mesh = mesh_builder.build()
        else:
            new_mesh_stage = Usd.Stage.Open(new_usd_mesh_file_path)
            new_default_prim = new_mesh_stage.GetDefaultPrim()
            if mesh_name != new_default_prim.GetName():
                raise ValueError(f"Mesh name {mesh_name} does not match default prim name in {new_usd_mesh_file_path}.")
            if not new_default_prim.IsA(UsdGeom.Mesh):
                raise ValueError(f"Default prim of {new_usd_mesh_file_path} is not a mesh.")

            mesh_builder = MeshBuilder(stage=new_mesh_stage, mesh_property=mesh_property)
            local_mesh = new_default_prim

        reference_prim = local_mesh.GetPrim()
        new_usd_mesh_file_relpath = os.path.relpath(new_usd_mesh_file_path,
                                                    os.path.dirname(self.stage.GetRootLayer().realPath))
        self.gprim.GetPrim().GetReferences().AddReference(f"./{new_usd_mesh_file_relpath}", reference_prim.GetPath())

        return mesh_builder

    def add_material(self,
                     material_name: str,
                     material_property: MaterialProperty,
                     subset: Optional[UsdGeom.Subset] = None) -> MaterialBuilder:
        new_usd_material_file_path = os.path.join(self.stage.GetRootLayer().realPath.replace(".usda", ""),
                                                  "materials",
                                                  "usd",
                                                  f"{material_name}.usda")
        material_name = modify_name(material_name)
        if not os.path.exists(new_usd_material_file_path):
            new_material_stage = Usd.Stage.CreateNew(new_usd_material_file_path)
            new_material = UsdShade.Material.Define(new_material_stage, f"/{material_name}")
            new_material_prim = new_material.GetPrim()
            new_material_stage.SetDefaultPrim(new_material_prim)
            UsdGeom.SetStageUpAxis(new_material_stage, UsdGeom.Tokens.z)
        else:
            new_material_stage = Usd.Stage.Open(new_usd_material_file_path)
            new_material_prim = new_material_stage.GetPrimAtPath(Sdf.Path(f"/{material_name}"))
            if not new_material_prim.IsA(UsdShade.Material):
                raise ValueError(f"Prim {material_name} of {new_usd_material_file_path} is not a material.")

        material_builder = MaterialBuilder(stage=new_material_stage,
                                           material_property=material_property)
        material = material_builder.build()

        local_materials_path = self.gprim.GetPrim().GetPath().AppendPath("Materials")
        if not self.stage.GetPrimAtPath(local_materials_path).IsValid():
            UsdGeom.Scope.Define(self.stage, local_materials_path)

        local_material_path = local_materials_path.AppendChild(material_name)
        if not self.stage.GetPrimAtPath(local_material_path).IsValid():
            UsdShade.Material.Define(self.stage, local_material_path)

        local_material_prim = self.stage.GetPrimAtPath(local_material_path)

        reference_material_prim = material.GetPrim()
        new_usd_material_file_relpath = os.path.relpath(new_usd_material_file_path,
                                                        os.path.dirname(self.stage.GetRootLayer().realPath))
        local_material_prim.GetReferences().AddReference(f"./{new_usd_material_file_relpath}",
                                                         reference_material_prim.GetPath())
        local_material = UsdShade.Material(local_material_prim)

        if subset is None:
            material_binding_api = UsdShade.MaterialBindingAPI.Apply(self.gprim.GetPrim())
        else:
            subset_name = subset.GetPrim().GetName()
            local_subset_path = self.gprim.GetPrim().GetPath().AppendPath(subset_name)
            local_subset = UsdGeom.Subset.Define(self.stage, local_subset_path)
            local_subset.CreateElementTypeAttr(UsdGeom.Tokens.face)
            local_subset.CreateIndicesAttr(subset.GetIndicesAttr().Get())
            local_subset.CreateFamilyNameAttr(subset.GetFamilyNameAttr().Get())
            material_binding_api = UsdShade.MaterialBindingAPI.Apply(local_subset.GetPrim())

        material_binding_api.Bind(local_material)

        return material_builder

    def build(self) -> UsdGeom.Gprim:
        if self.rgba is not None:
            self.gprim.CreateDisplayColorAttr(self.rgba[:3])
            self.gprim.CreateDisplayOpacityAttr(self.rgba[3])
        if not self.is_visible:
            self.gprim.CreateDisplayOpacityAttr([0.0])
            self.gprim.CreateVisibilityAttr("invisible")

        if self.is_collidable:
            physics_collision_api = UsdPhysics.CollisionAPI(self.gprim)
            physics_collision_api.CreateCollisionEnabledAttr(True)
            physics_collision_api.Apply(self.gprim.GetPrim())
            if self.type == GeomType.MESH:
                physics_mesh_collision_api = UsdPhysics.MeshCollisionAPI(self.gprim)
                physics_mesh_collision_api.CreateApproximationAttr("convexHull")
                physics_mesh_collision_api.Apply(self.gprim.GetPrim())

        return self.gprim

    def set_transform(
            self,
            pos: numpy.ndarray = numpy.array([0.0, 0.0, 0.0]),
            quat: numpy.ndarray = numpy.array([0.0, 0.0, 0.0, 1.0]),
            scale: numpy.ndarray = numpy.array([1.0, 1.0, 1.0]),
    ) -> None:
        """
        Set the transform of the body.
        :param pos: Array of x, y, z position.
        :param quat: Array of x, y, z, w quaternion.
        :param scale: Array of x, y, z scale.
        :return: None
        """
        mat = get_transform(pos=pos, quat=quat, scale=scale)
        self.gprim.ClearXformOpOrder()
        self.gprim.AddTransformOp().Set(mat)
        self._update_extent()

    def set_attribute(self, prefix: str = None, **kwargs) -> None:
        gprim = self.gprim
        for key, value in kwargs.items():
            attr = prefix + ":" + key if prefix is not None else key
            if not gprim.GetPrim().HasAttribute(attr):
                raise ValueError(f"Geom {gprim.GetPrim().GetName()} does not have attribute {attr}.")
            gprim.GetPrim().GetAttribute(attr).Set(value)
        self._update_extent()

    def _update_extent(self) -> None:
        gprim_prim = self.gprim.GetPrim()
        if self.type == GeomType.PLANE:
            mesh = UsdGeom.Mesh(gprim_prim)
            mesh.CreateExtentAttr([(-0.5, -0.5, 0), (0.5, 0.5, 0)])
        elif self.type == GeomType.CUBE:
            cube = UsdGeom.Cube(gprim_prim)
            cube.CreateExtentAttr(((-1, -1, -1), (1, 1, 1)))
        elif self.type == GeomType.SPHERE:
            sphere = UsdGeom.Sphere(gprim_prim)
            radius = sphere.GetRadiusAttr().Get()
            sphere.CreateExtentAttr(((-radius, -radius, -radius), (radius, radius, radius)))
        elif self.type == GeomType.CYLINDER:
            cylinder = UsdGeom.Cylinder(gprim_prim)
            radius = cylinder.GetRadiusAttr().Get()
            height = cylinder.GetHeightAttr().Get()
            cylinder.CreateExtentAttr(((-radius, -radius, -height / 2), (radius, radius, height / 2)))
        elif self.type == GeomType.CAPSULE:
            # TODO: Add more attributes
            capsule = UsdGeom.Cylinder(gprim_prim)
            radius = capsule.GetRadiusAttr().Get()
            height = capsule.GetHeightAttr().Get()
            capsule.CreateExtentAttr(((-radius, -radius, -height / 2), (radius, radius, height / 2)))

    def calculate_inertial(self) -> GeomInertial:
        self._inertial = self.origin_inertial

        gprim_transform = self.gprim.GetLocalTransformation()
        gprim_pos = gprim_transform.ExtractTranslation()
        gprim_pos = numpy.array([[*gprim_pos]])
        gprim_quat = gprim_transform.ExtractRotationQuat()
        gprim_quat = numpy.array([*gprim_quat.GetImaginary(), gprim_quat.GetReal()])
        self._inertial.inertia_tensor = shift_inertia_tensor(mass=self.inertial.mass,
                                                             inertia_tensor=self.inertial.inertia_tensor,
                                                             pos=gprim_pos,
                                                             quat=gprim_quat)
        self._inertial.center_of_mass = shift_center_of_mass(center_of_mass=self.inertial.center_of_mass,
                                                             pos=gprim_pos,
                                                             quat=gprim_quat)
        return self._inertial

    @property
    def origin_inertial(self) -> GeomInertial:
        origin_inertial = GeomInertial(mass=0.0,
                                       inertia_tensor=numpy.zeros((3, 3)),
                                       center_of_mass=numpy.zeros((1, 3)))
        geom_type = self.type
        if geom_type == GeomType.PLANE:
            length_x = 50.0
            length_y = 50.0
            origin_inertial.mass = self.density * length_x * length_y
            Izz = origin_inertial.mass * (length_x ** 2 + length_y ** 2) / 12
            origin_inertial.inertia_tensor = numpy.array([[0.0, 0.0, 0.0],
                                                          [0.0, 0.0, 0.0],
                                                          [0.0, 0.0, Izz]])
        elif geom_type == GeomType.CUBE:
            cube = UsdGeom.Cube(self.gprim.GetPrim())
            size = cube.GetSizeAttr().Get()
            scale_x = 1.0
            scale_y = 1.0
            scale_z = 1.0
            length_x = size * scale_x
            length_y = size * scale_y
            length_z = size * scale_z
            origin_inertial.mass = self.density * length_x * length_y * length_z
            Ixx = origin_inertial.mass * (length_y ** 2 + length_z ** 2) / 12
            Iyy = origin_inertial.mass * (length_x ** 2 + length_z ** 2) / 12
            Izz = origin_inertial.mass * (length_x ** 2 + length_y ** 2) / 12
            origin_inertial.inertia_tensor = numpy.array([[Ixx, 0.0, 0.0],
                                                          [0.0, Iyy, 0.0],
                                                          [0.0, 0.0, Izz]])
        elif geom_type == GeomType.SPHERE:
            sphere = UsdGeom.Sphere(self.gprim.GetPrim())
            radius = sphere.GetRadiusAttr().Get()
            scale_x = 1.0
            scale_y = 1.0
            scale_z = 1.0
            semi_axis_x = radius * scale_x
            semi_axis_y = radius * scale_y
            semi_axis_z = radius * scale_z
            origin_inertial.mass = self.density * 4 / 3 * numpy.pi * semi_axis_x * semi_axis_y * semi_axis_z
            Ixx = origin_inertial.mass * (semi_axis_y ** 2 + semi_axis_z ** 2) / 5
            Iyy = origin_inertial.mass * (semi_axis_x ** 2 + semi_axis_z ** 2) / 5
            Izz = origin_inertial.mass * (semi_axis_x ** 2 + semi_axis_y ** 2) / 5
            origin_inertial.inertia_tensor = numpy.array([[Ixx, 0.0, 0.0],
                                                          [0.0, Iyy, 0.0],
                                                          [0.0, 0.0, Izz]])

        elif geom_type == GeomType.CYLINDER:
            cylinder = UsdGeom.Cylinder(self.gprim.GetPrim())
            radius = cylinder.GetRadiusAttr().Get()
            height = cylinder.GetHeightAttr().Get()
            scale_x = 1.0
            scale_y = 1.0
            scale_z = 1.0
            if scale_x != scale_y:
                raise ValueError("Scale x and y must be equal.")
            radius = radius * scale_x
            height = height * scale_z
            origin_inertial.mass = self.density * numpy.pi * radius ** 2 * height
            Ixx = origin_inertial.mass * (3 * radius ** 2 + height ** 2) / 12
            Iyy = Ixx
            Izz = origin_inertial.mass * radius ** 2 / 2
            origin_inertial.inertia_tensor = numpy.array([[Ixx, 0.0, 0.0],
                                                          [0.0, Iyy, 0.0],
                                                          [0.0, 0.0, Izz]])

        elif geom_type == GeomType.CAPSULE:
            # TODO: Calculate inertia tensor for capsule
            capsule = UsdGeom.Cylinder(self.gprim.GetPrim())
            radius = capsule.GetRadiusAttr().Get()
            height = capsule.GetHeightAttr().Get()
            scale_x = 1.0
            scale_y = 1.0
            scale_z = 1.0
            if scale_x != scale_y:
                raise ValueError("Scale x and y must be equal.")
            radius = radius * scale_x
            height = height * scale_z
            origin_inertial.mass = self.density * numpy.pi * radius ** 2 * height
            Ixx = origin_inertial.mass * (3 * radius ** 2 + height ** 2) / 12
            Iyy = Ixx
            Izz = origin_inertial.mass * radius ** 2 / 2
            origin_inertial.inertia_tensor = numpy.array([[Ixx, 0.0, 0.0],
                                                          [0.0, Iyy, 0.0],
                                                          [0.0, 0.0, Izz]])

        elif geom_type == GeomType.MESH:
            mesh = UsdGeom.Mesh(self.gprim.GetPrim())
            vertices = numpy.array(mesh.GetPointsAttr().Get())
            faces = numpy.array(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)

            mesh_mass, mesh_inertia_tensor, mesh_center_of_mass = calculate_mesh_inertial(vertices=vertices,
                                                                                          faces=faces,
                                                                                          density=self.density)

            origin_inertial.mass += mesh_mass
            origin_inertial.inertia_tensor += mesh_inertia_tensor * mesh_mass
            origin_inertial.center_of_mass += mesh_center_of_mass

            if origin_inertial.mass > 0.0:
                origin_inertial.inertia_tensor /= origin_inertial.mass
        else:
            raise ValueError(f"Inertia for geom type {self.type} not supported.")

        return origin_inertial

    @property
    def inertial(self) -> GeomInertial:
        return self._inertial

    @property
    def stage(self) -> Usd.Stage:
        return self.gprim.GetPrim().GetStage()

    @property
    def gprim(self) -> UsdGeom.Gprim:
        return self._gprim

    @property
    def type(self) -> GeomType:
        return self._property.type

    @property
    def is_visible(self) -> bool:
        return self._property.is_visible

    @property
    def is_collidable(self) -> bool:
        return self._property.is_collidable

    @property
    def rgba(self) -> Optional[numpy.ndarray]:
        return self._property.rgba

    @property
    def density(self) -> float:
        return self._property.density
