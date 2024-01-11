#!/usr/bin/env python3

from dataclasses import dataclass
from typing import Optional, List, Dict, Tuple
from enum import Enum

import numpy

from .material_builder import MaterialBuilder
from .mesh_builder import MeshBuilder
from ..utils import modify_name, calculate_mesh_inertial, shift_inertia_tensor, shift_center_of_mass

from pxr import Usd, UsdGeom, Gf, Sdf, UsdShade, UsdPhysics


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


def reference_materials(material_builder: MaterialBuilder, stage: Usd.Stage, mesh_file_path: str):
    material_root_path = material_builder.root_prim.GetPath()
    material_root_prim = stage.GetPrimAtPath(material_root_path)
    if not material_root_prim.IsValid():
        material_root_prim = stage.DefinePrim(material_root_path)
    material_root_prim.GetReferences().AddReference(mesh_file_path, material_root_path)


def bind_materials(reference_stage: Usd.Stage, local_stage: Usd.Stage, reference_prim: Usd.Prim, local_prim: Usd.Prim):
    print(f"Binding material for {local_prim.GetPath()}")
    reference_material_binding_api = UsdShade.MaterialBindingAPI(reference_prim)
    material_binding_api = UsdShade.MaterialBindingAPI.Apply(local_prim)

    for path in reference_material_binding_api.GetDirectBindingRel().GetTargets():
        material = UsdShade.Material(reference_stage.GetPrimAtPath(path))
        material_binding_api.Bind(material)

    for reference_geom_subset in reference_material_binding_api.GetMaterialBindSubsets():
        local_geom_subset = local_stage.OverridePrim(
            local_prim.GetPath().AppendPath(reference_geom_subset.GetPrim().GetName()))

        bind_materials(reference_stage, local_stage, reference_geom_subset.GetPrim(), local_geom_subset)


class GeomBuilder:
    stage: Usd.Stage
    xform: UsdGeom.Xform
    geom_prims: List[UsdGeom.Gprim]
    mesh_builders: Dict[str, MeshBuilder]
    material_builders: List[MaterialBuilder]

    def __init__(self,
                 stage: Usd.Stage,
                 geom_name: str,
                 body_path: Sdf.Path,
                 geom_property: GeomProperty,
                 geom_inertia: GeomInertial = None) -> None:
        self._property = geom_property
        self._xform = UsdGeom.Xform.Define(stage, body_path.AppendPath(geom_name))
        self._mesh_builders = {}
        self._material_builders = {}
        self._geom_prims = self._create_geoms()
        self._inertial = geom_inertia

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

    def add_mesh(self, mesh_file_path: str) -> Tuple[MeshBuilder, MaterialBuilder]:
        if mesh_file_path in self.mesh_builders:
            mesh_builder = self.mesh_builders[mesh_file_path]
        else:
            mesh_builder = MeshBuilder(mesh_file_path)
            self.mesh_builders[mesh_file_path] = mesh_builder

        if mesh_file_path in self._material_builders:
            material_builder = self._material_builders[mesh_file_path]
        else:
            material_builder = MaterialBuilder(file_path=mesh_file_path)
            self._material_builders[mesh_file_path] = material_builder

        reference_prims = [mesh.GetPrim() for mesh in mesh_builder.xform.GetPrim().GetChildren()]
        for reference_prim in reference_prims:
            local_prim = self.stage.OverridePrim(self.xform.GetPath().AppendPath(reference_prim.GetName()))
            self.xform.GetPrim().GetReferences().AddReference(mesh_file_path, mesh_builder.xform.GetPath())
            if reference_prim.HasAPI(UsdShade.MaterialBindingAPI):
                bind_materials(mesh_builder.stage, self.stage, reference_prim, local_prim)

        reference_materials(material_builder, self.stage, mesh_file_path)

        return mesh_builder, material_builder

    def build(self) -> List[UsdGeom.Gprim]:
        for geom in self.geom_prims:
            if self.rgba is not None:
                geom.CreateDisplayColorAttr(self.rgba[:3])
                geom.CreateDisplayOpacityAttr(self.rgba[3])
            if not self.is_visible:
                geom.CreateDisplayOpacityAttr([0.0])
                geom.CreateVisibilityAttr("invisible")

        if self.is_collidable:
            for geom in self.geom_prims:
                physics_collision_api = UsdPhysics.CollisionAPI(geom)
                physics_collision_api.CreateCollisionEnabledAttr(True)
                physics_collision_api.Apply(geom.GetPrim())
                if self.type == GeomType.MESH:
                    physics_mesh_collision_api = UsdPhysics.MeshCollisionAPI(geom)
                    physics_mesh_collision_api.CreateApproximationAttr("convexHull")
                    physics_mesh_collision_api.Apply(geom.GetPrim())

        return self.geom_prims

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
        mat = Gf.Matrix4d()
        mat.SetTranslateOnly(Gf.Vec3d(*pos))
        mat.SetRotateOnly(Gf.Quatd(quat[3], Gf.Vec3d(*quat[:3])))
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

    def calculate_inertial(self) -> GeomInertial:
        self._inertial = self.origin_inertial

        xform_transform = self.xform.GetLocalTransformation()
        xform_pos = xform_transform.ExtractTranslation()
        xform_pos = numpy.array([[*xform_pos]])
        xform_quat = xform_transform.ExtractRotationQuat()
        xform_quat = numpy.array([*xform_quat.GetImaginary(), xform_quat.GetReal()])
        self._inertial.inertia_tensor = shift_inertia_tensor(mass=self._inertial.mass,
                                                             inertia_tensor=self._inertial.inertia_tensor,
                                                             pos=xform_pos,
                                                             quat=xform_quat)
        self._inertial.center_of_mass = shift_center_of_mass(center_of_mass=self._inertial.center_of_mass,
                                                             pos=xform_pos,
                                                             quat=xform_quat)
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
            cube = UsdGeom.Cube(self.geom_prims[0])
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
            sphere = UsdGeom.Sphere(self.geom_prims[0])
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
            cylinder = UsdGeom.Cylinder(self.geom_prims[0])
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
            capsule = UsdGeom.Cylinder(self.geom_prims[0])
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
            for mesh_builder in self.mesh_builders.values():
                for mesh in mesh_builder.meshes:
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
        return self.xform.GetPrim().GetStage()

    @property
    def geom_prims(self) -> List[UsdGeom.Gprim]:
        return [UsdGeom.Gprim(prim) for prim in self.xform.GetPrim().GetChildren()]

    @property
    def xform(self) -> UsdGeom.Xform:
        return self._xform

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

    @property
    def mesh_builders(self) -> Dict[str, MeshBuilder]:
        return self._mesh_builders

    @property
    def material_builders(self) -> List[MaterialBuilder]:
        return list(self._material_builders.values())
