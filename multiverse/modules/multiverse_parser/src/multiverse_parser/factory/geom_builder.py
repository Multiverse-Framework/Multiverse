#!/usr/bin/env python3.10

import os
from pxr import Usd, UsdGeom, Gf, UsdPhysics, Sdf, UsdShade
import bpy, bmesh
from enum import Enum
from mathutils import Matrix

from multiverse_parser.factory import TMP_DIR
from multiverse_parser.utils import modify_name
from .mesh_builder import (
    MeshBuilder,
    VisualMeshBuilder,
    CollisionMeshBuilder,
    mesh_dict,
)

geom_dict = {}


class GeomType(Enum):
    PLANE = 0
    CUBE = 1
    SPHERE = 2
    CYLINDER = 3
    MESH = 4


def inertia_of_triangle(v1, v2, v3, density) -> Matrix:
    # Compute the area of the triangle
    area = 0.5 * ((v2 - v1).cross(v3 - v1)).length
    # Compute the inertia tensor of the triangle
    inertia = Matrix()
    for i in range(3):
        for j in range(3):
            inertia[i][j] = (
                density
                * area
                / 12
                * (
                    (v1[i] ** 2 + v1[j] ** 2 + v2[i] ** 2 + v2[j] ** 2 + v3[i] ** 2 + v3[j] ** 2)
                    + (v1[i] * v2[i] + v1[j] * v2[j] + v2[i] * v3[i] + v2[j] * v3[j] + v1[i] * v3[i] + v1[j] * v3[j])
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
    def __init__(self, stage: Usd.Stage, geom_name: str, body_path: Sdf.Path, geom_type: GeomType) -> None:
        geom_dict[geom_name] = self
        self.stage = stage
        self.name = geom_name
        self.path = body_path.AppendPath(self.name)
        self.type = geom_type
        self.set_prim()
        self.pos = Gf.Vec3d(0.0, 0.0, 0.0)
        self.quat = Gf.Quatd(1.0, 0.0, 0.0, 0.0)
        self.scale = Gf.Vec3d(1.0, 1.0, 1.0)

    def set_prim(self) -> None:
        self.root_prim = UsdGeom.Xform.Define(self.stage, self.path)
        if self.type == GeomType.PLANE:
            self.geom_prim = UsdGeom.Mesh.Define(self.stage, self.path.AppendPath("Plane"))
            self.geom_prim.CreatePointsAttr([(-0.5, -0.5, 0), (0.5, -0.5, 0), (-0.5, 0.5, 0), (0.5, 0.5, 0)])
            self.geom_prim.CreateNormalsAttr([(0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1)])
            self.geom_prim.CreateFaceVertexCountsAttr([4])
            self.geom_prim.CreateFaceVertexIndicesAttr([0, 1, 3, 2])
        elif self.type == GeomType.CUBE:
            self.geom_prim = UsdGeom.Cube.Define(self.stage, self.path.AppendPath("Cube"))
        elif self.type == GeomType.SPHERE:
            self.geom_prim = UsdGeom.Sphere.Define(self.stage, self.path.AppendPath("Sphere"))
        elif self.type == GeomType.CYLINDER:
            self.geom_prim = UsdGeom.Cylinder.Define(self.stage, self.path.AppendPath("Cylinder"))
        elif self.type == GeomType.MESH:
            self.geom_prim = None

    def set_transform(
        self,
        pos: tuple = (0.0, 0.0, 0.0),
        quat: tuple = (1.0, 0.0, 0.0, 0.0),
        scale: tuple = (1.0, 1.0, 1.0),
    ) -> None:
        self.pos = Gf.Vec3d(pos)
        self.quat = Gf.Quatd(quat[0], Gf.Vec3d(quat[1], quat[2], quat[3]))
        self.scale = Gf.Vec3d(scale)

        mat = Gf.Matrix4d()
        mat.SetTranslateOnly(self.pos)
        mat.SetRotateOnly(self.quat)
        mat_scale = Gf.Matrix4d()
        mat_scale.SetScale(self.scale)
        mat = mat_scale * mat
        self.root_prim.AddTransformOp().Set(mat)

    def set_attribute(self, prefix: str = None, **kwargs) -> None:
        for key, value in kwargs.items():
            attr = prefix + ":" + key if prefix is not None else key
            if self.geom_prim.GetPrim().HasAttribute(attr):
                self.geom_prim.GetPrim().GetAttribute(attr).Set(value)

    def compute_extent(self) -> None:
        if self.type == GeomType.PLANE:
            self.geom_prim.CreateExtentAttr([(-0.5, -0.5, 0), (0.5, 0.5, 0)])
        elif self.type == GeomType.CUBE:
            self.geom_prim.CreateExtentAttr(((-1, -1, -1), (1, 1, 1)))
        elif self.type == GeomType.SPHERE:
            radius = self.geom_prim.GetRadiusAttr().Get()
            self.geom_prim.CreateExtentAttr(((-radius, -radius, -radius), (radius, radius, radius)))
        elif self.type == GeomType.CYLINDER:
            radius = self.geom_prim.GetRadiusAttr().Get()
            height = self.geom_prim.GetHeightAttr().Get()
            self.geom_prim.CreateExtentAttr(((-radius, -radius, -height / 2), (radius, radius, height / 2)))

    def add_mesh(self, mesh_name: str = None, visual: bool = True, material_name: str = None) -> MeshBuilder:
        mesh_name = modify_name(in_name=mesh_name)
        if mesh_name is None:
            mesh_name = "SM_" + self.name

        mesh_path = os.path.join(TMP_DIR, "visual" if visual else "collision", mesh_name + ".usda")
        mesh_ref = "./" + mesh_path

        if mesh_name in mesh_dict:
            mesh = mesh_dict[mesh_name]
        else:
            from multiverse_parser.factory import TMP_USD_FILE_DIR

            usd_file_path = os.path.join(TMP_USD_FILE_DIR, mesh_path)
            if visual:
                mesh = VisualMeshBuilder(mesh_name, usd_file_path)
            else:
                mesh = CollisionMeshBuilder(mesh_name, usd_file_path)

        self.geom_prim = self.stage.OverridePrim(self.path.AppendPath(mesh.mesh_prim.GetPrim().GetName()))

        self.root_prim.GetPrim().GetReferences().AddReference(mesh_ref, mesh.root_prim.GetPath())

        if visual:
            if material_name is None:
                material_name = "M_" + mesh_name.replace("SM_", "", 1)
            material = mesh.add_material(material_name=material_name)

            self.geom_prim = self.stage.OverridePrim(self.path.AppendPath(mesh.mesh_prim.GetPrim().GetName()))

            paths = mesh.root_prim.GetPrim().FindAllRelationshipTargetPaths()
            if len(paths) > 0 and mesh.mesh_prim.GetPrim().HasAPI(UsdShade.MaterialBindingAPI):
                material_binding_API = UsdShade.MaterialBindingAPI.Apply(self.geom_prim.GetPrim())
                for path in paths:
                    if UsdShade.Material(mesh.stage.GetPrimAtPath(path)):
                        material_binding_API.Bind(UsdShade.Material(mesh.stage.GetPrimAtPath(path)))

            prims_with_material = [
                prim
                for prim in mesh.stage.TraverseAll()
                if prim != self.geom_prim.GetPrim() and len(prim.FindAllRelationshipTargetPaths()) > 0 and prim.HasAPI(UsdShade.MaterialBindingAPI)
            ]
            for prim_with_material in prims_with_material:
                parent_prim = prim_with_material
                prim_path = prim_with_material.GetName()
                while not self.stage.GetPrimAtPath(self.path.AppendPath(prim_path)).IsValid() and parent_prim.IsValid():
                    parent_prim = parent_prim.GetParent()
                    prim_path = parent_prim.GetName() + "/" + prim_path

                prim = self.stage.OverridePrim(self.path.AppendPath(prim_path))
                material_binding_API = UsdShade.MaterialBindingAPI.Apply(prim)
                for path in prim_with_material.FindAllRelationshipTargetPaths():
                    if UsdShade.Material(mesh.stage.GetPrimAtPath(path)):
                        material_binding_API.Bind(UsdShade.Material(mesh.stage.GetPrimAtPath(path)))

            material_root_prim = self.stage.GetPrimAtPath(material.root_prim.GetPath())
            if not material_root_prim.IsValid():
                material_root_prim = self.stage.DefinePrim(material.root_prim.GetPath())

            material_root_prim.GetReferences().AddReference(mesh_ref, material.root_prim.GetPath())

        return mesh

    def enable_collision(self) -> None:
        physics_collision_api = UsdPhysics.CollisionAPI(self.geom_prim)
        physics_collision_api.CreateCollisionEnabledAttr(True)
        physics_collision_api.Apply(self.geom_prim.GetPrim())

        if self.type == GeomType.MESH:
            physics_mesh_collision_api = UsdPhysics.MeshCollisionAPI(self.geom_prim)
            physics_mesh_collision_api.CreateApproximationAttr("convexHull")
            physics_mesh_collision_api.Apply(self.geom_prim.GetPrim())

    def compute_inertial(self, density: float = 100) -> None:
        from multiverse_parser.utils import diagonalize_inertia

        obj = bpy.context.object
        mesh = bmesh.new()
        mesh.from_mesh(obj.data)
        volume = mesh.calc_volume()
        com = tuple(obj.location)

        mass = volume * density

        # Compute the inertia tensor
        mesh.verts.ensure_lookup_table()

        # Compute the inertia tensor by iterating through the faces
        inertia = Matrix.Identity(3)
        for i in range(3):
            for j in range(3):
                inertia[i][j] = 0

        for face in mesh.faces:
            v1, v2, v3 = [v.co for v in face.verts]
            inertia_add = inertia_of_triangle(v1, v2, v3, density)
            for i in range(3):
                for j in range(3):
                    inertia[i][j] += inertia_add[i][j]

        diagonal_inertia, principal_axes = diagonalize_inertia(inertia)
        self.set_inertial(mass=mass, com=com, diagonal_inertia=diagonal_inertia, density=density, principal_axes=principal_axes)

    def set_inertial(
        self,
        mass: float = 1e-1,
        com: tuple = (0.0, 0.0, 0.0),
        diagonal_inertia: tuple = (1e-3, 1e-3, 1e-3),
        density: float = 100,
        principal_axes: tuple = (1, 0, 0, 0),
    ) -> None:
        physics_mass_api = UsdPhysics.MassAPI(self.root_prim)
        physics_mass_api.CreateMassAttr(mass)
        physics_mass_api.CreateCenterOfMassAttr(Gf.Vec3f(com))
        physics_mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(diagonal_inertia))
        physics_mass_api.CreateDensityAttr(density)
        physics_mass_api.CreatePrincipalAxesAttr(Gf.Quatf(principal_axes[0], Gf.Vec3f(principal_axes[1], principal_axes[2], principal_axes[3])))
        physics_mass_api.Apply(self.root_prim.GetPrim())
