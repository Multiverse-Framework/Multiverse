#!/usr/bin/env python3.10

import os
from pxr import Usd, UsdGeom, Sdf, Gf, UsdPhysics

from .geom_builder import GeomBuilder, GeomType
from .joint_builder import JointBuilder, JointType, joint_dict

body_dict = {}

xform_cache = UsdGeom.XformCache()


class BodyBuilder:
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
        self.prim = UsdGeom.Xform.Define(self.stage, self.path)
        self.pos = Gf.Vec3d(0.0, 0.0, 0.0)
        self.quat = Gf.Quatd(1.0, 0.0, 0.0, 0.0)
        self.scale = Gf.Vec3d(1.0, 1.0, 1.0)
        self.geoms = set()
        self.joints = set()

    def set_transform(
        self,
        pos: tuple = (0.0, 0.0, 0.0),
        quat: tuple = (1.0, 0.0, 0.0, 0.0),
        scale: tuple = (1.0, 1.0, 1.0),
        relative_to: str = None,
    ):
        self.pos = Gf.Vec3d(pos)
        self.quat = Gf.Quatd(quat[0], Gf.Vec3d(quat[1], quat[2], quat[3]))
        self.scale = Gf.Vec3d(scale)

        mat = Gf.Matrix4d()
        mat.SetTranslateOnly(self.pos)
        mat.SetRotateOnly(self.quat)
        mat_scale = Gf.Matrix4d()
        mat_scale.SetScale(Gf.Vec3d(self.scale))
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

    def add_geom(self, geom_name: str, geom_type: GeomType) -> GeomBuilder:
        geom = GeomBuilder(self.stage, geom_name, self.path, geom_type)
        self.geoms.add(geom)
        return geom

    def add_joint(
        self,
        joint_name: str,
        parent_name: str,
        child_name: str,
        joint_type: JointType,
        joint_pos: tuple = (0.0, 0.0, 0.0),
        joint_axis: str = "Z",
    ) -> JointBuilder:
        if joint_name in joint_dict:
            print(f"Joint {joint_name} already exists.")
            joint = joint_dict[joint_name]
        else:
            if body_dict.get(parent_name) is None or body_dict.get(child_name) is None:
                return None

            joint = JointBuilder(
                self.stage,
                joint_name,
                body_dict[parent_name],
                body_dict[child_name],
                joint_type,
                joint_pos,
                joint_axis,
            )
            self.joints.add(joint)
        return joint

    def enable_collision(self) -> None:
        physics_rigid_body_api = UsdPhysics.RigidBodyAPI(self.prim)
        physics_rigid_body_api.CreateRigidBodyEnabledAttr(True)
        physics_rigid_body_api.Apply(self.prim.GetPrim())

        for geom in self.geoms:
            geom.enable_collision()

    def set_inertial(
        self,
        mass: float = 1e-9,
        com: tuple = (0.0, 0.0, 0.0),
        diagonal_inertia: tuple = (0.0, 0.0, 0.0),
    ) -> None:
        physics_mass_api = UsdPhysics.MassAPI(self.prim)
        physics_mass_api.CreateMassAttr(mass)
        physics_mass_api.CreateCenterOfMassAttr(Gf.Vec3f(com))
        physics_mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(diagonal_inertia))
        physics_mass_api.Apply(self.prim.GetPrim())
