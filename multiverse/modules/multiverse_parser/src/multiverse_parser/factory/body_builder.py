#!/usr/bin/env python3.10

from typing import Dict, Optional

from pxr import Usd, UsdGeom, Sdf, Gf, UsdPhysics

from ..utils import xform_cache, modify_name

# from .geom_builder import GeomBuilder, GeomType, geom_dict
from .joint_builder import JointBuilder, JointType


class BodyBuilder:
    stage: Usd.Stage
    path: Sdf.Path
    xform: UsdGeom.Xform
    joint_builders: Dict[str, JointBuilder]

    def __init__(self, stage: Usd.Stage, name: str, parent_xform: Optional[UsdGeom.Xform] = None) -> None:
        if parent_xform is not None:
            self.path = parent_xform.GetPath().AppendPath(name)
        else:
            self.path = Sdf.Path("/").AppendPath(name)
        self.stage = stage
        self.xform = UsdGeom.Xform.Define(self.stage, self.path)
        self.joint_builders = {}
        # self.geom_names = set()
        # self.joint_names = set()

    def set_transform(
            self,
            pos: tuple = (0.0, 0.0, 0.0),
            quat: tuple = (1.0, 0.0, 0.0, 0.0),
            scale: tuple = (1.0, 1.0, 1.0),
            relative_to_xform: Optional[UsdGeom.Xform] = None,
    ) -> None:
        """
        Set transform of the body.
        :param pos: Tuple of (x, y, z) position.
        :param quat: Tuple of (w, x, y, z) quaternion.
        :param scale: Tuple of (x, y, z) scale.
        :param relative_to_xform: Xform to set the transform relative to.
        :return: None
        """
        mat = Gf.Matrix4d()
        mat.SetTranslateOnly(pos)
        mat.SetRotateOnly(Gf.Quatd(quat[0], Gf.Vec3d(quat[1], quat[2], quat[3])))
        mat_scale = Gf.Matrix4d()
        mat_scale.SetScale(Gf.Vec3d(scale))
        mat = mat_scale * mat

        if relative_to_xform is not None:
            relative_to_prim = relative_to_xform.GetPrim()
            if relative_to_prim:
                parent_prim = self.xform.GetPrim().GetParent()
                if parent_prim.IsValid() and parent_prim != relative_to_prim:
                    parent_to_relative_mat, _ = xform_cache.ComputeRelativeTransform(relative_to_prim, parent_prim)
                    mat = mat * parent_to_relative_mat
            else:
                raise ValueError(f"Prim at path {relative_to_xform.GetPath()} not found.")

        self.xform.AddTransformOp().Set(mat)

    #
    # def add_geom(self, geom_name: str, geom_type: GeomType, is_visual: bool) -> GeomBuilder:
    #     goem_name = modify_name(in_name=geom_name)
    #
    #     if goem_name in geom_dict:
    #         print(f"Geom {goem_name} already exists.")
    #         geom_builder = geom_dict[goem_name]
    #     else:
    #         self.geom_names.add(geom_name)
    #         geom_builder = GeomBuilder(stage=self.stage, geom_name=geom_name, body_path=self.path, geom_type=geom_type, is_visual=is_visual)
    #     return geom_builder

    def add_joint(
        self,
        joint_name: str,
        parent_prim: Usd.Prim,
        joint_type: JointType,
        joint_pos: tuple = (0.0, 0.0, 0.0),
        joint_quat: tuple = None,
        joint_axis: str = "Z",
    ) -> JointBuilder:
        joint_name = modify_name(in_name=joint_name)

        if joint_name in self.joint_builders:
            print(f"Joint {joint_name} already exists.")
            joint_builder = self.joint_builders[joint_name]
        else:
            joint_builder = JointBuilder(
                stage=self.stage,
                name=joint_name,
                parent_prim=parent_prim,
                child_prim=self.xform.GetPrim(),
                joint_type=joint_type,
                joint_pos=joint_pos,
                joint_quat=joint_quat,
                joint_axis=joint_axis,
            )

        return joint_builder

    def enable_rigid_body(self) -> None:
        physics_rigid_body_api = UsdPhysics.RigidBodyAPI(self.xform)
        physics_rigid_body_api.CreateRigidBodyEnabledAttr(True)
        physics_rigid_body_api.Apply(self.xform.GetPrim())

    # def set_inertial(
    #     self,
    #     mass: float = 1e-1,
    #     com: tuple = (0.0, 0.0, 0.0),
    #     diagonal_inertia: tuple = (1e-3, 1e-3, 1e-3),
    #     density: float = 100,
    #     principal_axes: tuple = (1, 0, 0, 0),
    # ) -> None:
    #     physics_mass_api = UsdPhysics.MassAPI(self.xform)
    #     physics_mass_api.CreateMassAttr(mass)
    #     physics_mass_api.CreateCenterOfMassAttr(Gf.Vec3f(com))
    #     physics_mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(diagonal_inertia))
    #     physics_mass_api.CreateDensityAttr(density)
    #     physics_mass_api.CreatePrincipalAxesAttr(Gf.Quatf(principal_axes[0], Gf.Vec3f(principal_axes[1], principal_axes[2], principal_axes[3])))
    #     physics_mass_api.Apply(self.xform.GetPrim())
