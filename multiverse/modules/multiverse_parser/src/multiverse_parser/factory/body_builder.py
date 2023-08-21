#!/usr/bin/env python3.10

from pxr import Usd, UsdGeom, Sdf, Gf, UsdPhysics

from multiverse_parser.utils import modify_name, xform_cache
from .geom_builder import GeomBuilder, GeomType, geom_dict
from .joint_builder import JointBuilder, JointType, joint_dict

body_dict = {}


class BodyBuilder:
    def __init__(self, stage: Usd.Stage, name: str, parent_name: str = None) -> None:
        body_dict[name] = self
        if parent_name is not None:
            parent_xform = body_dict.get(parent_name).xform
            if parent_xform.GetPrim().IsValid():
                self.path = parent_xform.GetPath().AppendPath(name)
            else:
                print(f"Parent xform with name {parent_name} not found.")
                return
        else:
            self.path = Sdf.Path("/").AppendPath(name)
        self.stage = stage
        self.xform = UsdGeom.Xform.Define(self.stage, self.path)
        self.geom_names = set()
        self.joint_names = set()

    def set_transform(
        self,
        pos: tuple = (0.0, 0.0, 0.0),
        quat: tuple = (1.0, 0.0, 0.0, 0.0),
        scale: tuple = (1.0, 1.0, 1.0),
        relative_to: str = None,
    ) -> None:
        mat = Gf.Matrix4d()
        mat.SetTranslateOnly(pos)
        mat.SetRotateOnly(Gf.Quatd(quat[0], Gf.Vec3d(quat[1], quat[2], quat[3])))
        mat_scale = Gf.Matrix4d()
        mat_scale.SetScale(Gf.Vec3d(scale))
        mat = mat_scale * mat

        if relative_to is not None:
            relative_prim = body_dict[relative_to].xform.GetPrim()
            if relative_prim:
                parent_prim = self.xform.GetPrim().GetParent()
                if parent_prim.IsValid() and parent_prim != relative_prim:
                    parent_to_relative_mat, _ = xform_cache.ComputeRelativeTransform(relative_prim, parent_prim)
                    mat = mat * parent_to_relative_mat
            else:
                print(f"Prim at path {relative_to} not found.")

        self.xform.AddTransformOp().Set(mat)

    def add_geom(self, geom_name: str, geom_type: GeomType) -> GeomBuilder:
        goem_name = modify_name(in_name=geom_name)

        if goem_name in geom_dict:
            print(f"Geom {goem_name} already exists.")
            geom = geom_dict[goem_name]
        else:
            self.geom_names.add(geom_name)
            geom = GeomBuilder(stage=self.stage, geom_name=geom_name, body_path=self.path, geom_type=geom_type)
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
        joint_name = modify_name(in_name=joint_name)

        if joint_name in joint_dict:
            print(f"Joint {joint_name} already exists.")
            joint = joint_dict[joint_name]
        else:
            self.joint_names.add(joint_name)
            parent_name = modify_name(parent_name)
            child_name = modify_name(child_name)
            if body_dict.get(parent_name) is None or body_dict.get(child_name) is None:
                return None

            joint = JointBuilder(
                self.stage,
                joint_name,
                body_dict[parent_name].xform,
                body_dict[child_name].xform,
                joint_type,
                joint_pos,
                joint_axis,
            )

        return joint

    def enable_collision(self) -> None:
        physics_rigid_body_api = UsdPhysics.RigidBodyAPI(self.xform)
        physics_rigid_body_api.CreateRigidBodyEnabledAttr(True)
        physics_rigid_body_api.Apply(self.xform.GetPrim())

        for geom_name in self.geom_names:
            geom_dict[geom_name].enable_collision()

    def set_inertial(
        self,
        mass: float = 1e-1,
        com: tuple = (0.0, 0.0, 0.0),
        diagonal_inertia: tuple = (1e-3, 1e-3, 1e-3),
        density: float = 100,
        principal_axes: tuple = (1, 0, 0, 0),
    ) -> None:
        physics_mass_api = UsdPhysics.MassAPI(self.xform)
        physics_mass_api.CreateMassAttr(mass)
        physics_mass_api.CreateCenterOfMassAttr(Gf.Vec3f(com))
        physics_mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(diagonal_inertia))
        physics_mass_api.CreateDensityAttr(density)
        physics_mass_api.CreatePrincipalAxesAttr(Gf.Quatf(principal_axes[0], Gf.Vec3f(principal_axes[1], principal_axes[2], principal_axes[3])))
        physics_mass_api.Apply(self.xform.GetPrim())
