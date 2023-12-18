#!/usr/bin/env python3

from __future__ import annotations
from typing import Optional, Dict, List

import numpy

from .geom_builder import GeomBuilder, GeomProperty, GeomInertial
from .joint_builder import JointBuilder, JointProperty
from ..utils import xform_cache, modify_name, diagonalize_inertia, shift_center_of_mass, shift_inertia_tensor

from pxr import Usd, UsdGeom, Sdf, Gf, UsdPhysics


class BodyBuilder:
    stage: Usd.Stage
    xform: UsdGeom.Xform
    joint_builders: List[JointBuilder]
    child_body_builders: List[BodyBuilder]

    def __init__(self,
                 stage: Usd.Stage, name: str,
                 parent_xform: Optional[UsdGeom.Xform] = None) -> None:
        path = parent_xform.GetPath().AppendPath(name) if parent_xform is not None else Sdf.Path("/").AppendPath(name)
        self._xform = UsdGeom.Xform.Define(stage, path)
        self._joint_builders: Dict[str, JointBuilder] = {}
        self._geom_builders: Dict[str, GeomBuilder] = {}
        self._child_body_builders: Dict[str, BodyBuilder] = {}

    def set_transform(
            self,
            pos: numpy.ndarray = numpy.array([0.0, 0.0, 0.0]),
            quat: numpy.ndarray = numpy.array([0.0, 0.0, 0.0, 1.0]),
            scale: numpy.ndarray = numpy.array([1.0, 1.0, 1.0]),
            relative_to_xform: Optional[UsdGeom.Xform] = None,
    ) -> None:
        """
        Set the transform of the body.
        :param pos: Array of x, y, z position.
        :param quat: Array of x, y, z, w quaternion.
        :param scale: Array of x, y, z scale.
        :param relative_to_xform: Relative transform prim to apply the transform to.
        :return: None
        """
        mat = Gf.Matrix4d()
        mat.SetTranslateOnly(Gf.Vec3d(*pos))
        mat.SetRotateOnly(Gf.Quatd(quat[3], Gf.Vec3d(*quat[:3])))
        mat_scale = Gf.Matrix4d()
        mat_scale.SetScale(Gf.Vec3d(*scale))
        mat = mat_scale * mat

        if relative_to_xform is not None:
            relative_to_prim = relative_to_xform.GetPrim()
            if relative_to_prim:
                parent_prim = self._xform.GetPrim().GetParent()
                if parent_prim.IsValid() and parent_prim != relative_to_prim:
                    parent_to_relative_mat, _ = xform_cache.ComputeRelativeTransform(relative_to_prim, parent_prim)
                    mat = mat * parent_to_relative_mat
            else:
                raise ValueError(f"Prim at path {relative_to_xform.GetPath()} not found.")

        self._xform.AddTransformOp().Set(mat)

    def add_joint(self, joint_property: JointProperty) -> JointBuilder:
        joint_name = joint_property.name
        if joint_name in self._joint_builders:
            print(f"Joint {joint_name} already exists.")
            joint_builder = self._joint_builders[joint_name]
        else:
            joint_builder = JointBuilder(joint_property)
            joint_builder.build()
            self._joint_builders[joint_name] = joint_builder

        return joint_builder

    def enable_rigid_body(self) -> None:
        physics_rigid_body_api = UsdPhysics.RigidBodyAPI(self._xform)
        physics_rigid_body_api.CreateRigidBodyEnabledAttr(True)
        physics_rigid_body_api.Apply(self._xform.GetPrim())

    def add_geom(self, geom_property: GeomProperty) -> GeomBuilder:
        geom_name = geom_property.name
        if geom_name in self._geom_builders:
            print(f"Geom {geom_name} already exists.")
            geom_builder = self._geom_builders[geom_name]
        else:
            geom_builder = GeomBuilder(
                stage=self.stage,
                geom_name=geom_name,
                body_path=self._xform.GetPath(),
                geom_property=geom_property
            )
            self._geom_builders[geom_name] = geom_builder

        return geom_builder

    def get_joint_builder(self, joint_name: str) -> JointBuilder:
        joint_name = modify_name(in_name=joint_name)
        if joint_name not in self._joint_builders:
            raise ValueError(f"Joint {joint_name} not found in {self.__class__.__name__}.")
        return self._joint_builders[joint_name]

    def get_geom_builder(self, geom_name: str) -> GeomBuilder:
        geom_name = modify_name(in_name=geom_name)
        if geom_name not in self._joint_builders:
            raise ValueError(f"Geom {geom_name} not found in {self.__class__.__name__}.")
        return self._geom_builders[geom_name]

    def set_inertial(self,
                     mass: float,
                     center_of_mass: numpy.ndarray,
                     diagonal_inertia: numpy.ndarray,
                     principal_axes: numpy.ndarray = numpy.array([0.0, 0.0, 0.0, 1.0])) -> UsdPhysics.MassAPI:
        physics_mass_api = UsdPhysics.MassAPI(self.xform)
        physics_mass_api.CreateMassAttr(mass)
        physics_mass_api.CreateCenterOfMassAttr(Gf.Vec3f(*center_of_mass))
        physics_mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*diagonal_inertia))
        physics_mass_api.CreatePrincipalAxesAttr(Gf.Quatf(principal_axes[3], *principal_axes[:3]))
        physics_mass_api.Apply(self.xform.GetPrim())

        return physics_mass_api

    def compute_and_set_inertial(self) -> (GeomInertial, UsdPhysics.MassAPI):
        body_inertial = GeomInertial(mass=0.0,
                                     center_of_mass=numpy.zeros((1, 3)),
                                     inertia_tensor=numpy.zeros((3, 3)))
        for child_body_builder in self.child_body_builders:
            child_body_inertial, _ = child_body_builder.compute_and_set_inertial()
            body_inertial.mass += child_body_inertial.mass
            body_inertial.center_of_mass += child_body_inertial.center_of_mass * child_body_inertial.mass
            body_inertial.inertia_tensor += child_body_inertial.inertia_tensor

        if body_inertial.mass > 0.0:
            body_inertial.center_of_mass /= body_inertial.mass

        for geom_builder in self._geom_builders.values():
            geom_inertial = geom_builder.calculate_inertial()
            body_inertial.mass += geom_inertial.mass
            body_inertial.center_of_mass += geom_inertial.center_of_mass * geom_inertial.mass
            body_inertial.inertia_tensor += geom_inertial.inertia_tensor

        if body_inertial.mass > 0.0:
            body_inertial.center_of_mass /= body_inertial.mass

        xform_transform = self.xform.GetLocalTransformation()
        xform_pos = xform_transform.ExtractTranslation()
        xform_pos = numpy.array(xform_pos)
        xform_quat = xform_transform.ExtractRotationQuat()
        xform_quat = numpy.array([*xform_quat.GetImaginary(), xform_quat.GetReal()])

        body_center_of_mass = shift_center_of_mass(center_of_mass=body_inertial.center_of_mass,
                                                   pos=xform_pos,
                                                   quat=xform_quat)
        body_inertia_tensor = shift_inertia_tensor(mass=body_inertial.mass,
                                                   inertia_tensor=body_inertial.inertia_tensor)

        diagonal_inertia, principal_axes = diagonalize_inertia(inertia_tensor=body_inertia_tensor)

        return body_inertial, self.set_inertial(mass=body_inertial.mass,
                                                center_of_mass=body_center_of_mass[0],
                                                diagonal_inertia=diagonal_inertia,
                                                principal_axes=principal_axes)

    def add_child_body_builder(self, child_body_builder: BodyBuilder) -> None:
        child_body_name = child_body_builder.xform.GetPrim().GetName()
        if child_body_name in self._child_body_builders:
            print(f"Child body {child_body_name} already exists.")
        else:
            self._child_body_builders[child_body_name] = child_body_builder

    @property
    def stage(self) -> Usd.Stage:
        return self.xform.GetPrim().GetStage()

    @property
    def xform(self) -> Usd.Stage:
        return self._xform

    @property
    def joint_builders(self) -> List[JointBuilder]:
        return list(self._joint_builders.values())

    @property
    def child_body_builders(self) -> List[BodyBuilder]:
        return list(self._child_body_builders.values())
