#!/usr/bin/env python3.10

from typing import Dict, Optional

import numpy
from pxr import Usd, UsdGeom, Sdf, Gf, UsdPhysics

from ..utils import xform_cache, modify_name

from .geom_builder import GeomBuilder, GeomProperty
from .joint_builder import JointBuilder, JointProperty


class BodyBuilder:
    _stage: Usd.Stage
    _xform: UsdGeom.Xform
    _joint_builders: Dict[str, JointBuilder]
    _geom_builders: Dict[str, GeomBuilder]

    def __init__(self, stage: Usd.Stage, name: str, parent_xform: Optional[UsdGeom.Xform] = None) -> None:
        path = parent_xform.GetPath().AppendPath(name) if parent_xform is not None else Sdf.Path("/").AppendPath(name)
        self._stage = stage
        self._xform = UsdGeom.Xform.Define(self._stage, path)
        self._joint_builders = {}
        self._geom_builders = {}

    def set_transform(
            self,
            pos: numpy.ndarray = numpy.array([0.0, 0.0, 0.0]),
            quat: numpy.ndarray = numpy.array([0.0, 0.0, 0.0]),
            scale: numpy.ndarray = numpy.array([1.0, 1.0, 1.0]),
            relative_to_xform: Optional[UsdGeom.Xform] = None,
    ) -> None:
        """
        Set the transform of the body.
        :param pos: Array of x, y, z position.
        :param quat: Array of w, x, y, z quaternion.
        :param scale: Array of x, y, z scale.
        :param relative_to_xform: Relative transform prim to apply the transform to.
        :return: None
        """
        mat = Gf.Matrix4d()
        mat.SetTranslateOnly(Gf.Vec3d(*pos))
        mat.SetRotateOnly(Gf.Quatd(quat[0], Gf.Vec3d(quat[1], quat[2], quat[3])))
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
        geom_type = geom_property.type
        if geom_name in self._geom_builders:
            print(f"Geom {geom_name} already exists.")
            geom_builder = self._geom_builders[geom_name]
        else:
            geom_builder = GeomBuilder(
                stage=self._stage,
                geom_name=geom_name,
                body_path=self._xform.GetPath(),
                geom_type=geom_type,
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

    @property
    def xform(self) -> Usd.Stage:
        return self._xform
