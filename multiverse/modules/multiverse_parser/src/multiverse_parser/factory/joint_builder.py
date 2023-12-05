#!/usr/bin/env python3.10

from typing import List, Union, Optional, Sequence
from dataclasses import dataclass

import numpy
from pxr import Usd, UsdGeom, Gf, UsdPhysics, Sdf
from mujoco import mjtJoint
from enum import Enum

from ..utils import modify_name, xform_cache


class JointType(Enum):
    NONE = 0
    FIXED = 1
    REVOLUTE = 2
    CONTINUOUS = 3
    PRISMATIC = 4
    SPHERICAL = 5

    @classmethod
    def from_string(cls, type_str: str) -> "JointType":
        if type_str == "fixed":
            return JointType.FIXED
        elif type_str in ["revolute", "mjJNT_HINGE"]:
            return JointType.REVOLUTE
        elif type_str == "continuous":
            return JointType.CONTINUOUS
        elif type_str in ["prismatic", "mjJNT_SLIDE"]:
            return JointType.PRISMATIC
        else:
            print(f"Joint type {type_str} not supported.")
            return JointType.NONE

    @classmethod
    def from_mujoco(cls, jnt_type: mjtJoint) -> "JointType":
        if jnt_type == mjtJoint.mjJNT_FREE:
            return JointType.NONE
        elif jnt_type == mjtJoint.mjJNT_HINGE:
            return JointType.REVOLUTE
        elif jnt_type == mjtJoint.mjJNT_SLIDE:
            return JointType.PRISMATIC
        elif jnt_type == mjtJoint.mjJNT_BALL:
            return JointType.SPHERICAL
        else:
            print(f"Joint type {jnt_type} not supported.")
            return JointType.NONE


@dataclass(init=False)
class JointProperty:
    stage: Usd.Stage
    name: str
    path: str
    pos: Gf.Vec3d
    quat: Gf.Quatd
    axis: str
    type: JointType
    parent_prim: UsdGeom.Xform
    child_prim: UsdGeom.Xform

    def __init__(
            self,
            joint_name: str,
            joint_parent_prim: UsdGeom.Xform,
            joint_child_prim: UsdGeom.Xform,
            joint_pos: Sequence = numpy.array([0.0, 0.0, 0.0]),
            joint_quat: Optional[Sequence] = None,
            joint_axis: Union[str, List[float]] = "Z",
            joint_type: JointType = JointType.REVOLUTE,
    ) -> None:
        self.name = joint_name
        self.pos = joint_pos
        self.quat = joint_quat
        self.axis = joint_axis
        self.type = joint_type
        if joint_parent_prim.GetStage() != joint_child_prim.GetStage():
            raise ValueError(f"Parent prim {self.parent_prim.GetPath()} and child prim {self.child_prim.GetPath()} "
                             f"are not in the same stage.")
        self.parent_prim = joint_parent_prim
        self.child_prim = joint_child_prim

    @property
    def stage(self) -> Usd.Stage:
        return self.parent_prim.GetStage()

    @property
    def name(self) -> str:
        return self._name

    @name.setter
    def name(self, name: str) -> None:
        self._name = modify_name(in_name=name)

    @property
    def path(self) -> str:
        return self.parent_prim.GetPath().AppendChild(self.name)

    @property
    def pos(self) -> Gf.Vec3d:
        return self._pos

    @pos.setter
    def pos(self, pos: Sequence) -> None:
        self._pos = Gf.Vec3d(*pos)

    @property
    def quat(self) -> Gf.Quatd:
        if self.type == JointType.FIXED:
            return Gf.Quatd(1, 0, 0, 0)
        if self.axis == "X":
            return Gf.Quatd(0.7071068, 0, 0.7071068, 0)
        elif self.axis == "Y":
            return Gf.Quatd(0.7071068, -0.7071068, 0, 0)
        elif self.axis == "Z":
            return Gf.Quatd(1, 0, 0, 0)
        elif self.axis == "-X":
            return Gf.Quatd(0.7071068, 0, -0.7071068, 0)
        elif self.axis == "-Y":
            return Gf.Quatd(0.7071068, 0.7071068, 0, 0)
        elif self.axis == "-Z":
            return Gf.Quatd(0, 0, 1, 0)
        else:
            raise ValueError(f"Axis {self.axis} not supported.")

    @quat.setter
    def quat(self, quat: Sequence) -> None:
        # TODO: Convert quat to axis, then set axis to Z again
        if quat is not None:
            raise NotImplementedError("Quat not supported yet.")
        # quat = numpy.array(quat)
        # magnitude = numpy.linalg.norm(quat)
        # if not numpy.isclose(magnitude, 1.0):
        #     raise ValueError(f"Quat {quat} is not normalized.")
        # self._quat = Gf.Quatd(*quat)

    @property
    def axis(self) -> str:
        return self._axis

    @axis.setter
    def axis(self, axis: Union[str, Sequence]) -> None:
        if isinstance(axis, str):
            if axis.upper() in ["X", "Y", "Z", "-X", "-Y", "-Z"]:
                self._axis = axis.upper()
            else:
                raise ValueError(f"Axis {axis} not supported.")
        else:
            try:
                axis = numpy.array(axis)
            except ValueError:
                raise NotImplementedError(f"Axis {axis} type {type(axis)} not supported.")
            if numpy.allclose(axis, [1, 0, 0]):
                self._axis = "X"
            elif numpy.allclose(axis, [0, 1, 0]):
                self._axis = "Y"
            elif numpy.allclose(axis, [0, 0, 1]):
                self._axis = "Z"
            elif numpy.allclose(axis, [-1, 0, 0]):
                self._axis = "-X"
            elif numpy.allclose(axis, [0, -1, 0]):
                self._axis = "-Y"
            elif numpy.allclose(axis, [0, 0, -1]):
                self._axis = "-Z"
            else:
                raise ValueError(f"Axis {axis} not supported.")

    @property
    def type(self) -> JointType:
        return self._type

    @type.setter
    def type(self, value):
        self._type = value

    @property
    def parent_prim(self) -> UsdGeom.Xform:
        return self._parent_prim

    @parent_prim.setter
    def parent_prim(self, value):
        self._parent_prim = value

    @property
    def child_prim(self) -> UsdGeom.Xform:
        return self._child_prim

    @child_prim.setter
    def child_prim(self, value):
        self._child_prim = value


class JointBuilder:
    joint: UsdPhysics.Joint
    joint_property: JointProperty

    def __init__(
            self,
            joint_property: JointProperty
    ) -> None:
        self._joint_property = joint_property
        self._joint = self._create_joint()

    def build(self) -> UsdPhysics.Joint:
        self._joint = self._create_joint()
        self._joint.CreateCollisionEnabledAttr(False)

        self._joint.GetBody0Rel().SetTargets([self.parent_prim.GetPath()])
        self._joint.GetBody1Rel().SetTargets([self.child_prim.GetPath()])

        body1_transform = xform_cache.GetLocalToWorldTransform(self.parent_prim)
        body1_rot = body1_transform.ExtractRotationQuat()

        body2_transform = xform_cache.GetLocalToWorldTransform(self.child_prim)
        body1_to_body2_transform = body2_transform * body1_transform.GetInverse()
        body1_to_body2_pos = body1_to_body2_transform.ExtractTranslation()
        body1_to_body2_rot = body1_to_body2_transform.ExtractRotationQuat()

        self._joint.CreateLocalPos0Attr(body1_rot.Transform(self.pos) + body1_to_body2_pos)
        self._joint.CreateLocalPos1Attr(Gf.Vec3d())

        self._joint.CreateLocalRot0Attr(Gf.Quatf(body1_to_body2_rot * self.quat))
        self._joint.CreateLocalRot1Attr(Gf.Quatf(self.quat))

        return self._joint

    def _create_joint(self) -> UsdPhysics.Joint:
        if self.type == JointType.FIXED:
            return UsdPhysics.FixedJoint.Define(self.stage, self.path)
        elif self.type == JointType.REVOLUTE or self.type == JointType.CONTINUOUS:
            return UsdPhysics.RevoluteJoint.Define(self.stage, self.path)
        elif self.type == JointType.PRISMATIC:
            return UsdPhysics.PrismaticJoint.Define(self.stage, self.path)
        elif self.type == JointType.SPHERICAL:
            return UsdPhysics.SphericalJoint.Define(self.stage, self.path)
        else:
            print(f"Joint type {str(self.type)} not supported, using default joint.")
            return UsdPhysics.Joint.Define(self.stage, self.path)

    def set_limit(self, lower: float = None, upper: float = None) -> None:
        if lower is not None and upper is not None and lower > upper:
            raise ValueError(
                f"[Joint {self._joint.GetName()}] Lower limit {lower} is greater than upper limit {upper}.")

        if self.type == JointType.REVOLUTE or self.type == JointType.PRISMATIC:
            if lower is not None:
                UsdPhysics.RevoluteJoint(self.joint).CreateLowerLimitAttr(lower)
            if upper is not None:
                UsdPhysics.RevoluteJoint(self.joint).CreateUpperLimitAttr(upper)
        elif self.type == JointType.PRISMATIC:
            if lower is not None:
                UsdPhysics.PrismaticJoint(self.joint).CreateLowerLimitAttr(lower)
            if upper is not None:
                UsdPhysics.PrismaticJoint(self.joint).CreateUpperLimitAttr(upper)
        else:
            print(f"[Joint {self.joint.GetName()}] Joint type {str(self.type)} does not have limits.")

    @property
    def joint(self) -> UsdPhysics.Joint:
        return self._joint

    @property
    def stage(self) -> Usd.Stage:
        return self._joint_property.stage

    @property
    def path(self) -> Sdf.Path:
        return self._joint_property.path

    @property
    def pos(self) -> Gf.Vec3d:
        return self._joint_property.pos

    @property
    def quat(self) -> Gf.Quatd:
        return self._joint_property.quat

    @property
    def axis(self) -> str:
        return self._joint_property.axis

    @property
    def type(self) -> JointType:
        return self._joint_property.type

    @property
    def parent_prim(self) -> UsdGeom.Xform:
        return self._joint_property.parent_prim

    @property
    def child_prim(self) -> UsdGeom.Xform:
        return self._joint_property.child_prim
