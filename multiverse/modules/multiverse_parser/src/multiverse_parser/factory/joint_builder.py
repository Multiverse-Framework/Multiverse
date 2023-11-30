#!/usr/bin/env python3.10

from typing import List, Union

import numpy
from pxr import Usd, UsdGeom, Gf, UsdPhysics
from mujoco import mjtJoint
from enum import Enum

from ..utils import xform_cache


def get_joint_axis(axis: List[float]) -> str | None:
    if numpy.array_equal(axis, [1, 0, 0]):
        return "X"
    if numpy.array_equal(axis, [0, 1, 0]):
        return "Y"
    if numpy.array_equal(axis, [0, 0, 1]):
        return "Z"
    if numpy.array_equal(axis, [-1, 0, 0]):
        return "-X"
    if numpy.array_equal(axis, [0, -1, 0]):
        return "-Y"
    if numpy.array_equal(axis, [0, 0, -1]):
        return "-Z"
    return None


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


class JointBuilder:
    stage: Usd.Stage
    parent_prim: UsdGeom.Xform
    child_prim: UsdGeom.Xform
    path: str
    type: JointType
    pos: Gf.Vec3d
    quat: Gf.Quatd
    axis: str
    joint: UsdPhysics.Joint

    def __init__(
            self,
            stage: Usd.Stage,
            name: str,
            parent_prim: UsdGeom.Xform,
            child_prim: UsdGeom.Xform,
            joint_type: JointType,
            joint_pos: tuple = (0.0, 0.0, 0.0),
            joint_quat: tuple = None,
            joint_axis: Union[str, List[float]] = "Z",
    ) -> None:
        self.stage = stage
        self.parent_prim = parent_prim
        self.child_prim = child_prim
        self.path = self.parent_prim.GetPath().AppendPath(name)
        self.type = joint_type
        self.pos = Gf.Vec3d(joint_pos)
        self.axis = joint_axis if isinstance(joint_axis, str) else get_joint_axis(joint_axis)
        if joint_quat is not None:
            # TODO: Convert quat to axis, then set axis to Z again
            raise NotImplementedError(f"[Joint {name}] Joint quat {joint_quat} not supported yet.")

    def build(self) -> UsdPhysics.Joint:
        joint = self._create_joint()
        joint.CreateCollisionEnabledAttr(False)

        joint.GetBody0Rel().SetTargets([self.parent_prim.GetPath()])
        joint.GetBody1Rel().SetTargets([self.child_prim.GetPath()])

        body1_transform = xform_cache.GetLocalToWorldTransform(self.parent_prim)
        body1_rot = body1_transform.ExtractRotationQuat()

        body2_transform = xform_cache.GetLocalToWorldTransform(self.child_prim)
        body1_to_body2_transform = body2_transform * body1_transform.GetInverse()
        body1_to_body2_pos = body1_to_body2_transform.ExtractTranslation()
        body1_to_body2_rot = body1_to_body2_transform.ExtractRotationQuat()

        joint.CreateLocalPos0Attr(body1_rot.Transform(self.pos) + body1_to_body2_pos)
        joint.CreateLocalPos1Attr(Gf.Vec3d())

        joint.CreateLocalRot0Attr(Gf.Quatf(body1_to_body2_rot * self.quat))
        joint.CreateLocalRot1Attr(Gf.Quatf(self.quat))

        return joint

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
        joint = self.stage.GetPrimAtPath(self.path)

        if lower is not None and upper is not None and lower > upper:
            raise ValueError(
                f"[Joint {self.joint.GetName()}] Lower limit {lower} is greater than upper limit {upper}.")

        if self.type == JointType.REVOLUTE or self.type == JointType.PRISMATIC:
            if lower is not None:
                UsdPhysics.RevoluteJoint(joint).CreateLowerLimitAttr(lower)
            if upper is not None:
                UsdPhysics.RevoluteJoint(joint).CreateUpperLimitAttr(upper)
        elif self.type == JointType.PRISMATIC:
            if lower is not None:
                UsdPhysics.PrismaticJoint(joint).CreateLowerLimitAttr(lower)
            if upper is not None:
                UsdPhysics.PrismaticJoint(joint).CreateUpperLimitAttr(upper)
        else:
            print(f"[Joint {joint.GetName()}] Joint type {str(self.type)} does not have limits.")

    @property
    def quat(self) -> Gf.Quatd | None:
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
            joint = self.stage.GetPrimAtPath(self.path)
            raise ValueError(f"[Joint {joint.GetName()}] Axis {self.axis} not supported.")
