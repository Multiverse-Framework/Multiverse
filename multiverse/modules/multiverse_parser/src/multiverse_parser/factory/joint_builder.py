#!/usr/bin/env python3

from typing import List, Union, Optional, Sequence
from dataclasses import dataclass
from enum import Enum

import numpy
from mujoco import mjtJoint

from ..utils import modify_name, xform_cache

from pxr import Usd, UsdGeom, Gf, UsdPhysics, Sdf


class JointType(Enum):
    FREE = 0
    FIXED = 1
    REVOLUTE = 2
    CONTINUOUS = 3
    PRISMATIC = 4
    SPHERICAL = 5
    PLANAR = 6

    @classmethod
    def from_urdf(cls, type_str: str) -> "JointType":
        if type_str == "floating":
            return JointType.FREE
        if type_str == "fixed":
            return JointType.FIXED
        elif type_str == "revolute":
            return JointType.REVOLUTE
        elif type_str == "continuous":
            return JointType.CONTINUOUS
        elif type_str == "prismatic":
            return JointType.PRISMATIC
        elif type_str == "planar":
            return JointType.PLANAR
        else:
            raise ValueError(f"Joint type {type_str} not supported.")

    def to_urdf(self) -> str:
        if self == JointType.FREE:
            return "floating"
        elif self == JointType.FIXED:
            return "fixed"
        elif self == JointType.REVOLUTE:
            return "revolute"
        elif self == JointType.CONTINUOUS:
            return "continuous"
        elif self == JointType.PRISMATIC:
            return "prismatic"
        elif self == JointType.PLANAR:
            return "planar"
        else:
            raise ValueError(f"Joint type {self} not supported.")

    @classmethod
    def from_mujoco(cls, jnt_type: mjtJoint) -> "JointType":
        if jnt_type == mjtJoint.mjJNT_FREE:
            return JointType.FREE
        elif jnt_type == mjtJoint.mjJNT_HINGE:
            return JointType.REVOLUTE
        elif jnt_type == mjtJoint.mjJNT_SLIDE:
            return JointType.PRISMATIC
        elif jnt_type == mjtJoint.mjJNT_BALL:
            return JointType.SPHERICAL
        else:
            raise ValueError(f"Joint type {jnt_type} not supported.")

    def to_mujoco(self) -> mjtJoint:
        if self == JointType.FREE:
            return mjtJoint.mjJNT_FREE
        elif self == JointType.REVOLUTE or self == JointType.CONTINUOUS:
            return mjtJoint.mjJNT_HINGE
        elif self == JointType.PRISMATIC:
            return mjtJoint.mjJNT_SLIDE
        elif self == JointType.SPHERICAL:
            return mjtJoint.mjJNT_BALL
        else:
            raise ValueError(f"Joint type {self} not supported.")


class JointAxis(Enum):
    X = 0
    Y = 1
    Z = 2
    NEG_X = 3
    NEG_Y = 4
    NEG_Z = 5

    @classmethod
    def from_string(cls, axis_str: str) -> Optional["JointAxis"]:
        if axis_str == "X":
            return JointAxis.X
        elif axis_str == "Y":
            return JointAxis.Y
        elif axis_str == "Z":
            return JointAxis.Z
        elif axis_str == "-X":
            return JointAxis.NEG_X
        elif axis_str == "-Y":
            return JointAxis.NEG_Y
        elif axis_str == "-Z":
            return JointAxis.NEG_Z
        else:
            return None

    @classmethod
    def to_string(cls) -> str:
        if cls == JointAxis.X:
            return "X"
        elif cls == JointAxis.Y:
            return "Y"
        elif cls == JointAxis.Z:
            return "Z"
        elif cls == JointAxis.NEG_X:
            return "-X"
        elif cls == JointAxis.NEG_Y:
            return "-Y"
        elif cls == JointAxis.NEG_Z:
            return "-Z"
        else:
            raise ValueError(f"Joint axis {cls} not supported.")

    @classmethod
    def from_array(cls, joint_axis: numpy.ndarray) -> "JointAxis":
        if numpy.allclose(joint_axis, [1, 0, 0]):
            return JointAxis.X
        elif numpy.allclose(joint_axis, [0, 1, 0]):
            return JointAxis.Y
        elif numpy.allclose(joint_axis, [0, 0, 1]):
            return JointAxis.Z
        elif numpy.allclose(joint_axis, [-1, 0, 0]):
            return JointAxis.NEG_X
        elif numpy.allclose(joint_axis, [0, -1, 0]):
            return JointAxis.NEG_Y
        elif numpy.allclose(joint_axis, [0, 0, -1]):
            return JointAxis.NEG_Z
        else:
            return None

    def to_array(self) -> numpy.ndarray:
        if self == JointAxis.X:
            return numpy.array([1.0, 0.0, 0.0])
        elif self == JointAxis.Y:
            return numpy.array([0.0, 1.0, 0.0])
        elif self == JointAxis.Z:
            return numpy.array([0.0, 0.0, 1.0])
        elif self == JointAxis.NEG_X:
            return numpy.array([-1.0, 0.0, 0.0])
        elif self == JointAxis.NEG_Y:
            return numpy.array([0.0, -1.0, 0.0])
        elif self == JointAxis.NEG_Z:
            return numpy.array([0.0, 0.0, -1.0])
        else:
            raise ValueError(f"Joint axis {self} not supported.")

    @classmethod
    def from_quat(cls, quat: numpy.ndarray) -> "JointAxis":
        if numpy.allclose(quat, [0.0, 0.7071068, 0.0, 0.7071068]):
            return JointAxis.X
        elif numpy.allclose(quat, [-0.7071068, 0.0, 0.0, 0.7071068]):
            return JointAxis.Y
        elif numpy.allclose(quat, [0.0, 0.0, 0.0, 1.0]):
            return JointAxis.Z
        elif numpy.allclose(quat, [0.0, -0.7071068, 0.0, 0.7071068]):
            return JointAxis.NEG_X
        elif numpy.allclose(quat, [0.7071068, 0.0, 0.0, 0.7071068]):
            return JointAxis.NEG_Y
        elif numpy.allclose(quat, [0.0, 1.0, 0.0, 0.0]):
            return JointAxis.NEG_Z
        else:
            raise ValueError(f"Joint axis {quat} not supported.")

    def to_quat(self) -> numpy.ndarray:
        if self == JointAxis.X:
            return numpy.array([0.0, 0.7071068, 0.0, 0.7071068])
        elif self == JointAxis.Y:
            return numpy.array([-0.7071068, 0.0, 0.0, 0.7071068])
        elif self == JointAxis.Z:
            return numpy.array([0.0, 0.0, 0.0, 1.0])
        elif self == JointAxis.NEG_X:
            return numpy.array([0.0, -0.7071068, 0.0, 0.7071068])
        elif self == JointAxis.NEG_Y:
            return numpy.array([0.7071068, 0.0, 0.0, 0.7071068])
        elif self == JointAxis.NEG_Z:
            return numpy.array([0.0, 1.0, 0.0, 0.0])
        else:
            raise ValueError(f"Joint axis {self} not supported.")


@dataclass(init=False)
class JointProperty:
    stage: Usd.Stage
    def __init__(
            self,
            joint_name: str,
            joint_parent_prim: UsdGeom.Xform,
            joint_child_prim: UsdGeom.Xform,
            joint_pos: Sequence = numpy.array([0.0, 0.0, 0.0]),
            joint_quat: Optional[Sequence] = None,
            joint_axis: JointAxis = JointAxis.Z,
            joint_type: JointType = JointType.REVOLUTE,
    ) -> None:
        self.name = joint_name
        self.pos = joint_pos
        self.axis = joint_axis
        self.quat = joint_quat
        self.type = joint_type
        if joint_parent_prim.GetStage() != joint_child_prim.GetStage():
            raise ValueError(f"Parent prim {self.parent_prim.GetPath()} and child prim {self.child_prim.GetPath()} "
                             f"are not in the same stage.")
        self.parent_prim = joint_parent_prim
        self.child_prim = joint_child_prim
    name: str
    path: str
    pos: Gf.Vec3d
    quat: Gf.Quatd
    axis: JointAxis
    type: JointType
    parent_prim: UsdGeom.Xform

    child_prim: UsdGeom.Xform

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
        return Gf.Quatd(self._quat[3], Gf.Vec3d(*self._quat[:3]))

    @quat.setter
    def quat(self, quat: Sequence) -> None:
        if quat is not None:
            if self.axis == JointAxis.Z:
                self._quat = numpy.array([*quat])
            else:
                # TODO: Convert quat to axis, then set axis to Z again
                raise ValueError(f"Joint axis {self.axis} not supported.")
        else:
            self._quat = self.axis.to_quat()

    @property
    def axis(self) -> JointAxis:
        return self._axis

    @axis.setter
    def axis(self, axis: JointAxis) -> None:
        self._axis = axis

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

        if self.type == JointType.PRISMATIC or self.type == JointType.REVOLUTE or self.type == JointType.CONTINUOUS:
            self._joint.CreateAxisAttr("Z")

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
    def axis(self) -> JointAxis:
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
