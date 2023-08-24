#!/usr/bin/env python3.10

from pxr import Usd, UsdGeom, Gf, UsdPhysics
from enum import Enum

from multiverse_parser.utils import xform_cache

joint_dict = {}


class JointType(Enum):
    NONE = 0
    FIXED = 1
    REVOLUTE = 2
    CONTINUOUS = 3
    PRISMATIC = 4
    SPHERICAL = 5


class JointBuilder:
    def __init__(
        self,
        stage: Usd.Stage,
        name: str,
        parent_xform: UsdGeom.Xform,
        child_xform: UsdGeom.Xform,
        joint_type: JointType,
        pos: tuple = (0.0, 0.0, 0.0),
        quat: tuple = None,
        axis: tuple = "Z",
    ) -> None:
        joint_dict[name] = self
        self.stage = stage
        self.parent_xform = parent_xform
        self.child_xform = child_xform
        self.path = self.parent_xform.GetPath().AppendPath(name)
        self.type = joint_type
        self.pos = Gf.Vec3d(pos)
        self.axis = axis
        self.set_joint()
        if quat is None:
            self.set_axis()
        elif self.axis == "Z":
            self.quat = quat
        else:
            pass  # TODO: Convert quat to axis, then set axis to Z again

        self.joint.CreateCollisionEnabledAttr(False)

        self.joint.GetBody0Rel().SetTargets([self.parent_xform.GetPath()])
        self.joint.GetBody1Rel().SetTargets([self.child_xform.GetPath()])

        body1_transform = xform_cache.GetLocalToWorldTransform(self.parent_xform.GetPrim())
        body1_rot = body1_transform.ExtractRotationQuat()

        body2_transform = xform_cache.GetLocalToWorldTransform(self.child_xform.GetPrim())
        body1_to_body2_transform = body2_transform * body1_transform.GetInverse()
        body1_to_body2_pos = body1_to_body2_transform.ExtractTranslation()
        body1_to_body2_rot = body1_to_body2_transform.ExtractRotationQuat()

        self.joint.CreateLocalPos0Attr(body1_rot.Transform(self.pos) + body1_to_body2_pos)
        self.joint.CreateLocalPos1Attr(Gf.Vec3d())

        self.joint.CreateLocalRot0Attr(Gf.Quatf(body1_to_body2_rot * self.quat))
        self.joint.CreateLocalRot1Attr(Gf.Quatf(self.quat))

    def set_joint(self) -> None:
        if self.type == JointType.FIXED:
            self.joint = UsdPhysics.FixedJoint.Define(self.stage, self.path)
        elif self.type == JointType.REVOLUTE or self.type == JointType.CONTINUOUS:
            self.joint = UsdPhysics.RevoluteJoint.Define(self.stage, self.path)
        elif self.type == JointType.PRISMATIC:
            self.joint = UsdPhysics.PrismaticJoint.Define(self.stage, self.path)
        elif self.type == JointType.SPHERICAL:
            self.joint = UsdPhysics.SphericalJoint.Define(self.stage, self.path)

    def set_limit(self, lower: float = None, upper: float = None) -> None:
        if self.type == JointType.REVOLUTE or self.type == JointType.PRISMATIC:
            if lower is not None:
                self.joint.CreateLowerLimitAttr(lower)
            if upper is not None:
                self.joint.CreateUpperLimitAttr(upper)
        else:
            print(f"Joint type {str(self.type)} does not have limits.")

    def set_axis(self) -> None:
        self.joint.CreateAxisAttr("Z")
        if self.axis == "X":
            self.quat = Gf.Quatd(0.7071068, 0, 0.7071068, 0)
        elif self.axis == "Y":
            self.quat = Gf.Quatd(0.7071068, -0.7071068, 0, 0)
        elif self.axis == "Z":
            self.quat = Gf.Quatd(1, 0, 0, 0)
        elif self.axis == "-X":
            self.quat = Gf.Quatd(0.7071068, 0, -0.7071068, 0)
        elif self.axis == "-Y":
            self.quat = Gf.Quatd(0.7071068, 0.7071068, 0, 0)
        elif self.axis == "-Z":
            self.quat = Gf.Quatd(0, 0, 1, 0)
