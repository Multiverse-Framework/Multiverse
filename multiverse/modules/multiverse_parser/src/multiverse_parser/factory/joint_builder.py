#!/usr/bin/env python3.10

from pxr import Usd, Gf, UsdPhysics
from enum import Enum

joint_dict = {}


class JointType(Enum):
    FIXED = 0
    REVOLUTE = 1
    CONTINUOUS = 2
    PRISMATIC = 3
    SPHERICAL = 4


class JointBuilder:
    def __init__(
        self,
        stage: Usd.Stage,
        name: str,
        parent_body,
        child_body,
        joint_type: JointType,
        pos: tuple = (0.0, 0.0, 0.0),
        axis: str = "Z",
    ) -> None:
        joint_dict[name] = self
        self.stage = stage
        self.parent_body = parent_body
        self.child_body = child_body
        self.path = self.parent_body.prim.GetPath().AppendPath(name)
        self.type = joint_type
        self.set_prim()
        self.pos = Gf.Vec3f(pos)
        self.set_axis(axis)

        self.prim.CreateCollisionEnabledAttr(False)

        self.prim.GetBody0Rel().SetTargets([self.parent_body.prim.GetPath()])
        self.prim.GetBody1Rel().SetTargets([self.child_body.prim.GetPath()])

        body1_rot = Gf.Quatf(self.parent_body.quat)

        body2_pos = Gf.Vec3f(self.child_body.pos)
        body2_rot = Gf.Quatf(self.child_body.quat)

        self.prim.CreateLocalPos0Attr(body2_pos + body1_rot.Transform(self.pos))
        self.prim.CreateLocalPos1Attr(Gf.Vec3f())

        self.prim.CreateLocalRot0Attr(body2_rot * self.quat)
        self.prim.CreateLocalRot1Attr(self.quat)

    def set_prim(self) -> None:
        if self.type == JointType.FIXED:
            self.prim = UsdPhysics.FixedJoint.Define(self.stage, self.path)
        elif self.type == JointType.REVOLUTE or self.type == JointType.CONTINUOUS:
            self.prim = UsdPhysics.RevoluteJoint.Define(self.stage, self.path)
        elif self.type == JointType.PRISMATIC:
            self.prim = UsdPhysics.PrismaticJoint.Define(self.stage, self.path)
        elif self.type == JointType.SPHERICAL:
            self.prim = UsdPhysics.SphericalJoint.Define(self.stage, self.path)

    def set_limit(self, lower: float = None, upper: float = None) -> None:
        if self.type == JointType.REVOLUTE or self.type == JointType.PRISMATIC:
            if lower is not None:
                self.prim.CreateLowerLimitAttr(lower)
            if upper is not None:
                self.prim.CreateUpperLimitAttr(upper)
        else:
            print(f"Joint type {str(self.type)} does not have limits.")

    def set_axis(self, axis: str) -> None:
        self.prim.CreateAxisAttr("Z")
        if axis == "X":
            self.quat = Gf.Quatf(0.7071068, 0, 0.7071068, 0)
        elif axis == "Y":
            self.quat = Gf.Quatf(0.7071068, -0.7071068, 0, 0)
        elif axis == "Z":
            self.quat = Gf.Quatf(1, 0, 0, 0)
        elif axis == "-X":
            self.quat = Gf.Quatf(0.7071068, 0, -0.7071068, 0)
        elif axis == "-Y":
            self.quat = Gf.Quatf(0.7071068, 0.7071068, 0, 0)
        elif axis == "-Z":
            self.quat = Gf.Quatf(0, 0, 1, 0)
