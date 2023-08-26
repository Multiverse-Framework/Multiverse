#!/usr/bin/env python3.10

import os
import numpy
from math import radians
from urdf_parser_py import urdf
from pxr import UsdPhysics
from multiverse_parser import WorldBuilder, GeomType, JointType
from multiverse_parser.factory.body_builder import body_dict
from multiverse_parser.factory.joint_builder import joint_dict
from multiverse_parser.factory.geom_builder import geom_dict
from multiverse_parser.factory.mesh_builder import VisualMeshBuilder, CollisionMeshBuilder
from multiverse_parser.utils import *


class UrdfExporter:
    def __init__(
        self,
        urdf_file_path: str,
        world_builder: WorldBuilder,
        with_physics: bool,
        with_visual: bool,
        with_collision: bool,
    ) -> None:
        self.urdf_file_path = urdf_file_path
        self.world_builder = world_builder
        self.with_physics = with_physics
        self.with_visual = with_visual
        self.with_collision = with_collision

        self.robot = urdf.URDF(self.world_builder.stage.GetDefaultPrim().GetName())

        tmp_urdf_file_path = self.urdf_file_path
        mesh_path = os.path.splitext(os.path.basename(tmp_urdf_file_path))[0]
        while tmp_urdf_file_path != "/":
            tmp_urdf_file_path = os.path.dirname(tmp_urdf_file_path)
            mesh_path = os.path.join(os.path.basename(tmp_urdf_file_path), mesh_path)

            if os.path.exists(os.path.join(tmp_urdf_file_path, "package.xml")):
                break
        else:
            print(f"No ROS package found in {self.urdf_file_path}.")
            return

        self.urdf_mesh_dir_abs = os.path.join(os.path.dirname(tmp_urdf_file_path), mesh_path)
        self.urdf_mesh_dir_ros = "package://" + mesh_path

        for body_name in self.world_builder.body_names:
            if not with_physics or not self.build_joints(body_name=body_name):
                self.build_fixed_joint(child_link_name=body_name)
            self.build_link(body_name=body_name)

        self.export()

    def build_fixed_joint(self, child_link_name: str) -> None:
        body_builder = body_dict[child_link_name]
        parent_link_name = body_builder.xform.GetPrim().GetParent().GetName()
        if parent_link_name in body_dict:
            transformation = body_builder.xform.GetLocalTransformation()
            xyz = transformation.ExtractTranslation()
            quat = transformation.ExtractRotationQuat()
            rpy = quat_to_rpy(quat)

            joint = urdf.Joint(name=child_link_name + "_joint")
            joint.origin = urdf.Pose(xyz=xyz, rpy=rpy)
            joint.type = "fixed"
            joint.parent = parent_link_name
            joint.child = child_link_name
            self.robot.add_joint(joint)

    def build_joints(self, body_name: str) -> bool:
        body_builder = body_dict[body_name]

        for joint_name in body_builder.joint_names:
            joint_builder = joint_dict[joint_name]
            if joint_builder.type == JointType.NONE or joint_builder.type == JointType.FIXED or joint_builder.type == JointType.SPHERICAL:
                continue

            joint = urdf.Joint(name=joint_name)

            if joint_builder.type == JointType.PRISMATIC or joint_builder.type == JointType.REVOLUTE:
                limit = urdf.JointLimit()
                limit.effort = 1000
                limit.velocity = 1000

                if joint_builder.type == JointType.REVOLUTE:
                    limit.lower = radians(joint_builder.joint.GetLowerLimitAttr().Get())
                    limit.upper = radians(joint_builder.joint.GetUpperLimitAttr().Get())
                else:
                    limit.lower = joint_builder.joint.GetLowerLimitAttr().Get()
                    limit.upper = joint_builder.joint.GetUpperLimitAttr().Get()

                if joint_builder.type == JointType.PRISMATIC:
                    joint.type = "prismatic"

                elif joint_builder.type == JointType.REVOLUTE:
                    joint.type = "revolute"

                joint.limit = limit

            elif joint_builder.type == JointType.CONTINUOUS:
                joint.type = "continuous"

            if joint_builder.axis == "X":
                joint.axis = (1, 0, 0)
            elif joint_builder.axis == "Y":
                joint.axis = (0, 1, 0)
            elif joint_builder.axis == "Z":
                joint.axis = (0, 0, 1)
            elif joint_builder.axis == "-X":
                joint.axis = (-1, 0, 0)
            elif joint_builder.axis == "-Y":
                joint.axis = (0, -1, 0)
            elif joint_builder.axis == "-Z":
                joint.axis = (0, 0, -1)

            joint.parent = joint_builder.parent_xform.GetPrim().GetName()
            joint.child = joint_builder.child_xform.GetPrim().GetName()

            xyz = joint_builder.joint.GetLocalPos0Attr().Get()
            quat = joint_builder.joint.GetLocalRot0Attr().Get() * joint_builder.joint.GetLocalRot1Attr().Get().GetInverse()
            rpy = quat_to_rpy(quat)
            joint.origin = urdf.Pose(xyz=xyz, rpy=rpy)

            self.robot.add_joint(joint)

        return len(body_builder.joint_names) > 0

    def build_link(self, body_name: str) -> None:
        body_builder = body_dict[body_name]

        link = urdf.Link(name=body_name)

        if self.with_physics and body_builder.xform.GetPrim().HasAPI(UsdPhysics.MassAPI):
            physics_mass_api = UsdPhysics.MassAPI(body_builder.xform)
            mass = physics_mass_api.GetMassAttr().Get()
            xyz = physics_mass_api.GetCenterOfMassAttr().Get()
            rpy = quat_to_rpy(physics_mass_api.GetPrincipalAxesAttr().Get())
            origin = urdf.Pose(xyz=xyz, rpy=rpy)
            diagonal_inertia = physics_mass_api.GetDiagonalInertiaAttr().Get()
            inertia = urdf.Inertia(
                ixx=diagonal_inertia[0],
                iyy=diagonal_inertia[1],
                izz=diagonal_inertia[2],
            )
            link.inertial = urdf.Inertial(mass=mass, inertia=inertia, origin=origin)

        for geom_name in body_builder.geom_names:
            geom_builder = geom_dict[geom_name]

            geometry = None

            if geom_builder.type == GeomType.CUBE:
                geometry = urdf.Box(size=numpy.array([geom_builder.xform.GetLocalTransformation().GetRow(i).GetLength() for i in range(3)]) * 2)
            elif geom_builder.type == GeomType.SPHERE:
                geometry = urdf.Sphere(radius=geom_builder.geom.GetRadiusAttr().Get())
            elif geom_builder.type == GeomType.CYLINDER:
                geometry = urdf.Cylinder(
                    radius=geom_builder.geom.GetRadiusAttr().Get(),
                    length=geom_builder.geom.GetHeightAttr().Get(),
                )

            is_visual = not geom_builder.geom.GetPrim().HasAPI(UsdPhysics.CollisionAPI)

            transformation = geom_builder.xform.GetLocalTransformation()
            xyz = transformation.ExtractTranslation()
            quat = transformation.ExtractRotationQuat()
            rpy = quat_to_rpy(quat)
            origin = urdf.Pose(xyz, rpy)

            if geometry is not None:
                if self.with_visual and is_visual:
                    visual = urdf.Visual(
                        geometry=geometry,
                        material=None,
                        origin=origin,
                        name=geom_builder.name,
                    )
                    link.visual = visual

                if self.with_collision and not is_visual:
                    collision = urdf.Collision(geometry=geometry, origin=origin, name=geom_builder.name)
                    link.collision = collision

            if geom_builder.mesh_builder is not None:
                mesh_builder = geom_builder.mesh_builder
                clear_meshes()

                import_usd(mesh_builder.usd_file_path)

                # transform(xyz=xyz, rpy=rpy)

                if self.with_visual and is_visual:
                    mesh_rel_path = os.path.join(
                        "obj",
                        os.path.splitext(os.path.basename(mesh_builder.usd_file_path))[0] + ".obj",
                    )
                    export_obj(os.path.join(self.urdf_mesh_dir_abs, mesh_rel_path))
                    filename = os.path.join(self.urdf_mesh_dir_ros, mesh_rel_path)
                    geometry = urdf.Mesh(filename=filename, scale=rotate_vector_by_quat(vector=geom_builder.scale, quat=quat))
                    
                    visual = urdf.Visual(
                        geometry=geometry,
                        material=None,
                        origin=origin,
                        name=mesh_builder.name,
                    )
                    link.visual = visual

                if self.with_collision and not is_visual:
                    mesh_rel_path = os.path.join(
                        "stl",
                        os.path.splitext(os.path.basename(mesh_builder.usd_file_path))[0] + ".stl",
                    )

                    export_stl(os.path.join(self.urdf_mesh_dir_abs, mesh_rel_path))
                    filename = os.path.join(self.urdf_mesh_dir_ros, mesh_rel_path)
                    geometry = urdf.Mesh(filename=filename, scale=rotate_vector_by_quat(vector=geom_builder.scale, quat=quat))

                    collision = urdf.Collision(
                        geometry=geometry,
                        origin=origin,
                        name=mesh_builder.name,
                    )
                    link.collision = collision

        self.robot.add_link(link)

    def export(self):
        os.makedirs(name=os.path.dirname(self.urdf_file_path), exist_ok=True)

        xml_string = self.robot.to_xml_string()

        with open(self.urdf_file_path, "w") as file:
            file.write(xml_string)
