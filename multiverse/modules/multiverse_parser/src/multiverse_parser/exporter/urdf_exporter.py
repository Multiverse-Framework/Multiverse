#!/usr/bin/env python3

import os
from typing import Tuple, Optional, Union

import numpy
from scipy.spatial.transform import Rotation
from urdf_parser_py import urdf
from pxr import UsdUrdf, Gf, UsdPhysics, UsdGeom, Usd

from ..factory import Factory
from ..factory import (WorldBuilder,
                       BodyBuilder,
                       JointBuilder, JointType,
                       GeomBuilder, GeomType)
from ..importer.urdf_importer import build_urdf_inertial_api
from ..utils import xform_cache


def get_meshdir_paths(file_path: str, relative_to_ros_package: bool) -> Tuple[str, str]:
    meshdir_name = os.path.splitext(os.path.basename(file_path))[0]
    if relative_to_ros_package:
        tmp_urdf_file_path = file_path
        mesh_path = meshdir_name
        while tmp_urdf_file_path != "/":
            tmp_urdf_file_path = os.path.dirname(tmp_urdf_file_path)
            mesh_path = os.path.join(os.path.basename(tmp_urdf_file_path), mesh_path)

            if os.path.exists(os.path.join(tmp_urdf_file_path, "package.xml")):
                break
        else:
            raise ValueError(f"Could not find package.xml in any parent directory of {file_path}.")

        meshdir_abs = str(os.path.join(os.path.dirname(tmp_urdf_file_path), mesh_path))
        meshdir_ros = "package://" + mesh_path
    else:
        meshdir_abs = str(os.path.join(os.path.dirname(file_path), meshdir_name))
        meshdir_ros = "file://" + meshdir_abs

    return meshdir_abs, meshdir_ros


def get_robot_name(world_builder: WorldBuilder) -> str:
    usd_urdf = UsdUrdf.Urdf.Get(world_builder.stage, "/urdf")
    if not usd_urdf.GetPrim().IsValid():
        usd_urdf = UsdUrdf.Urdf.Define(world_builder.stage, "/urdf")
        usd_urdf.CreateNameAttr(world_builder.stage.GetDefaultPrim().GetName())
    return usd_urdf.GetNameAttr().Get()


def get_urdf_joint_api(joint_builder: JointBuilder) -> UsdUrdf.UrdfJointAPI:
    joint = joint_builder.joint
    joint_prim = joint.GetPrim()
    if joint_prim.HasAPI(UsdUrdf.UrdfJointAPI):
        urdf_joint_api = UsdUrdf.UrdfJointAPI(joint_prim)
    else:
        urdf_joint_api = UsdUrdf.UrdfJointAPI.Apply(joint_prim)
        urdf_joint_api.CreateTypeAttr(joint_builder.type.to_urdf())
        urdf_joint_api.CreateParentRel().AddTarget(joint_builder.parent_prim.GetPath())
        urdf_joint_api.CreateChildRel().AddTarget(joint_builder.child_prim.GetPath())
        if joint_builder.type in [JointType.REVOLUTE, JointType.PRISMATIC]:
            joint_axis = joint_builder.quat.Transform(Gf.Vec3d([0.0, 0.0, 1.0]))
            urdf_joint_api.CreateAxisAttr(Gf.Vec3f(*joint_axis))
            effort = 1000  # TODO: Find a way to get this value
            velocity = 1000  # TODO: Find a way to get this value
            if joint_builder.type == JointType.REVOLUTE:
                usd_joint = UsdPhysics.RevoluteJoint(joint_prim)
                upper_limit = numpy.deg2rad(usd_joint.GetUpperLimitAttr().Get())
                lower_limit = numpy.deg2rad(usd_joint.GetLowerLimitAttr().Get())
            elif joint_builder.type == JointType.PRISMATIC:
                usd_joint = UsdPhysics.PrismaticJoint(joint_prim)
                upper_limit = usd_joint.GetUpperLimitAttr().Get()
                lower_limit = usd_joint.GetLowerLimitAttr().Get()
            else:
                raise ValueError(f"Joint type {joint_builder.type} not supported.")
            urdf_joint_api.CreateUpperAttr(upper_limit)
            urdf_joint_api.CreateLowerAttr(lower_limit)
            urdf_joint_api.CreateEffortAttr(effort)
            urdf_joint_api.CreateVelocityAttr(velocity)

        parent_transform = xform_cache.GetLocalToWorldTransform(joint_builder.parent_prim)
        child_transform = xform_cache.GetLocalToWorldTransform(joint_builder.child_prim)
        parent_to_child_transform = child_transform * parent_transform.GetInverse()
        xyz = parent_to_child_transform.ExtractTranslation()

        quat = joint.GetLocalRot0Attr().Get() * joint.GetLocalRot1Attr().Get().GetInverse()
        quat = numpy.array([*quat.GetImaginary(), quat.GetReal()])
        rpy = Rotation.from_quat(quat).as_euler("xyz", degrees=False)
        rpy = Gf.Vec3f(*rpy)
        urdf_joint_api.CreateXyzAttr(xyz)
        urdf_joint_api.CreateRpyAttr(rpy)

    return urdf_joint_api


def get_urdf_origin(xform: UsdGeom.Xform) -> Tuple[Gf.Vec3f, Gf.Vec3f]:
    transformation = xform.GetLocalTransformation().RemoveScaleShear()
    xyz = transformation.ExtractTranslation()
    quat = transformation.ExtractRotationQuat()
    quat = numpy.array([*quat.GetImaginary(), quat.GetReal()])
    rpy = Rotation.from_quat(quat).as_euler("xyz", degrees=False)
    rpy = Gf.Vec3f(*rpy)
    return xyz, rpy


def get_urdf_geometry_api(geom_prim: Usd.Prim) -> Union[UsdUrdf.UrdfLinkVisualAPI, UsdUrdf.UrdfLinkCollisionAPI]:
    xform_prim = geom_prim.GetPrim().GetParent()
    if xform_prim.HasAPI(UsdUrdf.UrdfLinkVisualAPI):
        urdf_geometry_api = UsdUrdf.UrdfLinkVisualAPI(xform_prim)
    elif xform_prim.HasAPI(UsdUrdf.UrdfLinkCollisionAPI):
        urdf_geometry_api = UsdUrdf.UrdfLinkCollisionAPI(xform_prim)
    else:
        xform = UsdGeom.Xform(xform_prim)
        xyz, rpy = get_urdf_origin(xform)

        if not geom_prim.GetPrim().HasAPI(UsdPhysics.CollisionAPI):
            urdf_geometry_api = UsdUrdf.UrdfLinkVisualAPI.Apply(xform_prim)
        else:
            urdf_geometry_api = UsdUrdf.UrdfLinkCollisionAPI.Apply(xform_prim)

        urdf_geometry_api.CreateXyzAttr(xyz)
        urdf_geometry_api.CreateRpyAttr(rpy)

    return urdf_geometry_api


def build_geom(geom_name: str,
               link: urdf.Link,
               geometry: Union[urdf.Box, urdf.Sphere, urdf.Cylinder, urdf.Mesh],
               urdf_geometry_api: Union[UsdUrdf.UrdfLinkVisualAPI, UsdUrdf.UrdfLinkCollisionAPI]):
    xyz = urdf_geometry_api.GetXyzAttr().Get()
    rpy = urdf_geometry_api.GetRpyAttr().Get()
    origin = urdf.Pose(xyz=xyz, rpy=rpy)
    if isinstance(urdf_geometry_api, UsdUrdf.UrdfLinkVisualAPI):
        visual = urdf.Visual(
            geometry=geometry,
            material=None,  # TODO: Add material
            origin=origin,
            name=geom_name,
        )
        link.visual = visual
    elif isinstance(urdf_geometry_api, UsdUrdf.UrdfLinkCollisionAPI):
        collision = urdf.Collision(
            geometry=geometry,
            origin=origin,
            name=geom_name)
        link.collision = collision
    else:
        raise ValueError(f"API {urdf_geometry_api} not supported.")


def get_urdf_inertial_api(xform_prim: Usd.Prim):
    if xform_prim.HasAPI(UsdUrdf.UrdfLinkInertialAPI):
        urdf_link_inertial_api = UsdUrdf.UrdfLinkInertialAPI(xform_prim)
    else:
        physics_mass_api = UsdPhysics.MassAPI(xform_prim)
        urdf_link_inertial_api = build_urdf_inertial_api(physics_mass_api=physics_mass_api)
    return urdf_link_inertial_api


def get_urdf_geometry_box_api(geom_builder: GeomBuilder) -> UsdUrdf.UrdfGeometryBoxAPI:
    xform_prim = geom_builder.xform.GetPrim()
    if xform_prim.HasAPI(UsdUrdf.UrdfGeometryBoxAPI):
        urdf_geometry_box_api = UsdUrdf.UrdfGeometryBoxAPI(xform_prim)
    else:
        urdf_geometry_box_api = UsdUrdf.UrdfGeometryBoxAPI.Apply(xform_prim)
        size = numpy.array([geom_builder.xform.GetLocalTransformation().GetRow(i).GetLength() for i in range(3)]) * 2
        size = Gf.Vec3f(*size)
        urdf_geometry_box_api.CreateSizeAttr(size)
    return urdf_geometry_box_api


def get_urdf_geometry_sphere_api(geom_builder: GeomBuilder) -> UsdUrdf.UrdfGeometrySphereAPI:
    xform_prim = geom_builder.xform.GetPrim()
    if xform_prim.HasAPI(UsdUrdf.UrdfGeometrySphereAPI):
        urdf_geometry_sphere_api = UsdUrdf.UrdfGeometrySphereAPI(xform_prim)
    else:
        urdf_geometry_sphere_api = UsdUrdf.UrdfGeometrySphereAPI.Apply(xform_prim)
        geom_sphere_prim = UsdGeom.Sphere(geom_builder.geom_prims[0])
        radius = geom_sphere_prim.GetRadiusAttr().Get()
        urdf_geometry_sphere_api.CreateRadiusAttr(radius)
    return urdf_geometry_sphere_api


def get_urdf_geometry_cylinder_api(geom_builder: GeomBuilder) -> UsdUrdf.UrdfGeometryCylinderAPI:
    xform_prim = geom_builder.xform.GetPrim()
    if xform_prim.HasAPI(UsdUrdf.UrdfGeometryCylinderAPI):
        urdf_geometry_cylinder_api = UsdUrdf.UrdfGeometryCylinderAPI(xform_prim)
    else:
        urdf_geometry_cylinder_api = UsdUrdf.UrdfGeometryCylinderAPI.Apply(xform_prim)
        geom_cylinder_prim = UsdGeom.Cylinder(geom_builder.geom_prims[0])
        radius = geom_cylinder_prim.GetRadiusAttr().Get()
        length = geom_cylinder_prim.GetHeightAttr().Get()
        urdf_geometry_cylinder_api.CreateRadiusAttr(radius)
        urdf_geometry_cylinder_api.CreateLengthAttr(length)
    return urdf_geometry_cylinder_api


def get_mesh_rel_path(geom_prim: UsdGeom.Gprim, usd_file_path: str) -> str:
    if not geom_prim.GetPrim().HasAPI(UsdPhysics.CollisionAPI):
        file_extension = "obj"
    else:
        file_extension = "stl"
    return os.path.join(file_extension, os.path.splitext(os.path.basename(usd_file_path))[0] + f".{file_extension}")


def get_urdf_geometry_mesh_api(geom_builder, mesh_abs_path) -> UsdUrdf.UrdfGeometryMeshAPI:
    xform_prim = geom_builder.xform.GetPrim()
    if not xform_prim.HasAPI(UsdUrdf.UrdfGeometryMeshAPI):
        urdf_geometry_mesh_api = UsdUrdf.UrdfGeometryMeshAPI.Apply(xform_prim)
        urdf_geometry_mesh_api.CreateFilenameAttr(mesh_abs_path)
        transformation = geom_builder.xform.GetLocalTransformation()

        # TODO: Doesn't work for negative scale
        scale = [transformation.GetRow(i).GetLength() for i in range(3)]
        scale = Gf.Vec3f(*scale)
        urdf_geometry_mesh_api.CreateScaleAttr(scale)
    else:
        urdf_geometry_mesh_api = UsdUrdf.UrdfGeometryMeshAPI(xform_prim)
    return urdf_geometry_mesh_api


class UrdfExporter:
    def __init__(
            self,
            factory: Factory,
            file_path: str,
            relative_to_ros_package: bool = False,
    ) -> None:
        self._factory = factory
        robot_name = get_robot_name(world_builder=factory.world_builder)
        self._robot = urdf.URDF(name=robot_name)
        self._meshdir_abs, self._meshdir_ros = get_meshdir_paths(file_path=file_path,
                                                                 relative_to_ros_package=relative_to_ros_package)
        self._file_path = file_path

    def build(self) -> None:
        for body_builder in self.factory.world_builder.body_builders:
            self._build_joints(body_builder=body_builder)
            self._build_link(body_builder=body_builder)

    def _build_joints(self, body_builder: BodyBuilder) -> None:
        for joint_builder in body_builder.joint_builders:
            urdf_joint_api = get_urdf_joint_api(joint_builder=joint_builder)

            joint_type = urdf_joint_api.GetTypeAttr().Get()
            if joint_type == "fixed" or not self.factory.config.with_physics:
                urdf_joint = self._build_fixed_joint(body_builder=body_builder, urdf_joint_api=urdf_joint_api)
            else:
                urdf_joint = self._build_movable_joint(urdf_joint_api=urdf_joint_api)
            if urdf_joint is not None:
                self.robot.add_joint(urdf_joint)

        if len(body_builder.joint_builders) == 0:
            urdf_joint = self._build_fixed_joint(body_builder=body_builder)
            if urdf_joint is not None:
                self.robot.add_joint(urdf_joint)

    def _build_fixed_joint(self,
                           body_builder: BodyBuilder,
                           urdf_joint_api: Optional[UsdUrdf.UrdfJointAPI] = None) -> Optional[urdf.Joint]:
        child_xform = body_builder.xform
        child_prim = child_xform.GetPrim()
        child_link_name = child_prim.GetName()
        if child_link_name == self.robot.name:
            return None

        parent_link_name = child_prim.GetParent().GetName()

        urdf_joint = urdf.Joint(name=child_link_name + "_joint")
        if urdf_joint_api is not None:
            xyz = urdf_joint_api.GetXyzAttr().Get()
            rpy = urdf_joint_api.GetRpyAttr().Get()
        else:
            xyz, rpy = get_urdf_origin(xform=child_xform)
        urdf_joint.origin = urdf.Pose(xyz=xyz, rpy=rpy)
        urdf_joint.type = "fixed"
        urdf_joint.parent = parent_link_name
        urdf_joint.child = child_link_name

        return urdf_joint

    def _build_movable_joint(self, urdf_joint_api: UsdUrdf.UrdfJointAPI) -> urdf.Joint:
        joint_type = urdf_joint_api.GetTypeAttr().Get()
        joint_prim = urdf_joint_api.GetPrim()
        joint_name = joint_prim.GetName()

        urdf_joint = urdf.Joint(name=joint_name)
        xyz = urdf_joint_api.GetXyzAttr().Get()
        rpy = urdf_joint_api.GetRpyAttr().Get()
        urdf_joint.origin = urdf.Pose(xyz=xyz, rpy=rpy)
        urdf_joint.type = joint_type
        stage = self.factory.world_builder.stage
        parent_prim_path = urdf_joint_api.GetParentRel().GetTargets()[0]
        parent_prim = stage.GetPrimAtPath(parent_prim_path)
        parent_name = parent_prim.GetName()
        urdf_joint.parent = parent_name
        child_prim_path = urdf_joint_api.GetChildRel().GetTargets()[0]
        child_prim = stage.GetPrimAtPath(child_prim_path)
        child_name = child_prim.GetName()
        urdf_joint.child = child_name
        if joint_type in ["revolute", "prismatic", "continuous"]:
            urdf_joint.axis = urdf_joint_api.GetAxisAttr().Get()
        if joint_type in ["revolute", "prismatic"]:
            urdf_joint.limit = urdf.JointLimit()
            urdf_joint.limit.lower = urdf_joint_api.GetLowerAttr().Get()
            urdf_joint.limit.upper = urdf_joint_api.GetUpperAttr().Get()
            urdf_joint.limit.effort = urdf_joint_api.GetEffortAttr().Get()
            urdf_joint.limit.velocity = urdf_joint_api.GetVelocityAttr().Get()

        return urdf_joint

    def _build_link(self, body_builder: BodyBuilder) -> None:
        xform_prim = body_builder.xform.GetPrim()
        link_name = xform_prim.GetName()
        link = urdf.Link(name=link_name)

        if self.factory.config.with_physics and xform_prim.HasAPI(UsdPhysics.MassAPI):
            urdf_link_inertial_api = get_urdf_inertial_api(xform_prim)

            xyz = urdf_link_inertial_api.GetXyzAttr().Get()
            rpy = urdf_link_inertial_api.GetRpyAttr().Get()
            origin = urdf.Pose(xyz=xyz, rpy=rpy)
            mass = urdf_link_inertial_api.GetMassAttr().Get()
            ixx = urdf_link_inertial_api.GetIxxAttr().Get()
            iyy = urdf_link_inertial_api.GetIyyAttr().Get()
            izz = urdf_link_inertial_api.GetIzzAttr().Get()
            ixy = urdf_link_inertial_api.GetIxyAttr().Get()
            ixz = urdf_link_inertial_api.GetIxzAttr().Get()
            iyz = urdf_link_inertial_api.GetIyzAttr().Get()
            inertia = urdf.Inertia(ixx=ixx if ixx is not None else 1e-6,
                                   iyy=iyy if iyy is not None else 1e-6,
                                   izz=izz if izz is not None else 1e-6,
                                   ixy=ixy if ixy is not None else 0.0,
                                   ixz=ixz if ixz is not None else 0.0,
                                   iyz=iyz if iyz is not None else 0.0)
            link.inertial = urdf.Inertial(origin=origin, mass=mass, inertia=inertia)

            geom_builder: GeomBuilder

        for geom_builder in body_builder.geom_builders:
            self._build_geom(geom_builder=geom_builder, link=link)

        self.robot.add_link(link)

    def _build_geom(self, geom_builder: GeomBuilder, link: urdf.Link):
        if len(geom_builder.geom_prims) > 1:
            raise NotImplementedError("Multiple geometry prims not supported yet.")
        for geom_prim in geom_builder.geom_prims:
            urdf_geometry_api = get_urdf_geometry_api(geom_prim=geom_prim)
            xform_prim = geom_builder.xform.GetPrim()
            geom_name = xform_prim.GetName()
            if geom_builder.type == GeomType.CUBE:
                urdf_geometry_box_api = get_urdf_geometry_box_api(geom_builder=geom_builder)
                size = urdf_geometry_box_api.GetSizeAttr().Get()
                geometry = urdf.Box(size=size)
                build_geom(geom_name=geom_name,
                           link=link,
                           geometry=geometry,
                           urdf_geometry_api=urdf_geometry_api)
            elif geom_builder.type == GeomType.SPHERE:
                urdf_geometry_sphere_api = get_urdf_geometry_sphere_api(geom_builder=geom_builder)
                radius = urdf_geometry_sphere_api.GetRadiusAttr().Get()
                geometry = urdf.Sphere(radius=radius)
                build_geom(geom_name=geom_name,
                           link=link,
                           geometry=geometry,
                           urdf_geometry_api=urdf_geometry_api)
            elif geom_builder.type in [GeomType.CYLINDER, GeomType.CAPSULE]:
                urdf_geometry_cylinder_api = get_urdf_geometry_cylinder_api(geom_builder=geom_builder)
                radius = urdf_geometry_cylinder_api.GetRadiusAttr().Get()
                length = urdf_geometry_cylinder_api.GetLengthAttr().Get()
                geometry = urdf.Cylinder(radius=radius, length=length)
                build_geom(geom_name=geom_name,
                           link=link,
                           geometry=geometry,
                           urdf_geometry_api=urdf_geometry_api)
            elif geom_builder.type == GeomType.MESH:
                for usd_file_path, mesh_builder in geom_builder.mesh_builders.items():
                    mesh_rel_path = get_mesh_rel_path(geom_prim=geom_prim, usd_file_path=usd_file_path)
                    mesh_abs_path = os.path.join(self._meshdir_abs, mesh_rel_path)
                    mesh_ros_path = os.path.join(self._meshdir_ros, mesh_rel_path)
                    self.factory.export_mesh(in_mesh_file_path=usd_file_path,
                                             out_mesh_file_path=mesh_abs_path)

                    urdf_geometry_mesh_api = get_urdf_geometry_mesh_api(geom_builder=geom_builder,
                                                                        mesh_abs_path=mesh_abs_path)
                    scale = urdf_geometry_mesh_api.GetScaleAttr().Get()

                    geometry = urdf.Mesh(filename=mesh_ros_path, scale=scale)
                    build_geom(geom_name=geom_name,
                               link=link,
                               geometry=geometry,
                               urdf_geometry_api=urdf_geometry_api)
            else:
                raise NotImplementedError(f"Geom type {geom_builder.type} not supported yet.")

    def export(self):
        os.makedirs(name=os.path.dirname(self.file_path), exist_ok=True)

        xml_string = self.robot.to_xml_string()

        with open(self.file_path, "w") as file:
            file.write(xml_string)

    @property
    def file_path(self) -> str:
        return self._file_path

    @property
    def factory(self) -> Factory:
        return self._factory

    @property
    def robot(self) -> urdf.URDF:
        return self._robot
