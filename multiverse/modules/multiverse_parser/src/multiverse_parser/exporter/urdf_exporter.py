#!/usr/bin/env python3

import os
import shutil
from typing import Tuple, Optional, Union

import numpy
from scipy.spatial.transform import Rotation
from urdf_parser_py import urdf

from ..factory import Factory
from ..factory import (WorldBuilder,
                       BodyBuilder,
                       JointBuilder, JointType,
                       GeomBuilder, GeomType)
from ..importer.urdf_importer import build_urdf_inertial_api
from ..utils import xform_cache

from pxr import UsdUrdf, Gf, UsdPhysics, UsdGeom, Usd, UsdShade


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
        if joint_builder.type in [JointType.REVOLUTE, JointType.CONTINUOUS, JointType.PRISMATIC]:
            joint_axis = joint_builder.quat.Transform(Gf.Vec3d([0.0, 0.0, 1.0]))
            urdf_joint_api.CreateAxisAttr(Gf.Vec3f(*joint_axis))
            if joint_builder.type in [JointType.REVOLUTE, JointType.PRISMATIC]:
                effort = 1000  # TODO: Find a way to get this value
                velocity = 1000  # TODO: Find a way to get this value
                if joint_builder.type == JointType.REVOLUTE:
                    usd_joint = UsdPhysics.RevoluteJoint(joint_prim)
                    upper_limit = usd_joint.GetUpperLimitAttr().Get()
                    lower_limit = usd_joint.GetLowerLimitAttr().Get()
                    upper_limit = numpy.deg2rad(upper_limit)
                    lower_limit = numpy.deg2rad(lower_limit)
                else:
                    usd_joint = UsdPhysics.PrismaticJoint(joint_prim)
                    upper_limit = usd_joint.GetUpperLimitAttr().Get()
                    lower_limit = usd_joint.GetLowerLimitAttr().Get()
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


def get_urdf_geometry_api(gprim_prim: Usd.Prim) -> Union[UsdUrdf.UrdfLinkVisualAPI, UsdUrdf.UrdfLinkCollisionAPI]:
    if gprim_prim.HasAPI(UsdUrdf.UrdfLinkVisualAPI):
        urdf_geometry_api = UsdUrdf.UrdfLinkVisualAPI(gprim_prim)
    elif gprim_prim.HasAPI(UsdUrdf.UrdfLinkCollisionAPI):
        urdf_geometry_api = UsdUrdf.UrdfLinkCollisionAPI(gprim_prim)
    else:
        xform = UsdGeom.Xform(gprim_prim)
        xyz, rpy = get_urdf_origin(xform)

        if not gprim_prim.HasAPI(UsdPhysics.CollisionAPI):
            urdf_geometry_api = UsdUrdf.UrdfLinkVisualAPI.Apply(gprim_prim)
        else:
            urdf_geometry_api = UsdUrdf.UrdfLinkCollisionAPI.Apply(gprim_prim)

        urdf_geometry_api.CreateXyzAttr(xyz)
        urdf_geometry_api.CreateRpyAttr(rpy)

    return urdf_geometry_api


def build_geom(geom_name: str,
               link: urdf.Link,
               geometry: Union[urdf.Box, urdf.Sphere, urdf.Cylinder, urdf.Mesh],
               urdf_geometry_api: Union[UsdUrdf.UrdfLinkVisualAPI, UsdUrdf.UrdfLinkCollisionAPI],
               material: Optional[urdf.Material]) -> None:
    xyz = urdf_geometry_api.GetXyzAttr().Get()
    rpy = urdf_geometry_api.GetRpyAttr().Get()
    origin = urdf.Pose(xyz=xyz, rpy=rpy)

    if isinstance(urdf_geometry_api, UsdUrdf.UrdfLinkVisualAPI):
        visual = urdf.Visual(
            geometry=geometry,
            origin=origin,
            name=geom_name,
            material=material)
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
    gprim = geom_builder.gprim
    gprim_prim = gprim.GetPrim()
    if gprim_prim.HasAPI(UsdUrdf.UrdfGeometryBoxAPI):
        urdf_geometry_box_api = UsdUrdf.UrdfGeometryBoxAPI(gprim_prim)
    else:
        urdf_geometry_box_api = UsdUrdf.UrdfGeometryBoxAPI.Apply(gprim_prim)
        size = numpy.array([gprim.GetLocalTransformation().GetRow(i).GetLength() for i in range(3)]) * 2
        size = Gf.Vec3f(*size)
        urdf_geometry_box_api.CreateSizeAttr(size)
    return urdf_geometry_box_api


def get_urdf_geometry_sphere_api(geom_builder: GeomBuilder) -> UsdUrdf.UrdfGeometrySphereAPI:
    gprim = geom_builder.gprim
    gprim_prim = gprim.GetPrim()
    sphere = UsdGeom.Sphere(gprim_prim)
    if gprim_prim.HasAPI(UsdUrdf.UrdfGeometrySphereAPI):
        urdf_geometry_sphere_api = UsdUrdf.UrdfGeometrySphereAPI(gprim_prim)
    else:
        urdf_geometry_sphere_api = UsdUrdf.UrdfGeometrySphereAPI.Apply(gprim_prim)
        radius = sphere.GetRadiusAttr().Get()
        urdf_geometry_sphere_api.CreateRadiusAttr(radius)
    return urdf_geometry_sphere_api


def get_urdf_geometry_cylinder_api(geom_builder: GeomBuilder) -> UsdUrdf.UrdfGeometryCylinderAPI:
    gprim = geom_builder.gprim
    gprim_prim = gprim.GetPrim()
    cylinder = UsdGeom.Cylinder(gprim_prim)
    if gprim_prim.HasAPI(UsdUrdf.UrdfGeometryCylinderAPI):
        urdf_geometry_cylinder_api = UsdUrdf.UrdfGeometryCylinderAPI(gprim_prim)
    else:
        urdf_geometry_cylinder_api = UsdUrdf.UrdfGeometryCylinderAPI.Apply(gprim_prim)
        radius = cylinder.GetRadiusAttr().Get()
        length = cylinder.GetHeightAttr().Get()
        urdf_geometry_cylinder_api.CreateRadiusAttr(radius)
        urdf_geometry_cylinder_api.CreateLengthAttr(length)
    return urdf_geometry_cylinder_api


def get_urdf_geometry_mesh_api(geom_builder: GeomBuilder, mesh_file_relpath: str) -> UsdUrdf.UrdfGeometryMeshAPI:
    gprim = geom_builder.gprim
    gprim_prim = gprim.GetPrim()
    if not gprim_prim.HasAPI(UsdUrdf.UrdfGeometryMeshAPI):
        urdf_geometry_mesh_api = UsdUrdf.UrdfGeometryMeshAPI.Apply(gprim_prim)
        urdf_geometry_mesh_api.CreateFilenameAttr(f"./{mesh_file_relpath}")
        transformation = gprim.GetLocalTransformation()

        # TODO: Doesn't work for negative scale
        scale = [transformation.GetRow(i).GetLength() for i in range(3)]
        scale = Gf.Vec3f(*scale)
        urdf_geometry_mesh_api.CreateScaleAttr(scale)
    else:
        urdf_geometry_mesh_api = UsdUrdf.UrdfGeometryMeshAPI(gprim_prim)
    return urdf_geometry_mesh_api


class UrdfExporter:
    def __init__(
            self,
            factory: Factory,
            file_path: str,
            relative_to_ros_package: Optional[str] = None,
            visual_mesh_file_extension: str = "obj",
    ) -> None:
        self._factory = factory
        robot_name = get_robot_name(world_builder=factory.world_builder)
        self._robot = urdf.URDF(name=robot_name)
        self._ros_package_path = None
        self._file_path = file_path
        (self._mesh_dir_abspath,
         self._mesh_dir_rospath) = self._get_mesh_dir_paths(relative_to_ros_package=relative_to_ros_package)
        self._visual_mesh_file_extension = visual_mesh_file_extension
        self._mesh_dict = {}

    def build(self) -> None:
        for body_builder in self.factory.world_builder.body_builders:
            self._build_joints(body_builder=body_builder)
            self._build_link(body_builder=body_builder)

        self._build_meshes()

    def _get_mesh_dir_paths(self, relative_to_ros_package: Optional[str] = None) -> Tuple[str, str]:
        file_path = self.file_path
        meshdir_name = os.path.splitext(os.path.basename(file_path))[0]
        mesh_dir_abspath = str(os.path.join(os.path.dirname(file_path), meshdir_name, "meshes"))
        if relative_to_ros_package is not None:
            if self._ros_package_path is not None:
                ros_package_path = self._ros_package_path
            else:
                import rospkg
                rospack = rospkg.RosPack()
                try:
                    self._ros_package_path = rospack.get_path(relative_to_ros_package)
                except rospkg.ResourceNotFound:
                    print(f"Could not find ROS package {relative_to_ros_package}, "
                          f"searching for package.xml in parent directories of {file_path}.")
                    self._ros_package_path = file_path
                    mesh_dir_relpath = meshdir_name
                    while self._ros_package_path != "/":
                        self._ros_package_path = os.path.dirname(self._ros_package_path)
                        mesh_dir_relpath = os.path.join(os.path.basename(self._ros_package_path), str(mesh_dir_relpath))

                        if os.path.exists(os.path.join(self._ros_package_path, "package.xml")):
                            print(f"Found package.xml in {self._ros_package_path}.")
                            break
                    else:
                        raise FileNotFoundError(
                            f"Could not find package.xml in any parent directory of {file_path}.")
                ros_package_path = self._ros_package_path

            mesh_dir_relpath = os.path.relpath(mesh_dir_abspath, ros_package_path)
            mesh_dir_rospath = "package://" + relative_to_ros_package + "/" + mesh_dir_relpath
        else:
            # mesh_dir_rospath = "file://" + mesh_dir_abspath
            mesh_dir_relpath = os.path.relpath(mesh_dir_abspath, os.path.dirname(file_path))
            mesh_dir_rospath = "file://" + mesh_dir_relpath

        return mesh_dir_abspath, mesh_dir_rospath

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

        if len(urdf_joint_api.GetJointRel().GetTargets()) == 1:
            mimic_joint_path = urdf_joint_api.GetJointRel().GetTargets()[0]
            mimic_joint_prim = joint_prim.GetStage().GetPrimAtPath(mimic_joint_path)
            mimic = urdf.JointMimic()
            mimic.joint = mimic_joint_prim.GetName()
            mimic.multiplier = urdf_joint_api.GetMultiplierAttr().Get()
            mimic.offset = urdf_joint_api.GetOffsetAttr().Get()
            urdf_joint.mimic = mimic

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

        self.factory.execute_cmds()

        self.robot.add_link(link)

    def _build_geom(self, geom_builder: GeomBuilder, link: urdf.Link):
        gprim_prim = geom_builder.gprim.GetPrim()
        urdf_geometry_api = get_urdf_geometry_api(gprim_prim=gprim_prim)
        geom_name = gprim_prim.GetName()
        material = urdf.Material(name=geom_name + "_material", color=urdf.Color(geom_builder.rgba)) if geom_builder.rgba is not None else None
        if geom_builder.type == GeomType.CUBE:
            urdf_geometry_box_api = get_urdf_geometry_box_api(geom_builder=geom_builder)
            size = urdf_geometry_box_api.GetSizeAttr().Get()
            geometry = urdf.Box(size=size)
            build_geom(geom_name=geom_name,
                       link=link,
                       geometry=geometry,
                       urdf_geometry_api=urdf_geometry_api,
                       material=material)
        elif geom_builder.type == GeomType.SPHERE:
            urdf_geometry_sphere_api = get_urdf_geometry_sphere_api(geom_builder=geom_builder)
            radius = urdf_geometry_sphere_api.GetRadiusAttr().Get()
            geometry = urdf.Sphere(radius=radius)
            build_geom(geom_name=geom_name,
                       link=link,
                       geometry=geometry,
                       urdf_geometry_api=urdf_geometry_api,
                       material=material)
        elif geom_builder.type in [GeomType.CYLINDER, GeomType.CAPSULE]:
            urdf_geometry_cylinder_api = get_urdf_geometry_cylinder_api(geom_builder=geom_builder)
            radius = urdf_geometry_cylinder_api.GetRadiusAttr().Get()
            length = urdf_geometry_cylinder_api.GetLengthAttr().Get()
            geometry = urdf.Cylinder(radius=radius, length=length)
            build_geom(geom_name=geom_name,
                       link=link,
                       geometry=geometry,
                       urdf_geometry_api=urdf_geometry_api,
                       material=material)
        elif geom_builder.type == GeomType.MESH:
            usd_mesh_file_abspath = self._get_file_abspath_from_reference(prim=gprim_prim)
            tmp_usd_mesh_file_abspath = usd_mesh_file_abspath

            subset_prims = [subset_prim for subset_prim in gprim_prim.GetChildren()
                            if subset_prim.IsA(UsdGeom.Subset) and subset_prim.HasAPI(UsdShade.MaterialBindingAPI)]
            if gprim_prim.HasAPI(UsdShade.MaterialBindingAPI) or len(subset_prims) > 0:
                tmp_usd_mesh_file_abspath = tmp_usd_mesh_file_abspath.replace(".usda", "_tmp.usda")
                shutil.copy2(usd_mesh_file_abspath, tmp_usd_mesh_file_abspath)
                mesh_stage = Usd.Stage.Open(tmp_usd_mesh_file_abspath)
                mesh_prim = mesh_stage.GetDefaultPrim()
                mesh_path = mesh_prim.GetPath()
                mesh_material_scope_path = mesh_path.AppendChild("Materials")
                UsdGeom.Scope.Define(mesh_stage, mesh_material_scope_path)

                if gprim_prim.HasAPI(UsdShade.MaterialBindingAPI):
                    material_binding_api = UsdShade.MaterialBindingAPI(gprim_prim)
                    material_path = material_binding_api.GetDirectBindingRel().GetTargets()[0]
                    material_prim = geom_builder.stage.GetPrimAtPath(material_path)
                    material_name = material_prim.GetName()
                    usd_material_file_abspath = self._get_file_abspath_from_reference(prim=material_prim)
                    usd_material_file_relpath = os.path.relpath(usd_material_file_abspath,
                                                                os.path.dirname(usd_mesh_file_abspath))

                    mesh_material_path = mesh_material_scope_path.AppendChild(material_name)
                    mesh_material = UsdShade.Material.Define(mesh_stage, mesh_material_path)
                    mesh_material_prim = mesh_material.GetPrim()
                    material_stage = Usd.Stage.Open(usd_material_file_abspath)
                    ref_material_prim = material_stage.GetDefaultPrim()
                    ref_material_path = ref_material_prim.GetPath()
                    mesh_material_prim.GetReferences().AddReference(f"./{usd_material_file_relpath}", ref_material_path)
                    mesh_material_binding_api = UsdShade.MaterialBindingAPI.Apply(mesh_prim)
                    mesh_material_binding_api.Bind(mesh_material)
                    mesh_stage.GetRootLayer().Save()

                for subset_prim in subset_prims:
                    material_binding_api = UsdShade.MaterialBindingAPI(subset_prim)
                    material_path = material_binding_api.GetDirectBindingRel().GetTargets()[0]
                    material_prim = geom_builder.stage.GetPrimAtPath(material_path)
                    material_name = material_prim.GetName()
                    usd_material_file_abspath = self._get_file_abspath_from_reference(prim=material_prim)
                    usd_material_file_relpath = os.path.relpath(usd_material_file_abspath,
                                                                os.path.dirname(usd_mesh_file_abspath))

                    subset_material_path = mesh_material_scope_path.AppendChild(material_name)
                    subset_material = UsdShade.Material.Define(mesh_stage, subset_material_path)
                    subset_material_prim = subset_material.GetPrim()
                    material_stage = Usd.Stage.Open(usd_material_file_abspath)
                    ref_material_prim = material_stage.GetDefaultPrim()
                    ref_material_path = ref_material_prim.GetPath()
                    subset_material_prim.GetReferences().AddReference(f"./{usd_material_file_relpath}", ref_material_path)

                    subset = UsdGeom.Subset(subset_prim)
                    mesh_subset_path = mesh_path.AppendChild(subset_prim.GetName())
                    mesh_subset = UsdGeom.Subset.Define(mesh_stage, mesh_subset_path)
                    mesh_subset.CreateElementTypeAttr(subset.GetElementTypeAttr().Get())
                    mesh_subset.CreateIndicesAttr(subset.GetIndicesAttr().Get())
                    mesh_subset.CreateFamilyNameAttr(subset.GetFamilyNameAttr().Get())

                    subset_material_binding_api = UsdShade.MaterialBindingAPI.Apply(mesh_subset.GetPrim())
                    subset_material_binding_api.Bind(subset_material)
                    mesh_stage.GetRootLayer().Save()

            tmp_mesh_file_relpath = self._get_mesh_file_relpath(gprim_prim=gprim_prim,
                                                                usd_mesh_file_path=usd_mesh_file_abspath)
            tmp_mesh_file_abspath = os.path.join(self.factory.tmp_mesh_dir_path, tmp_mesh_file_relpath)
            self.factory.export_mesh(in_mesh_file_path=tmp_usd_mesh_file_abspath,
                                     out_mesh_file_path=tmp_mesh_file_abspath, execute_later=True)
            self._mesh_dict[geom_name] = (tmp_usd_mesh_file_abspath,
                                          tmp_mesh_file_relpath,
                                          geom_builder,
                                          link,
                                          urdf_geometry_api,
                                          material)
        else:
            raise NotImplementedError(f"Geom type {geom_builder.type} not supported yet.")

    def _build_meshes(self):
        for geom_name, (tmp_usd_mesh_file_abspath,
                        tmp_mesh_file_relpath,
                        geom_builder,
                        link,
                        urdf_geometry_api,
                        material) in self._mesh_dict.items():
            if "_tmp.usda" in tmp_usd_mesh_file_abspath and os.path.exists(tmp_usd_mesh_file_abspath):
                os.remove(tmp_usd_mesh_file_abspath)

            mesh_file_abspath = os.path.join(self._mesh_dir_abspath, tmp_mesh_file_relpath)
            mesh_file_relpath = os.path.relpath(path=mesh_file_abspath,
                                                start=os.path.dirname(self.file_path))
            urdf_geometry_mesh_api = get_urdf_geometry_mesh_api(geom_builder=geom_builder,
                                                                mesh_file_relpath=mesh_file_relpath)
            scale = urdf_geometry_mesh_api.GetScaleAttr().Get()

            mesh_file_rospath = os.path.join(self._mesh_dir_rospath, tmp_mesh_file_relpath)
            geometry = urdf.Mesh(filename=mesh_file_rospath, scale=scale)
            build_geom(geom_name=geom_name,
                       link=link,
                       geometry=geometry,
                       urdf_geometry_api=urdf_geometry_api,
                       material=material)

    def _get_mesh_file_relpath(self, gprim_prim: UsdGeom.Gprim, usd_mesh_file_path: str) -> str:
        if (gprim_prim.HasAPI(UsdShade.MaterialBindingAPI) or
                any([subset_prim.HasAPI(UsdShade.MaterialBindingAPI) for subset_prim in gprim_prim.GetChildren()])):
            file_extension = self.visual_mesh_file_extension
        else:
            file_extension = "stl"
        return os.path.join(f"{file_extension}",
                            os.path.splitext(os.path.basename(usd_mesh_file_path))[0] + f".{file_extension}")

    def _get_file_abspath_from_reference(self, prim: Usd.Prim) -> str:
        prepended_items = prim.GetPrimStack()[0].referenceList.prependedItems
        if len(prepended_items) != 1:
            raise NotImplementedError(f"Prim {prim.GetName()} has {len(prepended_items)} prepended items.")

        prepended_item = prepended_items[0]
        file_abspath = prepended_item.assetPath
        if not os.path.isabs(file_abspath):
            file_abspath = os.path.join(os.path.dirname(self.factory.tmp_usd_file_path), file_abspath)
        return file_abspath

    def export(self, keep_usd: bool = True):
        os.makedirs(name=os.path.dirname(self.file_path), exist_ok=True)

        xml_string = self.robot.to_xml_string()

        with open(self.file_path, "w") as file:
            file.write(xml_string)

        if keep_usd:
            self.factory.save_tmp_model(usd_file_path=self.file_path.replace(".urdf", ".usda"))
        else:
            self.factory.save_tmp_model(usd_file_path=self.file_path.replace(".urdf", ".usda"),
                                        excludes=["usd", ".usda"])

    @property
    def file_path(self) -> str:
        return self._file_path

    @property
    def factory(self) -> Factory:
        return self._factory

    @property
    def robot(self) -> urdf.URDF:
        return self._robot

    @property
    def visual_mesh_file_extension(self) -> str:
        return self._visual_mesh_file_extension
