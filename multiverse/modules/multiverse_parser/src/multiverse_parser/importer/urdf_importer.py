#!/usr/bin/env python3

import os
from math import degrees
from typing import Optional, Union, Dict

import numpy
from scipy.spatial.transform import Rotation
from urdf_parser_py import urdf

from .importer import Configuration, Importer
from ..factory import InertiaSource
from ..factory import WorldBuilder, BodyBuilder, JointBuilder, JointType, GeomBuilder, GeomType, GeomProperty, \
    JointProperty
from ..utils import xform_cache, shift_inertia_tensor, diagonalize_inertia

from pxr import UsdUrdf, Gf


def get_joint_pos_and_quat(urdf_joint) -> (numpy.ndarray, numpy.ndarray):
    if hasattr(urdf_joint, "origin") and urdf_joint.origin is not None:
        joint_pos = urdf_joint.origin.xyz
        joint_rpy = urdf_joint.origin.rpy
    else:
        joint_pos = numpy.array([0.0, 0.0, 0.0])
        joint_rpy = numpy.array([0.0, 0.0, 0.0])
    joint_quat = Rotation.from_euler('xyz', joint_rpy).as_quat()
    return joint_pos, joint_quat


class UrdfImporter(Importer):
    world_builder: WorldBuilder
    urdf_model: urdf.URDF
    _geom_type_map: Dict = {
        urdf.Box: GeomType.CUBE,
        urdf.Sphere: GeomType.SPHERE,
        urdf.Cylinder: GeomType.CYLINDER,
        urdf.Mesh: GeomType.MESH,
    }

    def __init__(
            self,
            file_path: str,
            with_physics: bool,
            with_visual: bool,
            with_collision: bool,
            inertia_source: InertiaSource = InertiaSource.FROM_SRC,
            default_rgba: Optional[numpy.ndarray] = None,
    ) -> None:
        self._world_builder = None
        with open(file_path) as file:
            urdf_string = file.read()
        self._urdf_model = urdf.URDF.from_xml_string(urdf_string)

        model_name = self.urdf_model.name
        super().__init__(file_path=file_path, config=Configuration(
            model_name=model_name,
            with_physics=with_physics,
            with_visual=with_visual,
            with_collision=with_collision,
            default_rgba=default_rgba,
            inertia_source=inertia_source
        ))

    def import_model(self, save_file_path: Optional[str] = None) -> str:
        self._world_builder = WorldBuilder(self.tmp_file_path)

        self._import_config()

        body_builder = self.world_builder.add_body(body_name=self._config.model_name)
        if self._config.model_name != self.urdf_model.get_root():
            print(f"Root link {self.urdf_model.get_root()} is not the model name {self._config.model_name}, "
                  f"add it as a root body.")
            body_builder = self.world_builder.add_body(body_name=self.urdf_model.get_root(),
                                                       parent_body_name=self._config.model_name)
            body_builder.enable_rigid_body()

        self._import_inertial(body=self.urdf_model.link_map[self.urdf_model.get_root()],
                              body_builder=body_builder)

        self._import_geoms(link=self.urdf_model.link_map[self.urdf_model.get_root()],
                           body_builder=body_builder)

        self._import_body_and_joint(urdf_link_name=self.urdf_model.get_root())

        self.world_builder.export()

        return self.tmp_file_path if save_file_path is None else self.save_tmp_model(file_path=save_file_path)

    def _import_config(self) -> None:
        usd_urdf = UsdUrdf.Urdf.Define(self.world_builder.stage, "/urdf")
        usd_urdf.CreateNameAttr(self._config.model_name)

    def _import_body_and_joint(self, urdf_link_name) -> None:
        if urdf_link_name not in self.urdf_model.child_map:
            return

        for child_joint_name, child_urdf_link_name in self.urdf_model.child_map[urdf_link_name]:
            child_urdf_joint: urdf.Joint = self.urdf_model.joint_map[child_joint_name]

            body_builder = self._import_body(body_name=urdf_link_name,
                                             child_body_name=child_urdf_link_name,
                                             joint=child_urdf_joint)

            self._import_geoms(link=self.urdf_model.link_map[child_urdf_link_name], body_builder=body_builder)

            if self._config.with_physics:
                self._import_joint(joint=child_urdf_joint,
                                   parent_body_name=urdf_link_name,
                                   child_body_name=child_urdf_link_name)

                self._import_body_and_joint(urdf_link_name=child_urdf_link_name)

    def _import_body(self, body_name: str, child_body_name: str, joint: urdf.Joint) -> BodyBuilder:
        joint_pos, joint_quat = get_joint_pos_and_quat(joint)

        if self._config.with_physics and joint.type != "fixed":
            body_builder = self.world_builder.add_body(body_name=child_body_name,
                                                       parent_body_name=self._config.model_name)
            body_builder.enable_rigid_body()
        else:
            body_builder = self.world_builder.add_body(body_name=child_body_name,
                                                       parent_body_name=body_name)

        relative_to_body_builder = self.world_builder.get_body_builder(body_name=body_name)
        relative_to_xform = relative_to_body_builder.xform
        body_builder.set_transform(pos=joint_pos, quat=joint_quat, relative_to_xform=relative_to_xform)

        self._import_inertial(body=self.urdf_model.link_map[child_body_name], body_builder=body_builder)

        urdf_link_api = UsdUrdf.UrdfLinkAPI.Apply(body_builder.xform.GetPrim())

        return body_builder

    def _import_inertial(self, body: urdf.Link, body_builder: BodyBuilder) -> None:
        if self._config.with_physics and self._config.inertia_source == InertiaSource.FROM_SRC:
            if body.inertial is not None:
                body_mass = body.inertial.mass
                body_center_of_mass = body.inertial.origin.xyz
                body_inertia = body.inertial.inertia
                body_inertia_tensor = numpy.array([[body_inertia.ixx, body_inertia.ixy, body_inertia.ixz],
                                                   [body_inertia.ixy, body_inertia.iyy, body_inertia.iyz],
                                                   [body_inertia.ixz, body_inertia.iyz, body_inertia.izz]])
                body_inertia_tensor = shift_inertia_tensor(mass=body_mass,
                                                           inertia_tensor=body_inertia_tensor,
                                                           quat=Rotation.from_euler('xyz',
                                                                                    body.inertial.origin.rpy).inv()
                                                           .as_quat())

                body_diagonal_inertia, body_principal_axes = diagonalize_inertia(inertia_tensor=body_inertia_tensor)

                body_builder.set_inertial(mass=body_mass,
                                          center_of_mass=body_center_of_mass,
                                          diagonal_inertia=body_diagonal_inertia,
                                          principal_axes=body_principal_axes)

    def _import_geoms(self, link: urdf.Link, body_builder: BodyBuilder) -> Dict[str, GeomBuilder]:
        geom_builders = {}
        geom_name = f"{link.name}_geom"
        if self._config.with_visual:
            for i, visual in enumerate(link.visuals):
                visual_geom_name = f"{geom_name}_visual_{i}"
                if visual_geom_name not in geom_builders:
                    geom_builders[visual_geom_name] = self._import_geom(geom_name=visual_geom_name,
                                                                        geom=visual,
                                                                        body_builder=body_builder)
                else:
                    raise ValueError(f"Geom {visual_geom_name} already exists.")
        if self._config.with_collision:
            for i, collision in enumerate(link.collisions):
                collision_geom_name = f"{geom_name}_collision_{i}"
                if collision_geom_name not in geom_builders:
                    geom_builders[collision_geom_name] = self._import_geom(geom_name=collision_geom_name,
                                                                           geom=collision,
                                                                           body_builder=body_builder)
                else:
                    raise ValueError(f"Geom {collision_geom_name} already exists.")
        return geom_builders

    def _import_geom(self, geom_name: str, geom: Union[urdf.Visual, urdf.Collision],
                     body_builder: BodyBuilder) -> GeomBuilder:
        if geom.origin is not None:
            geom_pos = geom.origin.xyz
            geom_rot = geom.origin.rpy
        else:
            geom_pos = numpy.array([0.0, 0.0, 0.0])
            geom_rot = numpy.array([0.0, 0.0, 0.0])
        geom_quat = Rotation.from_euler('xyz', geom_rot).as_quat()

        geom_is_visible = isinstance(geom, urdf.Visual)
        geom_is_collidable = isinstance(geom, urdf.Collision)
        geom_rgba = self._config.default_rgba if not hasattr(geom, "material") or not hasattr(geom.material, "color") \
            else geom.material.color.rgba
        geom_type = self._geom_type_map[type(geom.geometry)]
        geom_density = 1000.0
        geom_property = GeomProperty(geom_name=geom_name,
                                     geom_type=geom_type,
                                     is_visible=geom_is_visible,
                                     is_collidable=geom_is_collidable,
                                     rgba=geom_rgba,
                                     density=geom_density)
        geom_builder = body_builder.add_geom(geom_property=geom_property)
        geom_builder.build()

        tmp_origin_mesh_file_path = ""
        if type(geom.geometry) is urdf.Box:
            geom_scale = numpy.array([geom.geometry.size[i] / 2.0 for i in range(3)])
            geom_builder.set_transform(pos=geom_pos, quat=geom_quat, scale=geom_scale)
        elif type(geom.geometry) is urdf.Sphere:
            geom_builder.set_transform(pos=geom_pos, quat=geom_quat)
            geom_builder.set_attribute(radius=geom.geometry.radius)
        elif type(geom.geometry) is urdf.Cylinder:
            geom_builder.set_transform(pos=geom_pos, quat=geom_quat)
            geom_builder.set_attribute(radius=geom.geometry.radius, height=geom.geometry.length)
        elif type(geom.geometry) is urdf.Mesh:
            source_mesh_file_path = self.get_mesh_file_path(urdf_mesh_file_path=geom.geometry.filename)
            if source_mesh_file_path is not None:
                tmp_usd_mesh_file_path, tmp_origin_mesh_file_path = self.import_mesh(
                    mesh_file_path=source_mesh_file_path)
                tmp_origin_mesh_file_path = "file://" + tmp_origin_mesh_file_path
                geom_builder.add_mesh(mesh_file_path=tmp_usd_mesh_file_path)
                geom_builder.build()
                geom_scale = numpy.array([1.0, 1.0, 1.0]) if geom.geometry.scale is None else geom.geometry.scale
                geom_builder.set_transform(pos=geom_pos, quat=geom_quat, scale=geom_scale)
        else:
            raise ValueError(f"Geom type {type(geom.geometry)} not implemented.")

        geom_xform_prim = geom_builder.xform.GetPrim()
        urdf_geometry_api = UsdUrdf.UrdfGeometryAPI.Apply(geom_xform_prim)
        urdf_geom_type_str = "box" if type(geom.geometry) is urdf.Box \
            else "sphere" if type(geom.geometry) is urdf.Sphere \
            else "cylinder" if type(geom.geometry) is urdf.Cylinder \
            else "mesh" if type(geom.geometry) is urdf.Mesh \
            else None
        if urdf_geom_type_str is None:
            raise NotImplementedError(f"Geom type {type(geom.geometry)} not implemented.")
        urdf_geometry_api.CreateTypeAttr(urdf_geom_type_str)
        if type(geom.geometry) is urdf.Box:
            urdf_geometry_api.CreateSizeAttr(Gf.Vec3f(*geom.geometry.size))
        elif type(geom.geometry) is urdf.Sphere:
            urdf_geometry_api.CreateRadiusAttr(geom.geometry.radius)
        elif type(geom.geometry) is urdf.Cylinder:
            urdf_geometry_api.CreateLengthAttr(geom.geometry.length)
        elif type(geom.geometry) is urdf.Mesh:
            urdf_geometry_api.CreateFileNameAttr(tmp_origin_mesh_file_path)
        else:
            raise ValueError(f"Geom type {type(geom.geometry)} not implemented.")

        return geom_builder

    def get_mesh_file_path(self, urdf_mesh_file_path: str) -> Optional[str]:
        mesh_file_path = None
        if urdf_mesh_file_path.find("package://") != -1:
            urdf_mesh_file_path = urdf_mesh_file_path.replace("package://", "")
            package_name = urdf_mesh_file_path.split("/", 2)[0]
            try:
                import rospkg
                package_path = os.path.dirname(rospkg.RosPack().get_path(package_name))
                mesh_file_path = os.path.join(package_path, urdf_mesh_file_path)
            except (ImportError, rospkg.common.ResourceNotFound):
                print(f"Package {package_name} not found or rospkg not installed, "
                      f"searching for {urdf_mesh_file_path} in {os.getcwd()}...")
                file_paths = []
                for root, _, files in os.walk(os.getcwd()):
                    if urdf_mesh_file_path in files:
                        file_paths.append(os.path.join(root, urdf_mesh_file_path))

                if len(file_paths) == 0:
                    print(f"Mesh file {urdf_mesh_file_path} not found in {os.getcwd()}.")
                    return
                elif len(file_paths) == 1:
                    print(f"Found {file_paths[0]}")
                elif len(file_paths) > 1:
                    print(f"Found {len(file_paths)} meshes {urdf_mesh_file_path} in {os.getcwd()}, "
                          f"take the first one {file_paths[0]}.")
                    mesh_file_path = file_paths[0]

        elif urdf_mesh_file_path.find("file://") != -1:
            mesh_file_path = urdf_mesh_file_path.replace("file://", "")
            if not os.path.isabs(mesh_file_path):
                mesh_file_path = os.path.join(os.path.dirname(self.source_file_path), mesh_file_path)
                if not os.path.exists(mesh_file_path):
                    print(f"Mesh file {mesh_file_path} not found.")
                    mesh_file_path = None

        return mesh_file_path

    def _import_joint(self, joint: urdf.Joint, parent_body_name: str, child_body_name: str) -> Optional[JointBuilder]:
        joint_type = JointType.from_string(joint.type)

        joint_builder = None
        if joint_type != JointType.FIXED and joint_type != JointType.NONE:
            parent_body_builder = self.world_builder.get_body_builder(parent_body_name)
            child_body_builder = self.world_builder.get_body_builder(child_body_name)

            parent_prim = parent_body_builder.xform.GetPrim()
            child_prim = child_body_builder.xform.GetPrim()

            body1_transform = xform_cache.GetLocalToWorldTransform(parent_prim)
            body1_rot = body1_transform.ExtractRotationQuat()

            body2_transform = xform_cache.GetLocalToWorldTransform(child_prim)
            body1_to_body2_transform = body2_transform * body1_transform.GetInverse()
            body1_to_body2_pos = body1_to_body2_transform.ExtractTranslation()

            joint_pos, _ = get_joint_pos_and_quat(joint)
            joint_pos = Gf.Vec3d(joint_pos)
            joint_pos = body1_rot.GetInverse().Transform(joint_pos - body1_to_body2_pos)

            joint_property = JointProperty(
                joint_name=joint.name,
                joint_parent_prim=parent_body_builder.xform.GetPrim(),
                joint_child_prim=child_body_builder.xform.GetPrim(),
                joint_pos=joint_pos,
                joint_axis=joint.axis,
                joint_type=joint_type,
            )
            joint_builder = child_body_builder.add_joint(joint_property=joint_property)

            if joint_type == JointType.REVOLUTE:
                joint_builder.set_limit(lower=degrees(joint.limit.lower),
                                        upper=degrees(joint.limit.upper))
            elif joint_type == JointType.PRISMATIC:
                joint_builder.set_limit(lower=joint.limit.lower,
                                        upper=joint.limit.upper)

            urdf_joint_api = UsdUrdf.UrdfJointAPI.Apply(joint_builder.joint.GetPrim())
            urdf_joint_api.CreateTypeAttr(joint.type)

            urdf_origin_api = UsdUrdf.UrdfOriginAPI.Apply(joint_builder.joint.GetPrim())
            urdf_origin_api.CreateXyzAttr(Gf.Vec3f(*joint.origin.xyz))
            urdf_origin_api.CreateRpyAttr(Gf.Vec3f(*joint.origin.rpy))

        return joint_builder

    @property
    def world_builder(self) -> WorldBuilder:
        return self._world_builder

    @property
    def urdf_model(self) -> urdf.URDF:
        return self._urdf_model

    @property
    def geom_type_map(self) -> Dict:
        return self._geom_type_map
