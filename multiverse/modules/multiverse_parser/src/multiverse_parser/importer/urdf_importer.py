#!/usr/bin/env python3

import os
from math import degrees
from typing import Optional, Union, Dict

import numpy
from scipy.spatial.transform import Rotation
from urdf_parser_py import urdf

from ..factory import Factory, Configuration, InertiaSource
from ..factory import (WorldBuilder,
                       BodyBuilder,
                       JointBuilder, JointAxis, JointType, JointProperty,
                       GeomType, GeomProperty,
                       MeshProperty,
                       MaterialProperty)
from ..utils import xform_cache, shift_inertia_tensor, diagonalize_inertia

from pxr import UsdUrdf, Gf, UsdPhysics, Usd, UsdGeom, UsdShade


def build_urdf_inertial_api(physics_mass_api: UsdPhysics.MassAPI) -> UsdUrdf.UrdfLinkInertialAPI:
    mass = physics_mass_api.GetMassAttr().Get()
    xyz = physics_mass_api.GetCenterOfMassAttr().Get()
    quat = physics_mass_api.GetPrincipalAxesAttr().Get()
    quat = numpy.array([*quat.GetImaginary(), quat.GetReal()])
    rpy = Rotation.from_quat(quat).as_euler("xyz", degrees=False)
    diagonal_inertia = physics_mass_api.GetDiagonalInertiaAttr().Get()

    prim = physics_mass_api.GetPrim()
    urdf_link_inertial_api = UsdUrdf.UrdfLinkInertialAPI.Apply(prim)
    urdf_link_inertial_api.CreateMassAttr(mass)
    urdf_link_inertial_api.CreateXyzAttr(Gf.Vec3f(*xyz))
    urdf_link_inertial_api.CreateRpyAttr(Gf.Vec3f(*rpy))
    urdf_link_inertial_api.CreateIxxAttr(diagonal_inertia[0])
    urdf_link_inertial_api.CreateIyyAttr(diagonal_inertia[1])
    urdf_link_inertial_api.CreateIzzAttr(diagonal_inertia[2])

    return urdf_link_inertial_api


def get_joint_pos_and_quat(urdf_joint) -> (numpy.ndarray, numpy.ndarray):
    if hasattr(urdf_joint, "origin") and urdf_joint.origin is not None:
        joint_pos = urdf_joint.origin.xyz
        joint_rpy = urdf_joint.origin.rpy
    else:
        joint_pos = numpy.array([0.0, 0.0, 0.0])
        joint_rpy = numpy.array([0.0, 0.0, 0.0])
    joint_quat = Rotation.from_euler('xyz', joint_rpy).as_quat()
    return joint_pos, joint_quat


def get_urdf_link_api(geom: Union[urdf.Visual, urdf.Collision], gprim_prim: Usd.Prim):
    if type(geom) is urdf.Visual:
        urdf_link_api = UsdUrdf.UrdfLinkVisualAPI.Apply(gprim_prim)
    elif type(geom) is urdf.Collision:
        urdf_link_api = UsdUrdf.UrdfLinkCollisionAPI.Apply(gprim_prim)
    else:
        raise ValueError(f"Geom type {type(geom)} not supported.")

    if geom.origin is not None:
        urdf_link_api.CreateXyzAttr(Gf.Vec3f(*geom.origin.xyz))
        urdf_link_api.CreateRpyAttr(Gf.Vec3f(*geom.origin.rpy))
    else:
        urdf_link_api.CreateXyzAttr(Gf.Vec3f(0.0, 0.0, 0.0))
        urdf_link_api.CreateRpyAttr(Gf.Vec3f(0.0, 0.0, 0.0))

    return urdf_link_api


class UrdfImporter(Factory):
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
            fixed_base: bool,
            with_physics: bool,
            with_visual: bool,
            with_collision: bool,
            inertia_source: InertiaSource = InertiaSource.FROM_SRC,
            default_rgba: Optional[numpy.ndarray] = None,
    ) -> None:
        with open(file_path) as file:
            urdf_string = file.read()
        self._urdf_model = urdf.URDF.from_xml_string(urdf_string)

        model_name = self.urdf_model.name
        super().__init__(file_path=file_path, config=Configuration(
            model_name=model_name,
            fixed_base=fixed_base,
            with_physics=with_physics,
            with_visual=with_visual,
            with_collision=with_collision,
            default_rgba=default_rgba if default_rgba is not None else numpy.array([0.9, 0.9, 0.9, 1.0]),
            inertia_source=inertia_source
        ))

    def import_model(self, save_file_path: Optional[str] = None) -> str:
        self._world_builder = WorldBuilder(self.tmp_usd_file_path)

        self._import_config()

        body_builder = self.world_builder.add_body(body_name=self._config.model_name)
        if self._config.model_name != self.urdf_model.get_root():
            print(f"Root link {self.urdf_model.get_root()} is not the model name {self._config.model_name}, "
                  f"add it as a root body.")
            body_builder = self.world_builder.add_body(body_name=self.urdf_model.get_root(),
                                                       parent_body_name=self._config.model_name)

        self._import_geoms(link=self.urdf_model.link_map[self.urdf_model.get_root()],
                           body_builder=body_builder)

        self._import_inertial(body=self.urdf_model.link_map[self.urdf_model.get_root()],
                              body_builder=body_builder)

        self._import_body_and_joint(urdf_link_name=self.urdf_model.get_root())

        self.world_builder.export()

        return self.tmp_usd_file_path if save_file_path is None else self.save_tmp_model(usd_file_path=save_file_path)

    def _import_config(self) -> None:
        usd_urdf = UsdUrdf.Urdf.Define(self.world_builder.stage, "/urdf")
        usd_urdf.CreateNameAttr(self._config.model_name)
        UsdUrdf.UrdfRobot.Define(self.world_builder.stage, "/urdf/robot")
        UsdUrdf.UrdfMaterial.Define(self.world_builder.stage, "/urdf/robot/materials")

    def _import_body_and_joint(self, urdf_link_name) -> None:
        if urdf_link_name not in self.urdf_model.child_map:
            return

        for child_joint_name, child_urdf_link_name in self.urdf_model.child_map[urdf_link_name]:
            child_urdf_joint: urdf.Joint = self.urdf_model.joint_map[child_joint_name]

            body_builder = self._import_body(body_name=urdf_link_name,
                                             child_body_name=child_urdf_link_name,
                                             joint=child_urdf_joint)

            self._import_geoms(link=self.urdf_model.link_map[child_urdf_link_name], body_builder=body_builder)

            self._import_inertial(body=self.urdf_model.link_map[child_urdf_link_name], body_builder=body_builder)

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
        else:
            body_builder = self.world_builder.add_body(body_name=child_body_name,
                                                       parent_body_name=body_name)

        relative_to_body_builder = self.world_builder.get_body_builder(body_name=body_name)
        relative_to_xform = relative_to_body_builder.xform
        body_builder.set_transform(pos=joint_pos, quat=joint_quat, relative_to_xform=relative_to_xform)

        UsdUrdf.UrdfLinkAPI.Apply(body_builder.xform.GetPrim())

        return body_builder

    def _import_inertial(self, body: urdf.Link, body_builder: BodyBuilder) -> None:
        if self._config.with_physics and not (
                self._config.fixed_base and body.name == self._config.model_name):
            if self._config.inertia_source == InertiaSource.FROM_SRC:
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
                    physics_mass_api = body_builder.set_inertial(mass=body_mass,
                                                                 center_of_mass=body_center_of_mass,
                                                                 diagonal_inertia=body_diagonal_inertia,
                                                                 principal_axes=body_principal_axes)
                else:
                    _, physics_mass_api = body_builder.compute_and_set_inertial(
                        inertia_source=self._config.inertia_source)

            else:
                _, physics_mass_api = body_builder.compute_and_set_inertial(inertia_source=self._config.inertia_source)

            build_urdf_inertial_api(physics_mass_api=physics_mass_api)

    def _import_geoms(self, link: urdf.Link, body_builder: BodyBuilder) -> None:
        geom_name = f"{link.name}_geom"
        if self._config.with_visual:
            for i, visual in enumerate(link.visuals):
                visual_geom_name = f"{geom_name}_visual_{i}"
                self._import_geom(geom_name=visual_geom_name, geom=visual, body_builder=body_builder)
        if self._config.with_collision:
            for i, collision in enumerate(link.collisions):
                collision_geom_name = f"{geom_name}_collision_{i}"
                self._import_geom(geom_name=collision_geom_name, geom=collision, body_builder=body_builder)

    def _import_geom(self,
                     geom_name: str,
                     geom: Union[urdf.Visual, urdf.Collision],
                     body_builder: BodyBuilder) -> None:
        if geom.origin is not None:
            geom_pos = geom.origin.xyz
            geom_rpy = geom.origin.rpy
        else:
            geom_pos = numpy.array([0.0, 0.0, 0.0])
            geom_rpy = numpy.array([0.0, 0.0, 0.0])
        geom_quat = Rotation.from_euler('xyz', geom_rpy).as_quat()

        geom_is_visible = isinstance(geom, urdf.Visual)
        geom_is_collidable = isinstance(geom, urdf.Collision)
        geom_rgba = self._config.default_rgba if (not hasattr(geom, "material") or
                                                  not hasattr(geom.material, "color") or
                                                  geom.material.color is None) \
            else geom.material.color.rgba
        geom_type = self._geom_type_map[type(geom.geometry)]
        geom_density = 1000.0
        geom_property = GeomProperty(geom_type=geom_type,
                                     is_visible=geom_is_visible,
                                     is_collidable=geom_is_collidable,
                                     rgba=geom_rgba,
                                     density=geom_density)

        if not type(geom.geometry) is urdf.Mesh:
            geom_builder = body_builder.add_geom(geom_name=geom_name, geom_property=geom_property)
            geom_builder.build()

            gprim_prim = geom_builder.gprim.GetPrim()
            get_urdf_link_api(geom=geom, gprim_prim=gprim_prim)

            if type(geom.geometry) is urdf.Box:
                geom_scale = numpy.array([geom.geometry.size[i] / 2.0 for i in range(3)])
                geom_builder.set_transform(pos=geom_pos, quat=geom_quat, scale=geom_scale)

                urdf_geometry_box_api = UsdUrdf.UrdfGeometryBoxAPI.Apply(gprim_prim)
                urdf_geometry_box_api.CreateSizeAttr(Gf.Vec3f(*geom.geometry.size))
            elif type(geom.geometry) is urdf.Sphere:
                geom_builder.set_transform(pos=geom_pos, quat=geom_quat)
                geom_builder.set_attribute(radius=geom.geometry.radius)

                urdf_geometry_sphere_api = UsdUrdf.UrdfGeometrySphereAPI.Apply(gprim_prim)
                urdf_geometry_sphere_api.CreateRadiusAttr(geom.geometry.radius)
            elif type(geom.geometry) is urdf.Cylinder:
                geom_builder.set_transform(pos=geom_pos, quat=geom_quat)
                geom_builder.set_attribute(radius=geom.geometry.radius, height=geom.geometry.length)

                urdf_geometry_cylinder_api = UsdUrdf.UrdfGeometryCylinderAPI.Apply(gprim_prim)
                urdf_geometry_cylinder_api.CreateRadiusAttr(geom.geometry.radius)
                urdf_geometry_cylinder_api.CreateLengthAttr(geom.geometry.length)
            else:
                raise ValueError(f"Geom type {type(geom.geometry)} not implemented.")
        else:
            source_mesh_file_path = self.get_mesh_file_path(urdf_mesh_file_path=geom.geometry.filename)
            if source_mesh_file_path is not None:
                tmp_usd_mesh_file_path, tmp_origin_mesh_file_path = self.import_mesh(
                    mesh_file_path=source_mesh_file_path, merge_mesh=True)
                mesh_stage = Usd.Stage.Open(tmp_usd_mesh_file_path)
                for mesh_prim in [prim for prim in mesh_stage.Traverse() if prim.IsA(UsdGeom.Mesh)]:
                    mesh_name = mesh_prim.GetName()
                    mesh_path = mesh_prim.GetPath()
                    mesh_property = MeshProperty.from_mesh_file_path(mesh_file_path=tmp_usd_mesh_file_path,
                                                                     mesh_path=mesh_path)
                    if mesh_property.face_vertex_counts.size == 0 or mesh_property.face_vertex_indices.size == 0:
                        # TODO: Fix empty mesh
                        continue

                    geom_builder = body_builder.add_geom(geom_name=f"{geom_name}_{mesh_name}",
                                                         geom_property=geom_property)

                    geom_builder.add_mesh(mesh_name=mesh_name,
                                          mesh_property=mesh_property)
                    geom_builder.build()
                    geom_scale = numpy.array([1.0, 1.0, 1.0]) if geom.geometry.scale is None else geom.geometry.scale
                    geom_builder.set_transform(pos=geom_pos, quat=geom_quat, scale=geom_scale)

                    gprim_prim = geom_builder.gprim.GetPrim()
                    get_urdf_link_api(geom=geom, gprim_prim=gprim_prim)

                    urdf_geometry_mesh_api = UsdUrdf.UrdfGeometryMeshAPI.Apply(gprim_prim)
                    urdf_geometry_mesh_api.CreateFilenameAttr(f"./{tmp_origin_mesh_file_path}")
                    if geom.geometry.scale is not None:
                        urdf_geometry_mesh_api.CreateScaleAttr(Gf.Vec3f(*geom.geometry.scale))

                    if mesh_prim.HasAPI(UsdShade.MaterialBindingAPI):
                        material_binding_api = UsdShade.MaterialBindingAPI(mesh_prim)
                        material_paths = material_binding_api.GetDirectBindingRel().GetTargets()
                        if len(material_paths) > 1:
                            raise NotImplementedError(f"Mesh {mesh_name} has more than one material.")
                        material_path = material_paths[0]
                        material_property = MaterialProperty.from_material_file_path(
                            material_file_path=tmp_usd_mesh_file_path,
                            material_path=material_path)
                        material_builder = geom_builder.add_material(material_name=material_path.name,
                                                                     material_property=material_property)

                        stage = self.world_builder.stage
                        urdf_material_path = self.urdf_materials_prim.GetPath().AppendChild(material_path.name)
                        if not stage.GetPrimAtPath(urdf_material_path).IsValid():
                            urdf_material = UsdUrdf.UrdfMaterial.Define(stage, urdf_material_path)

                            if isinstance(material_builder.diffuse_color, str):
                                urdf_material.CreateTextureAttr(f"./{material_builder.diffuse_color}")
                            elif isinstance(material_builder.diffuse_color, numpy.ndarray):
                                rgba = Gf.Vec4f(*material_builder.diffuse_color.tolist(), material_builder.opacity)
                                urdf_material.CreateRgbaAttr(rgba)
                            else:
                                raise ValueError(f"Diffuse color {material_builder.diffuse_color} not supported.")

                        urdf_link_visual_api = UsdUrdf.UrdfLinkVisualAPI(gprim_prim)
                        urdf_link_visual_api.CreateMaterialRel().SetTargets([urdf_material_path])

                    for child_prim in [prim for prim in mesh_prim.GetChildren() if prim.IsA(UsdGeom.Subset)]:
                        material_binding_api = UsdShade.MaterialBindingAPI(child_prim)
                        material_paths = material_binding_api.GetDirectBindingRel().GetTargets()
                        if len(material_paths) > 1:
                            raise NotImplementedError(f"Mesh {mesh_name} has more than one material.")
                        material_path = material_paths[0]
                        material_property = MaterialProperty.from_material_file_path(
                            material_file_path=tmp_usd_mesh_file_path,
                            material_path=material_path)
                        geom_builder.add_material(material_name=material_path.name,
                                                  material_property=material_property,
                                                  subset=UsdGeom.Subset(child_prim))

    def get_mesh_file_path(self, urdf_mesh_file_path: str) -> Optional[str]:
        mesh_file_path = None
        if urdf_mesh_file_path.find("package://") != -1:
            import rospkg
            urdf_mesh_file_path = urdf_mesh_file_path.replace("package://", "")
            package_name = urdf_mesh_file_path.split("/", 2)[0]
            try:
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
        joint_type = JointType.from_urdf(joint.type)

        joint_builder = None
        if joint_type != JointType.FIXED:
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
            joint_pos = Gf.Vec3d(*joint_pos)
            joint_pos = body1_rot.GetInverse().Transform(joint_pos - body1_to_body2_pos)

            joint_property = JointProperty(
                joint_parent_prim=parent_body_builder.xform.GetPrim(),
                joint_child_prim=child_body_builder.xform.GetPrim(),
                joint_pos=joint_pos,
                joint_axis=JointAxis.from_array(joint.axis),
                joint_type=joint_type,
            )
            joint_builder = child_body_builder.add_joint(joint_name=joint.name, joint_property=joint_property)

            if joint_type == JointType.REVOLUTE:
                joint_builder.set_limit(lower=degrees(joint.limit.lower),
                                        upper=degrees(joint.limit.upper))
            elif joint_type == JointType.PRISMATIC:
                joint_builder.set_limit(lower=joint.limit.lower,
                                        upper=joint.limit.upper)

            urdf_joint_api = UsdUrdf.UrdfJointAPI.Apply(joint_builder.joint.GetPrim())
            urdf_joint_api.CreateTypeAttr(joint.type)
            if joint.origin is not None:
                urdf_joint_api.CreateXyzAttr(Gf.Vec3f(*joint.origin.xyz))
                urdf_joint_api.CreateRpyAttr(Gf.Vec3f(*joint.origin.rpy))
            urdf_joint_api.CreateParentRel().AddTarget(parent_prim.GetPath())
            urdf_joint_api.CreateChildRel().AddTarget(child_prim.GetPath())
            if joint_type in [JointType.REVOLUTE, JointType.CONTINUOUS, JointType.PRISMATIC]:
                urdf_joint_api.CreateAxisAttr(Gf.Vec3f(*joint.axis))

            if joint_type in [JointType.REVOLUTE, JointType.PRISMATIC]:
                urdf_joint_api.CreateLowerAttr(joint.limit.lower)
                urdf_joint_api.CreateUpperAttr(joint.limit.upper)
                urdf_joint_api.CreateEffortAttr(joint.limit.effort)
                urdf_joint_api.CreateVelocityAttr(joint.limit.velocity)

        return joint_builder

    @property
    def urdf_model(self) -> urdf.URDF:
        return self._urdf_model

    @property
    def geom_type_map(self) -> Dict:
        return self._geom_type_map

    @property
    def urdf_prim(self) -> Usd.Prim:
        return self.world_builder.stage.GetPrimAtPath("/urdf")

    @property
    def urdf_robot_prim(self) -> Usd.Prim:
        return self.urdf_prim.GetChild("robot")

    @property
    def urdf_materials_prim(self) -> Usd.Prim:
        return self.urdf_robot_prim.GetChild("materials")
