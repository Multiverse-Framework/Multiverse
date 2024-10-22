#!/usr/bin/env python3

import os.path
from math import degrees
from typing import Optional, List, Tuple, Dict

import numpy
import mujoco
import xml.etree.ElementTree as ET

from ..utils import modify_name
from ..factory import Factory, Configuration, InertiaSource
from ..factory import (WorldBuilder, BodyBuilder,
                       JointBuilder, JointType, JointProperty, get_joint_axis_and_quat,
                       GeomBuilder, GeomType, GeomProperty,
                       PointsBuilder, PointProperty,
                       MeshProperty,
                       MaterialProperty,
                       TextureBuilder)

from pxr import UsdMujoco, Gf, Usd


def get_model_name(xml_file_path: str) -> str:
    with open(xml_file_path) as xml_file:
        for line in xml_file:
            if "<mujoco model=" in line:
                return line.split('"')[1]
    return os.path.basename(xml_file_path).split(".")[0]


def get_body_name(mj_body) -> str:
    return mj_body.name if mj_body.name is not None else "Body_" + str(mj_body.id)


def get_bodies_with_composite(xml_file_path: str) -> Dict[str, List[ET.Element]]:
    bodies_with_composite = {}
    tree = ET.parse(xml_file_path)
    root = tree.getroot()
    for include in root.iter("include"):
        include_file_path = include.attrib["file"]
        if os.path.relpath(include_file_path):
            include_file_path = os.path.join(os.path.dirname(xml_file_path), include_file_path)
        bodies_with_composite.update(get_bodies_with_composite(include_file_path))

    for body in root.iter("body"):
        body_name = body.attrib["name"]
        for child in body:
            if child.tag == "composite":
                if body_name in bodies_with_composite:
                    bodies_with_composite[body_name].append(child)
                else:
                    bodies_with_composite[body_name] = [child]

    return bodies_with_composite


class MjcfImporter(Factory):
    mj_model: mujoco.MjModel
    _geom_type_map: Dict = {
        mujoco.mjtGeom.mjGEOM_PLANE: GeomType.PLANE,
        mujoco.mjtGeom.mjGEOM_BOX: GeomType.CUBE,
        mujoco.mjtGeom.mjGEOM_SPHERE: GeomType.SPHERE,
        mujoco.mjtGeom.mjGEOM_ELLIPSOID: GeomType.SPHERE,
        mujoco.mjtGeom.mjGEOM_CYLINDER: GeomType.CYLINDER,
        mujoco.mjtGeom.mjGEOM_CAPSULE: GeomType.CAPSULE,
        mujoco.mjtGeom.mjGEOM_MESH: GeomType.MESH,
    }

    def __init__(
            self,
            file_path: str,
            fixed_base: bool,
            with_physics: bool,
            with_visual: bool,
            with_collision: bool,
            inertia_source: InertiaSource = InertiaSource.FROM_SRC,
            default_rgba: Optional[numpy.ndarray] = None
    ) -> None:
        self._joint_builders: Dict[int, JointBuilder] = {}
        try:
            self._mj_model = mujoco.MjModel.from_xml_path(filename=file_path)
        except ValueError as e:
            log_file = "MUJOCO_LOG.TXT"
            if os.path.exists(log_file):
                print(f"Removing log file {log_file}...")
                os.remove(log_file)
            raise FileNotFoundError(f"{e}")
        model_name = get_model_name(xml_file_path=file_path)
        super().__init__(file_path=file_path, config=Configuration(
            model_name=model_name,
            fixed_base=fixed_base,
            with_physics=with_physics,
            with_visual=with_visual,
            with_collision=with_collision,
            default_rgba=default_rgba if default_rgba is not None else numpy.array([0.9, 0.9, 0.9, 1.0]),
            inertia_source=inertia_source
        ))
        self._bodies_with_composite = get_bodies_with_composite(self.source_file_path)

    def import_model(self, save_file_path: Optional[str] = None) -> str:
        self._world_builder = WorldBuilder(self.tmp_usd_file_path)

        self._import_config()

        self.world_builder.add_body(body_name=self._config.model_name)

        for body_id in range(1, self.mj_model.nbody):
            mj_body = self.mj_model.body(body_id)

            parent_body_id = mj_body.parentid[0]
            parent_body = self.mj_model.body(parent_body_id)
            parent_body_name = get_body_name(parent_body)
            if parent_body_name in self.bodies_with_composite:
                self._import_point(parent_body_name=parent_body_name, mj_body=mj_body)
            else:
                body_builder = self._import_body(mj_body=mj_body)
                self._import_geoms(mj_body=mj_body, body_builder=body_builder)
                if self._config.with_physics:
                    self._import_joints(mj_body=mj_body, body_builder=body_builder)

                self._import_inertial(mj_body=mj_body, body_builder=body_builder)

        for body_builder in self.world_builder.body_builders:
            self._import_points(body_builder=body_builder)

        self._import_equality()

        self.world_builder.export()

        return self.tmp_usd_file_path if save_file_path is None else self.save_tmp_model(usd_file_path=save_file_path)

    def _import_config(self):
        usd_mujoco = UsdMujoco.Mujoco.Define(self.world_builder.stage, "/mujoco")
        usd_mujoco.CreateModelAttr(self._config.model_name)

        mujoco_option_api = UsdMujoco.MujocoOptionAPI.Apply(usd_mujoco.GetPrim())
        mujoco_option_api.CreateTimeStepAttr(self.mj_model.opt.timestep)

        UsdMujoco.MujocoAsset.Define(self.world_builder.stage, "/mujoco/asset")
        UsdMujoco.MujocoMesh.Define(self.world_builder.stage, "/mujoco/asset/meshes")
        UsdMujoco.MujocoMaterial.Define(self.world_builder.stage, "/mujoco/asset/materials")
        UsdMujoco.MujocoTexture.Define(self.world_builder.stage, "/mujoco/asset/textures")

    def _import_body(self, mj_body) -> BodyBuilder:
        body_name = mj_body.name if mj_body.name is not None else "Body_" + str(mj_body.id)

        parent_mj_body = self.mj_model.body(mj_body.parentid)
        if parent_mj_body.id == 0:
            body_builder = self.world_builder.add_body(body_name=body_name,
                                                       parent_body_name=self._config.model_name,
                                                       body_id=mj_body.id)
        else:
            parent_body_name = get_body_name(parent_mj_body)
            if self._config.with_physics and mj_body.jntnum[0] > 0:
                body_builder = self.world_builder.add_body(body_name=body_name,
                                                           parent_body_name=self._config.model_name,
                                                           body_id=mj_body.id)
            else:
                body_builder = self.world_builder.add_body(body_name=body_name,
                                                           parent_body_name=parent_body_name,
                                                           body_id=mj_body.id)

            relative_to_body_builder = self.world_builder.get_body_builder(body_name=parent_body_name)
            relative_to_xform = relative_to_body_builder.xform
            body_pos = mj_body.pos
            body_quat = numpy.array([mj_body.quat[1],
                                     mj_body.quat[2],
                                     mj_body.quat[3],
                                     mj_body.quat[0]])
            body_builder.set_transform(
                pos=body_pos,
                quat=body_quat,
                relative_to_xform=relative_to_xform,
            )

            mujoco_body_api = UsdMujoco.MujocoBodyAPI.Apply(body_builder.xform.GetPrim())
            mujoco_body_api.CreatePosAttr(Gf.Vec3f(*body_pos))
            mujoco_body_api.CreateQuatAttr(Gf.Quatf(body_quat[3], *body_quat[:3]))

        return body_builder

    def _import_point(self, parent_body_name, mj_body):
        parent_body_id = mj_body.parentid
        parent_body_builder = self.world_builder.get_body_builder(body_name=parent_body_name)
        point_id = mj_body.id
        geom_id = mj_body.geomadr[0]
        geom = self.mj_model.geom(geom_id)
        point_width = 2 * geom.size[0]
        points_rgba = geom.rgba
        point_pos = mj_body.pos
        point_property = PointProperty(point_id=point_id,
                                       point_pos=point_pos,
                                       point_width=point_width)
        point_adr = parent_body_id
        for points_element in self.bodies_with_composite[parent_body_name]:
            points_count = [int(x) for x in points_element.attrib["count"].split(" ") if x != ""]
            points_num = numpy.prod(points_count)
            if point_adr < point_id <= point_adr + points_num:
                points_prefix = points_element.attrib["prefix"] if "prefix" in points_element.attrib else ""
                points_type = points_element.attrib["type"]
                points_name = f"{parent_body_name}_{points_prefix}_{points_type}s"
                parent_body_builder.add_point(points_name=points_name,
                                              point_property=point_property,
                                              points_rgba=points_rgba)
            point_adr += points_num

    def _import_points(self, body_builder: BodyBuilder):
        parent_body_name = body_builder.xform.GetPrim().GetName()
        if parent_body_name in self.bodies_with_composite:
            for points_builder, points_element in zip(body_builder.points_builders, self.bodies_with_composite[parent_body_name]):
                points_builder.build()

                points_type = points_element.attrib["type"]
                points_count = [int(x) for x in points_element.attrib["count"].split(" ") if x != ""]
                points_spacing = float(points_element.attrib["spacing"])
                points_offset = [float(x) for x in points_element.attrib["offset"].split(" ") if x != ""]
                points_prefix = points_element.attrib["prefix"] if "prefix" in points_element.attrib else ""

                mujoco_composite_api = UsdMujoco.MujocoCompositeAPI.Apply(points_builder.points.GetPrim())
                mujoco_composite_api.CreateTypeAttr(points_type)
                mujoco_composite_api.CreateCountAttr(Gf.Vec3i(*points_count))
                mujoco_composite_api.CreateSpacingAttr(points_spacing)
                mujoco_composite_api.CreateOffsetAttr(Gf.Vec3f(*points_offset))
                mujoco_composite_api.CreatePrefixAttr(points_prefix)

    def _import_joints(self, mj_body, body_builder: BodyBuilder):
        for joint_id in range(mj_body.jntadr[0], mj_body.jntadr[0] + mj_body.jntnum[0]):
            joint_builder = self._import_joint(mj_body, body_builder, joint_id)
            if joint_builder is not None:
                self._joint_builders[joint_id] = joint_builder

    def _import_joint(self, mj_body, body_builder: BodyBuilder, joint_id: int) -> Optional[JointBuilder]:
        mj_joint = self.mj_model.joint(joint_id)
        if mj_joint.type == mujoco.mjtJoint.mjJNT_FREE:
            return None

        joint_name = mj_joint.name if mj_joint.name is not None else "Joint_" + str(joint_id)
        joint_type = JointType.from_mujoco(jnt_type=mj_joint.type)

        parent_body_id = mj_body.parentid
        parent_body_name = get_body_name(self.mj_model.body(parent_body_id))
        parent_body_builder = self.world_builder.get_body_builder(body_name=parent_body_name)
        joint_axis, joint_quat = get_joint_axis_and_quat(joint_axis=mj_joint.axis)
        joint_property = JointProperty(
            joint_parent_prim=parent_body_builder.xform.GetPrim(),
            joint_child_prim=body_builder.xform.GetPrim(),
            joint_pos=mj_joint.pos,
            joint_quat=joint_quat,
            joint_axis=joint_axis,
            joint_type=joint_type,
        )
        joint_builder = body_builder.add_joint(joint_name=joint_name, joint_property=joint_property)

        if mj_joint.type == mujoco.mjtJoint.mjJNT_HINGE:
            joint_builder.set_limit(lower=degrees(mj_joint.range[0]),
                                    upper=degrees(mj_joint.range[1]))
        elif mj_joint.type == mujoco.mjtJoint.mjJNT_SLIDE:
            joint_builder.set_limit(lower=mj_joint.range[0],
                                    upper=mj_joint.range[1])

        mujoco_joint_api = UsdMujoco.MujocoJointAPI.Apply(joint_builder.joint.GetPrim())
        mj_joint_type = "hinge" if mj_joint.type == mujoco.mjtJoint.mjJNT_HINGE \
            else "slide" if mj_joint.type == mujoco.mjtJoint.mjJNT_SLIDE \
            else "ball" if mj_joint.type == mujoco.mjtJoint.mjJNT_BALL \
            else None
        if mj_joint_type is None:
            raise NotImplementedError(f"Joint type {mj_joint.type} not supported.")

        mujoco_joint_api.CreateTypeAttr(mj_joint_type)
        mujoco_joint_api.CreatePosAttr(Gf.Vec3f(*mj_joint.pos))
        mujoco_joint_api.CreateAxisAttr(Gf.Vec3f(*mj_joint.axis))
        if mj_joint.type in [mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE]:
            mujoco_joint_api.CreateRangeAttr(Gf.Vec2f(*mj_joint.range))

        return joint_builder

    def _import_geoms(self, mj_body, body_builder: BodyBuilder) -> List[GeomBuilder]:
        geom_builders = []
        for geom_id in range(mj_body.geomadr[0], mj_body.geomadr[0] + mj_body.geomnum[0]):
            geom_builder = self._import_geom(body_builder, geom_id)
            if geom_builder is not None:
                geom_builders.append(geom_builder)
        return geom_builders

    def _import_inertial(self, mj_body, body_builder):
        if self._config.with_physics and not (
                self._config.fixed_base and mj_body.id == 1):
            if self._config.inertia_source == InertiaSource.FROM_SRC:
                body_mass = mj_body.mass[0]
                body_center_of_mass = mj_body.ipos
                body_diagonal_inertia = mj_body.inertia
                body_principal_axes = numpy.array([mj_body.iquat[1],
                                                   mj_body.iquat[2],
                                                   mj_body.iquat[3],
                                                   mj_body.iquat[0]])
                body_builder.set_inertial(mass=body_mass,
                                          center_of_mass=body_center_of_mass,
                                          diagonal_inertia=body_diagonal_inertia,
                                          principal_axes=body_principal_axes)
            else:
                _, physics_mass_api = body_builder.compute_and_set_inertial(inertia_source=self._config.inertia_source)

    def _import_geom(self, body_builder: BodyBuilder, geom_id: int) -> Optional[GeomBuilder]:
        mj_geom = self.mj_model.geom(geom_id)
        geom_is_visible = (mj_geom.contype == 0) and (mj_geom.conaffinity == 0)
        geom_is_collidable = (mj_geom.contype != 0) or (mj_geom.conaffinity != 0)
        geom_builder = None
        geom_pos = mj_geom.pos
        geom_quat = numpy.array([mj_geom.quat[1],
                                 mj_geom.quat[2],
                                 mj_geom.quat[3],
                                 mj_geom.quat[0]])

        if geom_is_visible and self._config.with_visual or geom_is_collidable and self._config.with_collision:
            geom_name = mj_geom.name if mj_geom.name != "" else "Geom_" + str(geom_id)
            geom_rgba = mj_geom.rgba
            geom_type = self._geom_type_map[mj_geom.type[0]]
            geom_density = 1000.0
            geom_property = GeomProperty(geom_type=geom_type,
                                         is_visible=geom_is_visible,
                                         is_collidable=geom_is_collidable,
                                         rgba=geom_rgba,
                                         density=geom_density)
            geom_builder = body_builder.add_geom(geom_name=geom_name, geom_property=geom_property)
            geom_builder.build()

            gprim_prim = geom_builder.gprim.GetPrim()
            mujoco_geom_api = UsdMujoco.MujocoGeomAPI.Apply(gprim_prim)
            mj_geom_type_str = "plane" if mj_geom.type == mujoco.mjtGeom.mjGEOM_PLANE \
                else "box" if mj_geom.type == mujoco.mjtGeom.mjGEOM_BOX \
                else "sphere" if mj_geom.type == mujoco.mjtGeom.mjGEOM_SPHERE \
                else "ellipsoid" if mj_geom.type == mujoco.mjtGeom.mjGEOM_ELLIPSOID \
                else "cylinder" if mj_geom.type == mujoco.mjtGeom.mjGEOM_CYLINDER \
                else "capsule" if mj_geom.type == mujoco.mjtGeom.mjGEOM_CAPSULE \
                else "mesh" if mj_geom.type == mujoco.mjtGeom.mjGEOM_MESH \
                else None
            if mj_geom_type_str is None:
                raise NotImplementedError(f"Geom type {mj_geom.type} not supported.")
            mujoco_geom_api.CreateTypeAttr(mj_geom_type_str)
            geom_size = mj_geom.size if mj_geom.type != mujoco.mjtGeom.mjGEOM_MESH else numpy.array([1.0, 1.0, 1.0])
            mujoco_geom_api.CreateSizeAttr(Gf.Vec3f(*geom_size))
            mujoco_geom_api.CreatePosAttr(Gf.Vec3f(*geom_pos))
            mujoco_geom_api.CreateQuatAttr(Gf.Quatf(geom_quat[3], *geom_quat[:3]))

            if mj_geom.type in [mujoco.mjtGeom.mjGEOM_PLANE]:
                geom_builder.set_transform(pos=geom_pos, quat=geom_quat, scale=numpy.array([50, 50, 1]))
            elif mj_geom.type in [mujoco.mjtGeom.mjGEOM_BOX]:
                geom_builder.set_transform(pos=geom_pos, quat=geom_quat, scale=mj_geom.size)
            elif mj_geom.type in [mujoco.mjtGeom.mjGEOM_SPHERE, mujoco.mjtGeom.mjGEOM_ELLIPSOID]:
                # TODO: Fix ellipsoid
                geom_builder.set_transform(pos=geom_pos, quat=geom_quat)
                geom_builder.set_attribute(radius=mj_geom.size[0])
            elif mj_geom.type in [mujoco.mjtGeom.mjGEOM_CYLINDER]:
                geom_builder.set_transform(pos=geom_pos, quat=geom_quat)
                geom_builder.set_attribute(radius=mj_geom.size[0], height=mj_geom.size[1] * 2)
            elif mj_geom.type in [mujoco.mjtGeom.mjGEOM_CAPSULE]:
                # TODO: Fix capsule
                geom_builder.set_transform(pos=geom_pos, quat=geom_quat)
                geom_builder.set_attribute(radius=mj_geom.size[0], height=mj_geom.size[1] * 2)
            elif mj_geom.type in [mujoco.mjtGeom.mjGEOM_MESH]:
                mesh_id = mj_geom.dataid[0]
                mesh_name = self.mj_model.mesh(mesh_id).name
                points, normals, face_vertex_counts, face_vertex_indices = self._get_mesh_data(mesh_id=mesh_id)
                tmp_usd_mesh_file_path = os.path.join(self.tmp_mesh_dir_path,
                                                      "usd",
                                                      f"{mesh_name}.usda")

                mat_id = mj_geom.matid[0]
                if (mat_id == -1 or
                        self.mj_model.mat_texid[mat_id][mujoco.mjtTextureRole.mjTEXROLE_RGB] == -1 or
                        self.mj_model.mesh_texcoordadr[mesh_id] == -1):
                    file_ext = "stl"
                    texture_coordinates = None
                    texture_file_path = None
                else:
                    texture_id = self.mj_model.mat_texid[mat_id][mujoco.mjtTextureRole.mjTEXROLE_RGB]
                    file_ext = "obj"
                    mesh_texcoordadr = self.mj_model.mesh_texcoordadr[mesh_id]
                    mesh_texcoordnum = self.mj_model.mesh_texcoordnum[mesh_id]
                    mesh_texcoord = self.mj_model.mesh_texcoord[mesh_texcoordadr:mesh_texcoordadr + mesh_texcoordnum]
                    mesh_texcoord = numpy.array([[x, 1 - y] for x, y in mesh_texcoord])
                    mesh_faceadr = self.mj_model.mesh_faceadr[mesh_id]
                    mesh_facenum = self.mj_model.mesh_facenum[mesh_id]
                    mesh_facetexcoord = self.mj_model.mesh_facetexcoord[mesh_faceadr:mesh_faceadr + mesh_facenum]

                    texture_coordinates = mesh_texcoord[mesh_facetexcoord].reshape(-1, 2)

                    texture_name = self.mj_model.tex(texture_id).name
                    if texture_name == "":
                        texture_name = "Texture_" + str(texture_id)
                    width = self.mj_model.tex_width[texture_id]
                    height = self.mj_model.tex_height[texture_id]
                    texture_adr = self.mj_model.tex_adr[texture_id]
                    texture_num = width * height * 3
                    rgb = self.mj_model.tex_data[texture_adr:texture_adr + texture_num]
                    rgb = rgb.reshape((height, width, 3))
                    texture_file_path = os.path.join(self._tmp_texture_dir_path, f"{texture_name}.png")
                    texture_builder = TextureBuilder(file_path=texture_file_path)
                    texture_builder.rgb = rgb

                mesh_property = MeshProperty(points=points,
                                             normals=normals,
                                             face_vertex_counts=face_vertex_counts,
                                             face_vertex_indices=face_vertex_indices,
                                             texture_coordinates=texture_coordinates,
                                             mesh_file_name=mesh_name)
                mesh_builder = geom_builder.add_mesh(mesh_name=mesh_name, mesh_property=mesh_property)
                mesh_name = mesh_builder.mesh.GetPrim().GetPath().name

                if mat_id != -1:
                    material_name = self.mj_model.mat(mat_id).name
                    material_name = modify_name(material_name, "Material_")

                    mujoco_material_path = self.mujoco_materials_prim.GetPath().AppendChild(material_name)
                    mujoco_material = UsdMujoco.MujocoMaterial.Define(self.world_builder.stage,
                                                                      mujoco_material_path)
                    mujoco_geom_api.CreateMaterialRel().SetTargets([mujoco_material_path])

                    texture_id = self.mj_model.mat_texid[mat_id][mujoco.mjtTextureRole.mjTEXROLE_RGB]
                    if texture_id == -1 or texture_file_path is None:
                        mat_rgba = self.mj_model.mat_rgba[mat_id]
                        mat_emission = float(self.mj_model.mat_emission[mat_id])
                        mat_specular = float(self.mj_model.mat_specular[mat_id])
                        (diffuse_color,
                         opacity,
                         emissive_color,
                         specular_color) = self._get_material_data(mat_rgba=mat_rgba,
                                                                   mat_emission=mat_emission,
                                                                   mat_specular=mat_specular)

                        mujoco_material.CreateRgbaAttr(Gf.Vec4f(*mat_rgba.tolist()))
                        mujoco_material.CreateEmissionAttr(mat_emission)
                        mujoco_material.CreateSpecularAttr(mat_specular)
                    else:
                        diffuse_color = texture_file_path
                        opacity = None
                        emissive_color = None
                        specular_color = None

                        texture_name = os.path.splitext(os.path.basename(texture_file_path))[0]
                        mujoco_texture_path = self.mujoco_textures_prim.GetPath().AppendChild(texture_name)
                        mujoco_material.CreateTextureRel().SetTargets([mujoco_texture_path])

                        mujoco_texture = UsdMujoco.MujocoTexture.Define(self.world_builder.stage,
                                                                        mujoco_texture_path)
                        texture_type = self.mj_model.tex_type[texture_id]
                        if texture_type == mujoco.mjtTexture.mjTEXTURE_2D:
                            mujoco_texture.CreateTypeAttr("2d")
                        elif texture_type == mujoco.mjtTexture.mjTEXTURE_CUBE:
                            mujoco_texture.CreateTypeAttr("cube")
                        elif texture_type == mujoco.mjtTexture.mjTEXTURE_SKYBOX:
                            mujoco_texture.CreateTypeAttr("skybox")
                        else:
                            raise NotImplementedError(f"Texture type {texture_type} not supported.")

                        mujoco_texture.CreateFileAttr(f"{texture_name}.png")

                    material_property = MaterialProperty(diffuse_color=diffuse_color,
                                                         opacity=opacity,
                                                         emissive_color=emissive_color,
                                                         specular_color=specular_color)
                    geom_builder.add_material(material_name=material_name,
                                              material_property=material_property)

                tmp_mesh_file_path = os.path.join(self.tmp_mesh_dir_path,
                                                  file_ext,
                                                  f"{mesh_name}.{file_ext}")
                self.export_mesh(in_mesh_file_path=tmp_usd_mesh_file_path,
                                 out_mesh_file_path=tmp_mesh_file_path)

                mujoco_mesh_path = self.mujoco_meshes_prim.GetPath().AppendChild(mesh_name)
                mujoco_mesh = UsdMujoco.MujocoMesh.Define(self.world_builder.stage, mujoco_mesh_path)
                mujoco_mesh.CreateFileAttr(f"./{tmp_mesh_file_path}")

                mujoco_geom_api.CreateMeshRel().SetTargets([mujoco_mesh_path])

                geom_builder.build()
                geom_builder.set_transform(pos=geom_pos, quat=geom_quat)
            else:
                raise NotImplementedError(f"Geom type {mj_geom.type} not supported.")

        return geom_builder

    def _get_mesh_data(self, mesh_id: int) -> Tuple[numpy.ndarray, numpy.ndarray, numpy.ndarray, numpy.ndarray]:
        vert_adr = self.mj_model.mesh_vertadr[mesh_id]
        vert_num = self.mj_model.mesh_vertnum[mesh_id]
        points = self.mj_model.mesh_vert[vert_adr:vert_adr + vert_num]

        normal_adr = self.mj_model.mesh_normaladr[mesh_id]
        normal_num = self.mj_model.mesh_normalnum[mesh_id]
        mesh_normal = self.mj_model.mesh_normal[normal_adr:normal_adr + normal_num]

        face_adr = self.mj_model.mesh_faceadr[mesh_id]
        face_num = self.mj_model.mesh_facenum[mesh_id]
        mesh_facenormal = self.mj_model.mesh_facenormal[face_adr:face_adr + face_num]

        normals = mesh_normal[mesh_facenormal].reshape(-1, 3)

        face_vertex_counts = numpy.empty(shape=self.mj_model.mesh_facenum[mesh_id], dtype=float)
        face_vertex_counts.fill(3)

        face_vertex_indices = self.mj_model.mesh_face[face_adr:face_adr + face_num]

        return points, normals, face_vertex_counts, face_vertex_indices

    def _get_material_data(self, mat_rgba, mat_emission, mat_specular) \
            -> Tuple[numpy.ndarray, float, numpy.ndarray, numpy.ndarray]:
        diffuse_color = numpy.array([float(x) for x in mat_rgba[:3]])
        opacity = float(mat_rgba[3])
        emissive_color = numpy.array([float(x * mat_emission) for x in mat_rgba[:3]])
        specular_color = numpy.array([float(mat_specular) for _ in range(3)])

        return diffuse_color, opacity, emissive_color, specular_color

    def _import_equality(self):
        equality_prim = UsdMujoco.MujocoEquality.Define(self.world_builder.stage, "/mujoco/equality")
        for equality_id in range(self.mj_model.neq):
            equality = self.mj_model.equality(equality_id)
            if equality.type == mujoco.mjtEq.mjEQ_JOINT and self.config.with_physics:
                equality_name = equality.name
                if equality_name == "":
                    equality_name = "Equality_" + str(equality_id)
                joint1_id = equality.obj1id[0]
                joint2_id = equality.obj2id[0]
                joint1_path = self._joint_builders[joint1_id].joint.GetPath()
                joint2_path = self._joint_builders[joint2_id].joint.GetPath()
                mujoco_equality_joint = UsdMujoco.MujocoEqualityJoint.Define(self.world_builder.stage,
                                                                             equality_prim.GetPath().AppendChild(
                                                                                 equality_name))
                mujoco_equality_joint.CreateJoint1Rel().SetTargets([joint1_path])
                mujoco_equality_joint.CreateJoint2Rel().SetTargets([joint2_path])
                mujoco_equality_joint.CreatePolycoefAttr(equality.data[:5])

    @property
    def mj_model(self) -> mujoco.MjModel:
        return self._mj_model

    @property
    def geom_type_map(self) -> Dict:
        return self._geom_type_map

    @property
    def mujoco_prim(self) -> Usd.Prim:
        return self.world_builder.stage.GetPrimAtPath("/mujoco")

    @property
    def mujoco_asset_prim(self) -> Usd.Prim:
        return self.mujoco_prim.GetChild("asset")

    @property
    def mujoco_meshes_prim(self) -> Usd.Prim:
        return self.mujoco_asset_prim.GetChild("meshes")

    @property
    def mujoco_materials_prim(self) -> Usd.Prim:
        return self.mujoco_asset_prim.GetChild("materials")

    @property
    def mujoco_textures_prim(self) -> Usd.Prim:
        return self.mujoco_asset_prim.GetChild("textures")

    @property
    def bodies_with_composite(self) -> Dict[str, List[ET.Element]]:
        return self._bodies_with_composite
