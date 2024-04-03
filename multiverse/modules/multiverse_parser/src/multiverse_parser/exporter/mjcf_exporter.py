#!/usr/bin/env python3

from dataclasses import dataclass
import os
from typing import Optional, Tuple
from xml.etree import ElementTree as ET
from xml.dom import minidom

import numpy
from numpy import radians

from ..factory import Factory
from ..factory import (JointBuilder, JointType,
                       GeomBuilder, GeomType,
                       MaterialProperty)
from ..utils import xform_cache, modify_name

from pxr import UsdMujoco, UsdUrdf, Gf, UsdPhysics, UsdGeom, Usd, UsdShade, Sdf


def build_inertial(xform_prim: Usd.Prim, body: ET.Element) -> None:
    mujoco_body_inertial_api = get_mujoco_inertial_api(xform_prim=xform_prim)

    inertial = ET.SubElement(body, "inertial")

    mass = mujoco_body_inertial_api.GetMassAttr().Get()
    pos = mujoco_body_inertial_api.GetPosAttr().Get()
    quat = mujoco_body_inertial_api.GetQuatAttr().Get()
    quat = numpy.array([quat.GetReal(), *quat.GetImaginary()])
    diaginertia = mujoco_body_inertial_api.GetDiaginertiaAttr().Get()

    inertial.set("mass", str(mass))
    inertial.set("pos", " ".join(map(str, pos)))
    inertial.set("quat", " ".join(map(str, quat)))
    inertial.set("diaginertia", " ".join(map(str, diaginertia)))


def get_mujoco_inertial_api(xform_prim: UsdGeom.Xform) -> UsdMujoco.MujocoBodyInertialAPI:
    if not xform_prim.HasAPI(UsdPhysics.MassAPI):
        return None

    physics_mass_api = UsdPhysics.MassAPI(xform_prim)
    mass = physics_mass_api.GetMassAttr().Get()
    pos = physics_mass_api.GetCenterOfMassAttr().Get()
    quat = physics_mass_api.GetPrincipalAxesAttr().Get()
    diagonal_inertia = physics_mass_api.GetDiagonalInertiaAttr().Get()

    mujoco_inertial_api = UsdMujoco.MujocoBodyInertialAPI.Apply(xform_prim)
    mujoco_inertial_api.CreateMassAttr(mass)
    mujoco_inertial_api.CreatePosAttr(pos)
    mujoco_inertial_api.CreateQuatAttr(quat)
    mujoco_inertial_api.CreateDiaginertiaAttr(diagonal_inertia)

    return mujoco_inertial_api


def get_mujoco_body_api(xform_prim: Usd.Prim,
                        parent_xform_prim: Optional[Usd.Prim] = None) -> UsdMujoco.MujocoBodyAPI:
    if parent_xform_prim is None:
        body_relative_transform = xform_cache.GetLocalToWorldTransform(xform_prim)
    else:
        parent_body_transform = xform_cache.GetLocalToWorldTransform(parent_xform_prim)
        body_transformation = xform_cache.GetLocalToWorldTransform(xform_prim)
        body_relative_transform = body_transformation * parent_body_transform.GetInverse()
    body_relative_transform = body_relative_transform.RemoveScaleShear()
    body_relative_pos = body_relative_transform.ExtractTranslation()
    body_relative_quat = body_relative_transform.ExtractRotationQuat()

    mujoco_body_api = UsdMujoco.MujocoBodyAPI.Apply(xform_prim)
    mujoco_body_api.CreatePosAttr(body_relative_pos)
    mujoco_body_api.CreateQuatAttr(Gf.Quatf(body_relative_quat))

    return mujoco_body_api


def get_mujoco_joint_api(joint_builder: JointBuilder) -> UsdMujoco.MujocoJointAPI:
    joint = joint_builder.joint
    joint_prim = joint.GetPrim()
    if joint_prim.HasAPI(UsdMujoco.MujocoJointAPI):
        mujoco_joint_api = UsdMujoco.MujocoJointAPI(joint_prim)
    else:
        mj_joint_type = "hinge" if joint_builder.type in [JointType.REVOLUTE, JointType.CONTINUOUS] \
            else "slide" if joint_builder.type == JointType.PRISMATIC \
            else "ball" if joint_builder.type == JointType.SPHERICAL \
            else None
        if mj_joint_type is None:
            raise NotImplementedError(f"Joint type {joint_builder.type} not supported.")

        mj_joint_pos = joint_builder.pos
        mj_joint_axis = joint_builder.axis.to_array()

        mujoco_joint_api = UsdMujoco.MujocoJointAPI.Apply(joint_prim)
        mujoco_joint_api.CreateTypeAttr(mj_joint_type)
        mujoco_joint_api.CreatePosAttr(mj_joint_pos)
        mujoco_joint_api.CreateAxisAttr(Gf.Vec3f(*mj_joint_axis))
        if joint_builder.type == JointType.PRISMATIC or joint_builder.type == JointType.REVOLUTE:
            if joint_builder.type == JointType.PRISMATIC:
                lower = joint.GetLowerLimitAttr().Get()
                upper = joint.GetUpperLimitAttr().Get()
            else:
                lower = radians(joint.GetLowerLimitAttr().Get())
                upper = radians(joint.GetUpperLimitAttr().Get())
            mujoco_joint_api.CreateRangeAttr(Gf.Vec2f(lower, upper))

    return mujoco_joint_api


def get_mujoco_geom_api(geom_builder: GeomBuilder) -> UsdMujoco.MujocoGeomAPI:
    gprim = geom_builder.gprim
    gprim_prim = gprim.GetPrim()
    if gprim_prim.HasAPI(UsdMujoco.MujocoGeomAPI):
        mujoco_geom_api = UsdMujoco.MujocoGeomAPI(gprim_prim)
    else:
        geom_transformation = gprim.GetLocalTransformation().RemoveScaleShear()
        geom_pos = geom_transformation.ExtractTranslation()
        geom_quat = geom_transformation.ExtractRotationQuat()
        if geom_builder.type == GeomType.CUBE:
            if gprim_prim.HasAPI(UsdUrdf.UrdfGeometryBoxAPI):
                urdf_geometry_box_api = UsdUrdf.UrdfGeometryBoxAPI(gprim_prim)
                geom_size = urdf_geometry_box_api.GetSizeAttr().Get()
                geom_size = numpy.array([*geom_size]) / 2.0
            else:
                raise NotImplementedError(f"Geom type {geom_builder.type} not implemented.")
            geom_type = "box"
        elif geom_builder.type == GeomType.SPHERE:
            geom_size = numpy.array([gprim.GetRadiusAttr().Get(), 0.0, 0.0])
            geom_type = "sphere"
        elif geom_builder.type == GeomType.CYLINDER:
            geom_size = numpy.array([gprim.GetRadiusAttr().Get(),
                                     gprim.GetHeightAttr().Get() / 2, 0.0])
            geom_type = "cylinder"
        elif geom_builder.type == GeomType.CAPSULE:
            geom_size = numpy.array([gprim.GetRadiusAttr().Get(),
                                     gprim.GetHeightAttr().Get() / 2, 0.0])
            geom_type = "capsule"
        elif geom_builder.type == GeomType.MESH:
            if gprim_prim.HasAPI(UsdUrdf.UrdfGeometryMeshAPI):
                urdf_geometry_mesh_api = UsdUrdf.UrdfGeometryMeshAPI(gprim_prim)
                geom_size = urdf_geometry_mesh_api.GetScaleAttr().Get()
                geom_size = numpy.array([*geom_size]) if geom_size is not None else numpy.array([1.0, 1.0, 1.0])
            else:
                xform = UsdGeom.Xform(gprim_prim)
                transformation = xform.GetLocalTransformation()
                geom_size = numpy.array([round(transformation.GetRow(i).GetLength(), 3) for i in range(3)])
            geom_type = "mesh"
        else:
            raise NotImplementedError(f"Geom type {geom_builder.type} not implemented.")

        mujoco_geom_api = UsdMujoco.MujocoGeomAPI.Apply(gprim_prim)
        mujoco_geom_api.CreatePosAttr(geom_pos)
        mujoco_geom_api.CreateQuatAttr(Gf.Quatf(geom_quat))
        mujoco_geom_api.CreateSizeAttr(Gf.Vec3f(*geom_size))
        mujoco_geom_api.CreateTypeAttr(geom_type)
        if geom_builder.type == GeomType.MESH:
            prepended_items = gprim_prim.GetPrimStack()[0].referenceList.prependedItems
            if len(prepended_items) != 1:
                raise NotImplementedError(f"Geom {gprim_prim.GetName()} has {len(prepended_items)} prepended items.")

            stage = gprim_prim.GetStage()
            mesh_file_path = prepended_items[0].assetPath
            mesh_name = os.path.splitext(os.path.basename(mesh_file_path))[0]
            mesh_name = add_scale_to_mesh_name(mesh_name=mesh_name, mesh_scale=geom_size)

            mujoco_asset_prim = stage.GetPrimAtPath("/mujoco/asset")
            mujoco_mesh_path = mujoco_asset_prim.GetPath().AppendChild("meshes").AppendChild(mesh_name)
            if (not stage.GetPrimAtPath(mujoco_mesh_path).IsValid() or
                    not stage.GetPrimAtPath(mujoco_mesh_path).IsA(UsdMujoco.MujocoMesh)):
                stage.GetRootLayer().Save()
                raise ValueError(f"Mesh {mujoco_mesh_path} does not exist in {stage.GetRootLayer().realPath}")
            mujoco_geom_api.CreateMeshRel().SetTargets([mujoco_mesh_path])

            if gprim_prim.HasAPI(UsdShade.MaterialBindingAPI):
                material_binding_api = UsdShade.MaterialBindingAPI(gprim_prim)
                material_path = material_binding_api.GetDirectBindingRel().GetTargets()[0]
                material_name = material_path.name
                mujoco_material_path = mujoco_asset_prim.GetPath().AppendChild("materials").AppendChild(
                    material_name)
                if not stage.GetPrimAtPath(mujoco_material_path).IsA(UsdMujoco.MujocoMaterial):
                    raise ValueError(f"Material {material_name} does not exist.")
                mujoco_geom_api.CreateMaterialRel().SetTargets([mujoco_material_path])

    return mujoco_geom_api


def add_scale_to_mesh_name(mesh_name: str, mesh_scale: numpy.ndarray) -> str:
    if not numpy.isclose(mesh_scale, numpy.array([1.0, 1.0, 1.0])).all():
        mesh_name += "_" + "_".join(map(str, mesh_scale))
    mesh_name = modify_name(mesh_name, "Mesh_")
    return mesh_name


@dataclass(frozen=True, eq=True)
class MeshFileProperty:
    scale: Tuple[float, float, float]
    has_texture: bool


class MjcfExporter:
    def __init__(
            self,
            factory: Factory,
            file_path: str,
    ) -> None:
        self._factory = factory
        self._file_path = file_path
        self._mesh_dir_abs_path = os.path.join(os.path.dirname(self.file_path), self.file_name)
        self._root = ET.Element("mujoco")
        self._body_dict = {}

    def build(self) -> None:
        self._build_config()

        worldbody = ET.SubElement(self.root, "worldbody")
        self.body_dict["worldbody"] = worldbody

        world_builder = self.factory.world_builder
        first_body_builder = world_builder.body_builders[0]
        first_body_name = first_body_builder.xform.GetPrim().GetName()
        if first_body_name == "world":
            self.body_dict["world"] = worldbody
            for geom_builder in first_body_builder.geom_builders:
                self._build_geom(geom_builder=geom_builder, body=worldbody)
        else:
            self._build_body(body_name=first_body_name, parent_body_name="worldbody")

        body_builders = world_builder.body_builders
        reduces_body_builders = body_builders

        stop = False
        while not stop:
            stop = True
            for body_builder in body_builders:
                body_name = body_builder.xform.GetPrim().GetName()
                parent_body_name = body_builder.xform.GetPrim().GetParent().GetName()
                if (parent_body_name in self.body_dict and
                        body_name not in self.body_dict and
                        len(body_builder.joint_builders) == 0):
                    stop = False
                    self._build_body(body_name=body_name, parent_body_name=parent_body_name)
                    reduces_body_builders.remove(body_builder)
                for joint_builder in body_builder.joint_builders:
                    parent_body_name = joint_builder.parent_prim.GetName()
                    child_body_name = joint_builder.child_prim.GetName()
                    if parent_body_name in self.body_dict and child_body_name not in self.body_dict:
                        stop = False
                        self._build_body(body_name=child_body_name, parent_body_name=parent_body_name)
                        if self.factory.config.with_physics:
                            self._build_joint(joint_builder=joint_builder, body_name=child_body_name)
                        child_body_builder = world_builder.get_body_builder(child_body_name)
                        reduces_body_builders.remove(child_body_builder)
            body_builders = reduces_body_builders

        self._export_equality()

    def _build_config(self):
        stage = self.factory.world_builder.stage

        self._import_mujoco()

        self._build_mujoco_asset_mesh_and_material_prims()

        model_name = UsdMujoco.Mujoco(self.mujoco_prim).GetModelAttr().Get()
        self.root.set("model", model_name)

        compiler = ET.SubElement(self.root, "compiler")
        meshdir = os.path.join(self.file_name, "meshes")
        compiler.set("meshdir", meshdir)
        texturedir = os.path.join(self.file_name, "textures")
        compiler.set("texturedir", texturedir)
        compiler.set("angle", "radian")
        compiler.set("autolimits", "true")
        compiler.set("balanceinertia", "true")
        compiler.set("boundmass", "0.000001")
        compiler.set("boundinertia", "0.000001")

        default = ET.SubElement(self.root, "default")
        default_visual = ET.SubElement(default, "default")
        default_visual.set("class", "visual")
        default_visual_geom = ET.SubElement(default_visual, "geom")
        default_visual_geom.set("contype", "0")
        default_visual_geom.set("conaffinity", "0")

        default_collision = ET.SubElement(default, "default")
        default_collision.set("class", "collision")
        default_collision_geom = ET.SubElement(default_collision, "geom")
        default_collision_geom.set(
            "rgba",
            " ".join(
                map(
                    str, self.factory.config.default_rgba
                )
            ),
        )

        asset = ET.SubElement(self.root, "asset")
        mujoco_meshes = [UsdMujoco.MujocoMesh(prim) for prim in self.mujoco_meshes_prim.GetChildren()
                         if prim.IsA(UsdMujoco.MujocoMesh)]
        for mujoco_mesh in mujoco_meshes:
            mesh = ET.SubElement(asset, "mesh")
            mesh.set("name", mujoco_mesh.GetPrim().GetName())
            tmp_mesh_path = mujoco_mesh.GetFileAttr().Get().path
            tmp_mesh_relpath = os.path.relpath(tmp_mesh_path, self.factory.tmp_mesh_dir_path)

            mesh.set("file", tmp_mesh_relpath)
            scale = mujoco_mesh.GetScaleAttr().Get()
            mesh.set("scale", " ".join(map(str, scale)))

        mujoco_materials = [UsdMujoco.MujocoMaterial(prim) for prim in self.mujoco_materials_prim.GetChildren()
                            if prim.IsA(UsdMujoco.MujocoMaterial)]
        for mujoco_material in mujoco_materials:
            material = ET.SubElement(asset, "material")
            material.set("name", mujoco_material.GetPrim().GetName())
            if len(mujoco_material.GetTextureRel().GetTargets()) > 1:
                raise NotImplementedError(f"Material {mujoco_material.GetPrim().GetName()} has "
                                          f"{len(mujoco_material.GetTextureRel().GetTargets())} textures.")
            for texture_path in mujoco_material.GetTextureRel().GetTargets():
                texture_prim = stage.GetPrimAtPath(texture_path)
                texture_name = texture_prim.GetName()
                material.set("texture", texture_name)

            if len(mujoco_material.GetTextureRel().GetTargets()) == 0:
                rgba = mujoco_material.GetRgbaAttr().Get()
                if rgba is not None:
                    material.set("rgba", " ".join(map(str, rgba)))
                emission = mujoco_material.GetEmissionAttr().Get()
                if emission is not None:
                    material.set("emission", str(emission))
                specular = mujoco_material.GetSpecularAttr().Get()
                if specular is not None:
                    material.set("specular", str(specular))

        mujoco_textures = [UsdMujoco.MujocoTexture(prim) for prim in self.mujoco_textures_prim.GetChildren()
                           if prim.IsA(UsdMujoco.MujocoTexture)]
        for mujoco_texture in mujoco_textures:
            texture = ET.SubElement(asset, "texture")
            texture.set("name", mujoco_texture.GetPrim().GetName())
            texture_type = mujoco_texture.GetTypeAttr().Get()
            texture.set("type", texture_type)
            texture.set("file", mujoco_texture.GetFileAttr().Get().path)

    def _import_mujoco(self):
        stage = self.factory.world_builder.stage
        if not stage.GetPrimAtPath("/mujoco").IsValid():
            usd_mujoco = UsdMujoco.Mujoco.Define(stage, "/mujoco")
            model_name = stage.GetDefaultPrim().GetName()
            usd_mujoco.CreateModelAttr(model_name)
            self._import_option()
            self._import_asset()

    def _import_option(self):
        if not self.mujoco_prim.HasAPI(UsdMujoco.MujocoOptionAPI):
            UsdMujoco.MujocoOptionAPI.Apply(self.mujoco_prim)

    def _import_asset(self):
        stage = self.factory.world_builder.stage
        if not stage.GetPrimAtPath("/mujoco/asset").IsValid():
            UsdMujoco.MujocoAsset.Define(stage, "/mujoco/asset")
            UsdMujoco.MujocoMesh.Define(stage, "/mujoco/asset/meshes")
            UsdMujoco.MujocoMaterial.Define(stage, "/mujoco/asset/materials")
            UsdMujoco.MujocoTexture.Define(stage, "/mujoco/asset/textures")

    def _build_mujoco_asset_mesh_and_material_prims(self):
        stage = self.factory.world_builder.stage
        mesh_files = {}
        mesh_dir_name = os.path.dirname(stage.GetRootLayer().realPath)
        for prim in stage.TraverseAll():
            if not prim.IsA(UsdGeom.Mesh):
                continue
            mujoco_mesh_api = UsdMujoco.MujocoGeomAPI(prim)
            if len(mujoco_mesh_api.GetMaterialRel().GetTargets()) > 0:
                continue
            prepended_items = prim.GetPrimStack()[0].referenceList.prependedItems
            if len(prepended_items) > 0:
                for prepended_item in prepended_items:
                    mesh_file_path = prepended_item.assetPath
                    if not os.path.isabs(mesh_file_path):
                        mesh_file_path = os.path.join(mesh_dir_name, mesh_file_path)
                    if prim.HasAPI(UsdUrdf.UrdfGeometryMeshAPI):
                        urdf_geometry_mesh_api = UsdUrdf.UrdfGeometryMeshAPI(prim)
                        mesh_scale = urdf_geometry_mesh_api.GetScaleAttr().Get()
                        mesh_scale = tuple(mesh_scale) if mesh_scale is not None else (1.0, 1.0, 1.0)
                    else:
                        xform = UsdGeom.Xform(prim)
                        transformation = xform.GetLocalTransformation()
                        mesh_scale = tuple(round(transformation.GetRow(i).GetLength(), 3) for i in range(3))

                    mesh_has_texture = False
                    if prim.HasAPI(UsdShade.MaterialBindingAPI):
                        material_binding_api = UsdShade.MaterialBindingAPI(prim)
                        material_path = material_binding_api.GetDirectBindingRel().GetTargets()[0]
                        material_prim = stage.GetPrimAtPath(material_path)
                        material_property = MaterialProperty.from_prim(material_prim=material_prim)
                        if isinstance(material_property.diffuse_color, str):
                            mesh_has_texture = True

                    mesh_file_property = MeshFileProperty(scale=mesh_scale,
                                                          has_texture=mesh_has_texture)

                    if mesh_file_path not in mesh_files:
                        mesh_files[mesh_file_path] = {mesh_file_property}
                    elif mesh_scale not in mesh_files[mesh_file_path]:
                        mesh_files[mesh_file_path].add(mesh_file_property)

        for mesh_file_path, mesh_file_properties in mesh_files.items():
            mesh_file_name = os.path.splitext(os.path.basename(mesh_file_path))[0]
            for mesh_file_property in mesh_file_properties:
                mesh_file_ext = "obj" if mesh_file_property.has_texture else "stl"
                tmp_mesh_file_path = os.path.join(self.factory.tmp_mesh_dir_path,
                                                  mesh_file_ext,
                                                  f"{mesh_file_name}.{mesh_file_ext}")
                if not os.path.exists(tmp_mesh_file_path):
                    self.factory.export_mesh(in_mesh_file_path=mesh_file_path,
                                             out_mesh_file_path=tmp_mesh_file_path)

                mesh_file_name = add_scale_to_mesh_name(mesh_name=mesh_file_name,
                                                   mesh_scale=numpy.array(mesh_file_property.scale))
                mujoco_mesh_path = self.mujoco_meshes_prim.GetPath().AppendChild(mesh_file_name)
                if stage.GetPrimAtPath(mujoco_mesh_path).IsValid():
                    continue
                mujoco_mesh = UsdMujoco.MujocoMesh.Define(stage, mujoco_mesh_path)
                mujoco_mesh.CreateFileAttr(tmp_mesh_file_path)
                mujoco_mesh.CreateScaleAttr(Gf.Vec3f(*mesh_file_property.scale))

        materials = {}
        for material_prim in [child_prim for child_prim in stage.TraverseAll() if child_prim.IsA(UsdShade.Material)]:
            material_name = material_prim.GetName()
            if material_name in materials:
                continue
            materials[material_name] = MaterialProperty.from_prim(material_prim=material_prim)

        for material_name, material_property in materials.items():
            mujoco_material_path = self.mujoco_materials_prim.GetPath().AppendChild(material_name)
            if stage.GetPrimAtPath(mujoco_material_path).IsValid():
                continue

            mujoco_material = UsdMujoco.MujocoMaterial.Define(stage, mujoco_material_path)
            if material_property.diffuse_color is not None and material_property.opacity is not None:
                if isinstance(material_property.diffuse_color, numpy.ndarray):
                    rgba = Gf.Vec4f(*material_property.diffuse_color.tolist(), material_property.opacity)
                    mujoco_material.CreateRgbaAttr(rgba)

                    if material_property.emissive_color is not None and all(
                            [c != 0.0 for c in material_property.diffuse_color]):
                        emissions = [material_property.emissive_color[i] / material_property.diffuse_color[i]
                                     for i in range(3)]
                        if emissions[0] == emissions[1] == emissions[2]:
                            mujoco_material.CreateEmissionAttr(float(emissions[0]))

                elif isinstance(material_property.diffuse_color, str):
                    texture_name = os.path.splitext(os.path.basename(material_property.diffuse_color))[0]
                    mujoco_texture_path = self.mujoco_textures_prim.GetPath().AppendChild(texture_name)
                    mujoco_material.CreateTextureRel().SetTargets([mujoco_texture_path])

                    mujoco_texture = UsdMujoco.MujocoTexture.Define(stage, mujoco_texture_path)
                    mujoco_texture.CreateTypeAttr("2d")
                    mujoco_texture.CreateFileAttr(f"{texture_name}.png")
                else:
                    raise NotImplementedError(f"Material {material_name} does not have a proper diffuse color.")

            specular_color = material_property.specular_color
            if (isinstance(specular_color, numpy.ndarray) and
                    specular_color[0] == specular_color[1] == specular_color[2]):
                specular = float(specular_color[0])
                mujoco_material.CreateSpecularAttr(specular)

    def _build_body(self, body_name: str, parent_body_name: str) -> None:
        parent_body = self.body_dict[parent_body_name]
        body = ET.SubElement(parent_body, "body")
        self.body_dict[body_name] = body

        body.set("name", body_name)

        world_builder = self.factory.world_builder
        body_builder = world_builder.get_body_builder(body_name)

        xform_prim = body_builder.xform.GetPrim()
        if self.factory.config.with_physics and xform_prim.HasAPI(UsdPhysics.MassAPI):
            build_inertial(xform_prim=xform_prim, body=body)

        if xform_prim.HasAPI(UsdMujoco.MujocoBodyAPI):
            mujoco_body_api = UsdMujoco.MujocoBodyAPI(xform_prim)
        else:
            if parent_body_name == "worldbody":
                mujoco_body_api = get_mujoco_body_api(xform_prim=xform_prim)
            else:
                parent_body_builder = world_builder.get_body_builder(parent_body_name)
                parent_xform_prim = parent_body_builder.xform.GetPrim()
                mujoco_body_api = get_mujoco_body_api(xform_prim=xform_prim, parent_xform_prim=parent_xform_prim)

        pos = mujoco_body_api.GetPosAttr().Get()
        quat = mujoco_body_api.GetQuatAttr().Get()
        quat = numpy.array([quat.GetReal(), *quat.GetImaginary()])

        body.set("pos", " ".join(map(str, pos)))
        body.set("quat", " ".join(map(str, quat)))

        for geom_builder in body_builder.geom_builders:
            self._build_geom(geom_builder=geom_builder, body=body)

    def _build_geom(self, geom_builder: GeomBuilder, body: ET.Element) -> None:
        mujoco_geom_api = get_mujoco_geom_api(geom_builder=geom_builder)

        gprim_prim = geom_builder.gprim.GetPrim()
        geom_name = gprim_prim.GetName()
        geom = ET.SubElement(body, "geom")
        geom.set("name", geom_name)
        geom_type = mujoco_geom_api.GetTypeAttr().Get()
        geom.set("type", geom_type)
        geom_pos = mujoco_geom_api.GetPosAttr().Get()
        geom.set("pos", " ".join(map(str, geom_pos)))
        geom_quat = mujoco_geom_api.GetQuatAttr().Get()
        geom_quat = numpy.array([geom_quat.GetReal(), *geom_quat.GetImaginary()])
        geom.set("quat", " ".join(map(str, geom_quat)))
        if geom_type != "mesh":
            geom_size = mujoco_geom_api.GetSizeAttr().Get()
            geom.set("size", " ".join(map(str, geom_size)))
        else:
            mesh_rel_path = mujoco_geom_api.GetMeshRel().GetTargets()[0]
            mesh_name = mesh_rel_path.name
            geom.set("mesh", mesh_name)

        if len(mujoco_geom_api.GetMaterialRel().GetTargets()) > 0:
            material_rel_path = mujoco_geom_api.GetMaterialRel().GetTargets()[0]
            material_name = material_rel_path.name
            geom.set("material", material_name)
        elif not gprim_prim.GetPrim().HasAPI(UsdPhysics.CollisionAPI):
            rgba = geom_builder.rgba
            if rgba is not None:
                geom.set("rgba", " ".join(map(str, rgba)))

        if gprim_prim.GetPrim().HasAPI(UsdPhysics.CollisionAPI):
            geom.set("class", "collision")
        else:
            geom.set("class", "visual")

    def _build_joint(self, joint_builder: JointBuilder, body_name: str) -> None:
        mujoco_joint_api = get_mujoco_joint_api(joint_builder=joint_builder)
        if joint_builder.type == JointType.FIXED:
            return

        joint_prim = joint_builder.joint.GetPrim()
        joint_name = joint_prim.GetName()

        body = self.body_dict[body_name]
        joint = ET.SubElement(body, "joint")
        joint.set("name", joint_name)

        joint_type = mujoco_joint_api.GetTypeAttr().Get()
        joint_pos = mujoco_joint_api.GetPosAttr().Get()
        joint.set("type", joint_type)
        joint.set("pos", " ".join(map(str, joint_pos)))

        if joint_builder.type == JointType.PRISMATIC or joint_builder.type == JointType.REVOLUTE:
            joint_range = mujoco_joint_api.GetRangeAttr().Get()
            joint.set("range", " ".join(map(str, joint_range)))

        if joint_builder.type != JointType.SPHERICAL:
            joint_axis = mujoco_joint_api.GetAxisAttr().Get()
            joint.set("axis", " ".join(map(str, joint_axis)))

    def _export_equality(self):
        stage = self.factory.world_builder.stage

        equality_path = Sdf.Path("/mujoco/equality")
        equality_prim = stage.GetPrimAtPath(equality_path)
        if not equality_prim.IsValid():
            equality_prim = UsdMujoco.MujocoEquality.Define(stage, equality_path).GetPrim()

        equality = ET.SubElement(self.root, "equality")
        for child_prim in equality_prim.GetChildren():
            if child_prim.IsA(UsdMujoco.MujocoEqualityJoint):
                equality_joint_prim = UsdMujoco.MujocoEqualityJoint(child_prim)
                joint1_path = equality_joint_prim.GetJoint1Rel().GetTargets()[0]
                joint2_path = equality_joint_prim.GetJoint2Rel().GetTargets()[0]
                if (not stage.GetPrimAtPath(joint1_path).IsA(UsdPhysics.Joint)
                        or not stage.GetPrimAtPath(joint2_path).IsA(UsdPhysics.Joint)):
                    raise ValueError(f"Equality joint {child_prim.GetName()} does not exist.")
                joint1_name = joint1_path.name
                joint2_name = joint2_path.name
                poly_coef = equality_joint_prim.GetPolycoefAttr().Get()

                equality_joint = ET.SubElement(equality, "joint")
                equality_joint.set("name", child_prim.GetName())
                equality_joint.set("joint1", joint1_name)
                equality_joint.set("joint2", joint2_name)
                equality_joint.set("polycoef", " ".join(map(str, poly_coef)))

    def export(self, keep_usd: bool = True) -> None:
        os.makedirs(name=os.path.dirname(self.file_path), exist_ok=True)

        rough_string = ET.tostring(self.root, "utf-8")
        parsed_string = minidom.parseString(rough_string)
        pretty_string = parsed_string.toprettyxml()

        with open(self.file_path, "w", encoding="utf-8") as file:
            file.write(pretty_string)

        if keep_usd:
            self.factory.save_tmp_model(usd_file_path=self.file_path.replace(".xml", ".usda"))
        else:
            self.factory.save_tmp_model(usd_file_path=self.file_path.replace(".xml", ".usda"),
                                        excludes=["usd", ".usda"])

    @property
    def file_path(self) -> str:
        return self._file_path

    @property
    def factory(self) -> Factory:
        return self._factory

    @property
    def file_name(self) -> str:
        return os.path.splitext(os.path.basename(self.file_path))[0]

    @property
    def mesh_dir_abspath(self) -> str:
        return self._mesh_dir_abs_path

    @property
    def root(self) -> ET.Element:
        return self._root

    @property
    def body_dict(self) -> dict:
        return self._body_dict

    @property
    def mujoco_prim(self) -> Usd.Prim:
        return self.factory.world_builder.stage.GetPrimAtPath("/mujoco")

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