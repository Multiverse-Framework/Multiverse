#!/usr/bin/env python3

import os
from typing import Optional, Tuple
from xml.etree import ElementTree as ET
from xml.dom import minidom

import numpy
from numpy import radians
from pxr import UsdMujoco, UsdUrdf, Gf, UsdPhysics, UsdGeom, Usd, UsdShade

from ..factory import copy_and_overwrite
from ..factory import Factory
from ..factory import (JointBuilder, JointType,
                       GeomBuilder, GeomType)
from ..utils import xform_cache, modify_name, scale_mesh


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


def get_mujoco_prim(stage: Usd.Stage) -> Usd.Prim:
    if stage.GetPrimAtPath("/mujoco").IsValid():
        usd_mujoco = UsdMujoco.Mujoco(stage.GetPrimAtPath("/mujoco"))
    else:
        usd_mujoco = UsdMujoco.Mujoco.Define(stage, "/mujoco")
        model_name = stage.GetDefaultPrim().GetName()
        usd_mujoco.CreateModelAttr(model_name)
    return usd_mujoco.GetPrim()


def get_mujoco_option_api(mujoco_prim) -> UsdMujoco.MujocoOptionAPI:
    if mujoco_prim.HasAPI(UsdMujoco.MujocoOptionAPI):
        mujoco_option_api = UsdMujoco.MujocoOptionAPI(mujoco_prim)
    else:
        mujoco_option_api = UsdMujoco.MujocoOptionAPI.Apply(mujoco_prim)
    return mujoco_option_api


def get_mujoco_asset_prim(stage) -> Tuple[Usd.Prim, Usd.Prim, Usd.Prim, Usd.Prim]:
    if stage.GetPrimAtPath("/mujoco/asset").IsValid():
        mujoco_asset_prim = stage.GetPrimAtPath("/mujoco/asset")
        mujoco_meshes_prim = stage.GetPrimAtPath("/mujoco/asset/meshes")
        mujoco_materials_prim = stage.GetPrimAtPath("/mujoco/asset/materials")
        mujoco_textures_prim = stage.GetPrimAtPath("/mujoco/asset/textures")
    else:
        mujoco_asset_prim = UsdMujoco.MujocoAsset.Define(stage, "/mujoco/asset")
        mujoco_meshes_prim = UsdMujoco.MujocoMesh.Define(stage, "/mujoco/asset/meshes")
        mujoco_materials_prim = UsdMujoco.MujocoMaterial.Define(stage, "/mujoco/asset/materials")
        mujoco_textures_prim = UsdMujoco.MujocoTexture.Define(stage, "/mujoco/asset/textures")
    return (mujoco_asset_prim.GetPrim(),
            mujoco_meshes_prim.GetPrim(),
            mujoco_materials_prim.GetPrim(),
            mujoco_textures_prim.GetPrim())


def get_mujoco_geom_api(geom_builder: GeomBuilder) -> UsdMujoco.MujocoGeomAPI:
    xform = geom_builder.xform
    xform_prim = xform.GetPrim()
    if xform_prim.HasAPI(UsdMujoco.MujocoGeomAPI):
        mujoco_geom_api = UsdMujoco.MujocoGeomAPI(xform_prim)
    else:
        geom_transformation = xform.GetLocalTransformation().RemoveScaleShear()
        geom_pos = geom_transformation.ExtractTranslation()
        geom_quat = geom_transformation.ExtractRotationQuat()
        if geom_builder.type == GeomType.CUBE:
            if xform_prim.HasAPI(UsdUrdf.UrdfGeometryBoxAPI):
                urdf_geometry_box_api = UsdUrdf.UrdfGeometryBoxAPI(xform_prim)
                geom_size = urdf_geometry_box_api.GetSizeAttr().Get()
                geom_size = numpy.array([*geom_size]) / 2.0
            else:
                raise NotImplementedError(f"Geom type {geom_builder.type} not implemented.")
            geom_type = "box"
        elif geom_builder.type == GeomType.SPHERE:
            geom_sphere_prim = UsdGeom.Sphere(geom_builder.gprims[0])
            geom_size = numpy.array([geom_sphere_prim.GetRadiusAttr().Get(), 0.0, 0.0])
            geom_type = "sphere"
        elif geom_builder.type == GeomType.CYLINDER:
            geom_cylinder_prim = UsdGeom.Cylinder(geom_builder.gprims[0])
            geom_size = numpy.array([geom_cylinder_prim.GetRadiusAttr().Get(),
                                     geom_cylinder_prim.GetHeightAttr().Get() / 2, 0.0])
            geom_type = "cylinder"
        elif geom_builder.type == GeomType.CAPSULE:
            geom_cylinder_prim = UsdGeom.Cylinder(geom_builder.gprims[0])
            geom_size = numpy.array([geom_cylinder_prim.GetRadiusAttr().Get(),
                                     geom_cylinder_prim.GetHeightAttr().Get() / 2, 0.0])
            geom_type = "capsule"
        elif geom_builder.type == GeomType.MESH:
            if xform_prim.HasAPI(UsdUrdf.UrdfGeometryMeshAPI):
                urdf_geometry_mesh_api = UsdUrdf.UrdfGeometryMeshAPI(xform_prim)
                geom_size = urdf_geometry_mesh_api.GetScaleAttr().Get()
                geom_size = numpy.array([*geom_size]) if geom_size is not None else numpy.array([1.0, 1.0, 1.0])
            else:
                raise NotImplementedError(f"Geom type {geom_builder.type} not implemented.")
            geom_type = "mesh"
        else:
            raise NotImplementedError(f"Geom type {geom_builder.type} not implemented.")

        mujoco_geom_api = UsdMujoco.MujocoGeomAPI.Apply(xform_prim)
        mujoco_geom_api.CreatePosAttr(geom_pos)
        mujoco_geom_api.CreateQuatAttr(Gf.Quatf(geom_quat))
        mujoco_geom_api.CreateSizeAttr(Gf.Vec3f(*geom_size))
        mujoco_geom_api.CreateTypeAttr(geom_type)
        if geom_builder.type == GeomType.MESH:
            stage = xform_prim.GetStage()
            mujoco_asset_prim = stage.GetPrimAtPath("/mujoco/asset")
            if xform_prim.HasAPI(UsdUrdf.UrdfGeometryMeshAPI):
                urdf_geometry_mesh_api = UsdUrdf.UrdfGeometryMeshAPI(xform_prim)
                mesh_path = urdf_geometry_mesh_api.GetFilenameAttr().Get()
                mesh_name = os.path.splitext(os.path.basename(mesh_path.path))[0]
            else:
                raise NotImplementedError(f"Geom type {geom_builder.type} not implemented.")

            mesh_name = add_scale_to_mesh_name(mesh_name=mesh_name, mesh_scale=geom_size)
            mujoco_mesh_path = mujoco_asset_prim.GetPath().AppendChild("meshes").AppendChild(mesh_name)
            mujoco_geom_api.CreateMeshRel().SetTargets([mujoco_mesh_path])

            if len(geom_builder.material_builders) > 1:
                raise NotImplementedError(f"Geom {geom_builder.xform.GetPrim().GetName()} has "
                                          f"{len(geom_builder.material_builders)} materials.")
            for material_builder in geom_builder.material_builders:
                if len(material_builder.materials) > 1:
                    raise NotImplementedError(f"Material in {material_builder.stage.GetRootLayer().realPath} has "
                                              f"{len(material_builder.materials)} materials.")
                for material in material_builder.materials:
                    material_name = material.GetPrim().GetName()
                    mujoco_material_path = mujoco_asset_prim.GetPath().AppendChild("materials").AppendChild(
                        material_name)
                    mujoco_geom_api.CreateMaterialRel().SetTargets([mujoco_material_path])

    return mujoco_geom_api


def add_scale_to_mesh_name(mesh_name: str, mesh_scale: numpy.ndarray) -> str:
    if not numpy.isclose(mesh_scale, numpy.array([1.0, 1.0, 1.0])).all():
        mesh_name += "_" + "_".join(map(str, mesh_scale))
    mesh_name = modify_name(mesh_name)
    return mesh_name


class MjcfExporter:
    def __init__(
            self,
            factory: Factory,
            file_path: str
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

    def _build_config(self):
        stage = self.factory.world_builder.stage

        mujoco_prim = get_mujoco_prim(stage=stage)

        mujoco_option_api = get_mujoco_option_api(mujoco_prim=mujoco_prim)

        (mujoco_asset_prim,
         mujoco_meshes_prim,
         mujoco_materials_prim,
         mujoco_textures_prim) = get_mujoco_asset_prim(stage=stage)

        self._build_mujoco_asset_mesh_and_material_prims(stage=self.factory.world_builder.stage,
                                                         mujoco_meshes_prim=mujoco_meshes_prim,
                                                         mujoco_materials_prim=mujoco_materials_prim)

        model_name = UsdMujoco.Mujoco(mujoco_prim).GetModelAttr().Get()
        self.root.set("model", model_name)

        compiler = ET.SubElement(self.root, "compiler")
        compiler.set("meshdir", self.file_name + "/")

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
        mujoco_meshes = [UsdMujoco.MujocoMesh(prim) for prim in mujoco_meshes_prim.GetChildren()
                         if prim.IsA(UsdMujoco.MujocoMesh)]
        for mujoco_mesh in mujoco_meshes:
            mesh = ET.SubElement(asset, "mesh")
            mesh.set("name", mujoco_mesh.GetPrim().GetName())
            tmp_mesh_path = mujoco_mesh.GetFileAttr().Get().path
            tmp_mesh_dir_path = os.path.join(os.path.dirname(self.factory.tmp_usd_file_path), "tmp")
            tmp_mesh_relpath = os.path.relpath(tmp_mesh_path, tmp_mesh_dir_path)

            mesh.set("file", tmp_mesh_relpath)
            scale = mujoco_mesh.GetScaleAttr().Get()
            mesh.set("scale", " ".join(map(str, scale)))

        mujoco_materials = [UsdMujoco.MujocoMaterial(prim) for prim in mujoco_materials_prim.GetChildren()
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
                material.set("rgba", " ".join(map(str, rgba)))
                emission = mujoco_material.GetEmissionAttr().Get()
                material.set("emission", str(emission))
                specular = mujoco_material.GetSpecularAttr().Get()
                material.set("specular", str(specular))

        mujoco_textures = [UsdMujoco.MujocoTexture(prim) for prim in mujoco_textures_prim.GetChildren()
                           if prim.IsA(UsdMujoco.MujocoTexture)]
        for mujoco_texture in mujoco_textures:
            texture = ET.SubElement(asset, "texture")
            texture.set("name", mujoco_texture.GetPrim().GetName())
            texture_type = mujoco_texture.GetTypeAttr().Get()
            texture.set("type", texture_type)
            texture.set("file", mujoco_texture.GetFileAttr().Get().path)

    def _build_mujoco_asset_mesh_and_material_prims(self,
                                                    stage: Usd.Stage,
                                                    mujoco_meshes_prim: Usd.Prim,
                                                    mujoco_materials_prim: Usd.Prim):
        mesh_file_paths = {}
        materials = {}
        mesh_dir_name = os.path.dirname(stage.GetRootLayer().realPath)

        for prim in stage.TraverseAll():
            if not prim.IsA(UsdGeom.Xform):
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
                        mesh_scale = (1.0, 1.0, 1.0)
                    if mesh_file_path not in mesh_file_paths:
                        mesh_file_paths[mesh_file_path] = {mesh_scale}
                    elif mesh_scale not in mesh_file_paths[mesh_file_path]:
                        mesh_file_paths[mesh_file_path].add(mesh_scale)

        for mesh_file_path, mesh_scales in mesh_file_paths.items():
            mesh_stage = Usd.Stage.Open(mesh_file_path)
            mesh_file_name = os.path.basename(mesh_file_path).split(".")[0]
            mesh_file_ext = "from_obj" if len([material for material in mesh_stage.TraverseAll() if
                                     material.IsA(UsdShade.Material)]) > 0 else "from_stl"
            tmp_mesh_file_path = os.path.join(os.path.dirname(self.factory.tmp_usd_mesh_dir_path), mesh_file_ext,
                                              f"{mesh_file_name}.{mesh_file_ext}")
            self.factory.export_mesh(in_mesh_file_path=mesh_file_path,
                                     out_mesh_file_path=tmp_mesh_file_path)

            for mesh_scale in mesh_scales:
                mesh_name = add_scale_to_mesh_name(mesh_name=mesh_file_name, mesh_scale=numpy.array(mesh_scale))
                mujoco_mesh_path = mujoco_meshes_prim.GetPath().AppendChild(mesh_name)
                if stage.GetPrimAtPath(mujoco_mesh_path).IsValid():
                    raise ValueError(f"Mesh {mesh_name} already exists.")
                mujoco_mesh = UsdMujoco.MujocoMesh.Define(stage, mujoco_mesh_path)
                mujoco_mesh.CreateFileAttr(tmp_mesh_file_path)
                mujoco_mesh.CreateScaleAttr(mesh_scale)

            for prim in mesh_stage.TraverseAll():
                if prim.IsA(UsdShade.Material):
                    material = UsdShade.Material(prim)
                    material_name = material.GetPrim().GetName()
                    if material_name in materials:
                        continue
                    materials[material_name] = {}
                    for surface_output in material.GetSurfaceOutputs():
                        connected_source = surface_output.GetConnectedSource()[0]
                        shader_prim = connected_source.GetPrim()
                        if shader_prim.IsA(UsdShade.Shader):
                            shader = UsdShade.Shader(shader_prim)
                            for shader_input in shader.GetInputs():
                                if shader_input.GetBaseName() == "diffuseColor":
                                    diffuse_color = shader_input.GetAttr().Get()
                                    materials[material_name]["diffuse_color"] = diffuse_color
                                elif shader_input.GetBaseName() == "opacity":
                                    opacity = shader_input.GetAttr().Get()
                                    materials[material_name]["opacity"] = opacity
                                elif shader_input.GetBaseName() == "emissiveColor":
                                    emissive_color = shader_input.GetAttr().Get()
                                    materials[material_name]["emissive_color"] = emissive_color
                                elif shader_input.GetBaseName() == "specular":
                                    specular = shader_input.GetAttr().Get()
                                    materials[material_name]["specular_color"] = specular

        for material_name, material_props in materials.items():
            mujoco_material_path = mujoco_materials_prim.GetPath().AppendChild(material_name)
            if stage.GetPrimAtPath(mujoco_material_path).IsValid():
                continue

            mujoco_material = UsdMujoco.MujocoMaterial.Define(stage, mujoco_material_path)
            rgba = Gf.Vec4f(1.0, 1.0, 1.0, 0.0)
            emission = 0.0
            specular = 0.0
            if material_props.get("diffuse_color") is not None and material_props.get("opacity") is not None:
                rgba = Gf.Vec4f(*material_props["diffuse_color"], material_props["opacity"])
                if "emissive_color" in material_props and all([c != 0.0 for c in material_props["diffuse_color"]]):
                    emissions = [material_props["emissive_color"][i] / material_props["diffuse_color"][i]
                                 for i in range(3)]
                    if emissions[0] == emissions[1] == emissions[2]:
                        emission = emissions[0]
            if material_props.get("specular_color") is not None:
                specular = material_props["specular_color"]
            mujoco_material.CreateRgbaAttr(rgba)
            mujoco_material.CreateEmissionAttr(emission)
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

        xform_prim = geom_builder.xform.GetPrim()
        geom_name = xform_prim.GetName()
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
            material_prim = xform_prim.GetStage().GetPrimAtPath(material_rel_path)
            material_name = material_prim.GetName()
            geom.set("material", material_name)

        if len(geom_builder.gprims) > 1:
            raise NotImplementedError(f"Geom {geom_builder.xform.GetPrim().GetName()} has "
                                      f"{len(geom_builder.gprims)} geom prims.")
        geom_prim = geom_builder.gprims[0]
        if geom_prim.GetPrim().HasAPI(UsdPhysics.CollisionAPI):
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

    def export(self):
        os.makedirs(name=os.path.dirname(self.file_path), exist_ok=True)

        rough_string = ET.tostring(self.root, "utf-8")
        parsed_string = minidom.parseString(rough_string)
        pretty_string = parsed_string.toprettyxml()

        with open(self.file_path, "w", encoding="utf-8") as file:
            file.write(pretty_string)

        new_mesh_dir = os.path.join(os.path.dirname(self.file_path), self.file_name)
        tmp_mesh_dir = os.path.join(os.path.dirname(self.factory.tmp_usd_file_path), "tmp")

        copy_and_overwrite(source_folder=tmp_mesh_dir, destination_folder=new_mesh_dir, excludes=["usd"])

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
