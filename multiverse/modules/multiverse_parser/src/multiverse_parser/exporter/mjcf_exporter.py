#!/usr/bin/env python3

import os
# from math import radians, isclose
from xml.etree import ElementTree as ET
from xml.dom import minidom

from pxr import UsdMujoco, Gf, UsdPhysics, UsdGeom

from ..factory import Factory
from ..factory import (WorldBuilder,
                       BodyBuilder,
                       JointBuilder, JointType,
                       GeomBuilder, GeomType)
from ..importer.mjcf_importer import build_mujoco_inertial_api

class MjcfExporter:
    def __init__(
            self,
            factory: Factory,
            file_path: str
    ) -> None:
        self._factory = factory
        self._file_path = file_path
        self._meshdir_abs = os.path.join(os.path.dirname(self.file_path), self.file_name)
        self._root = ET.Element("mujoco")
        self._body_dict = {}

    def build(self) -> None:
        self._build_config()

    #         self.asset = ET.SubElement(self.root, "asset")

        worldbody = ET.SubElement(self.root, "worldbody")
        self.body_dict["worldbody"] = worldbody

        world_builder = self.factory.world_builder
        first_body_name = world_builder.body_builders[0].xform.GetPrim.GetName()
        if first_body_name == "world":
            self.body_dict["world"] = worldbody
            # for geom_name in body_dict["world"].geom_names:
            #     self.build_geom(geom_name, worldbody)
        else:
            self._build_body(body_name=first_body_name, parent_body_name="worldbody")

    #         body_names = self.world_builder.body_names
    #         reduces_body_names = body_names
    #
    #         stop = False
    #         while not stop:
    #             stop = True
    #             for body_name in body_names:
    #                 body_builder = body_dict[body_name]
    #                 parent_body_name = body_builder.xform.GetPrim().GetParent().GetName()
    #                 if parent_body_name in self.body_dict and body_name not in self.body_dict and len(body_builder.joint_names) == 0:
    #                     stop = False
    #                     self.build_link(body_name=body_name, parent_body_name=parent_body_name)
    #                     reduces_body_names.remove(body_name)
    #             for joint_name, joint_builder in joint_dict.items():
    #                 parent_body_name = joint_builder.parent_prim.GetPrim().GetName()
    #                 child_body_name = joint_builder.child_prim.GetPrim().GetName()
    #                 if parent_body_name in self.body_dict and child_body_name not in self.body_dict:
    #                     stop = False
    #                     self.build_link(body_name=child_body_name, parent_body_name=parent_body_name)
    #                     if with_physics:
    #                         self.build_joint(joint_name=joint_name, body_name=child_body_name)
    #                     reduces_body_names.remove(child_body_name)
    #             body_names = reduces_body_names
    #
    #         self.export()
    #
    def _build_body(self, body_name: str, parent_body_name: str) -> None:
            parent_body = self.body_dict[parent_body_name]
            body = ET.SubElement(parent_body, "body")
            self.body_dict[body_name] = body

            body.set("name", body_name)

            world_builder = self.factory.world_builder
            body_builder = world_builder.get_body_builder(body_name)

            xform_prim = body_builder.xform.GetPrim()
            if self.factory.config.with_physics and xform_prim.HasAPI(UsdPhysics.MassAPI):
                inertial = ET.SubElement(body, "inertial")
                if not xform_prim.HasAPI(UsdMujoco.MujocoBodyInertialAPI):
                    physics_mass_api = UsdPhysics.MassAPI(xform_prim)
                    mujoco_body_inertial_api = build_mujoco_inertial_api(physics_mass_api)
                else:
                    mujoco_body_inertial_api = UsdMujoco.MujocoBodyInertialAPI(xform_prim)

    #             if physics_mass_api.GetCenterOfMassAttr().Get() is None:
    #                 inertial.set("pos", "0 0 0")
    #             else:
    #                 inertial.set(
    #                     "pos",
    #                     " ".join(map(str, physics_mass_api.GetCenterOfMassAttr().Get())),
    #                 )
    #             if physics_mass_api.GetMassAttr().Get() is not None:
    #                 inertial.set("mass", str(physics_mass_api.GetMassAttr().Get()))
    #             if physics_mass_api.GetDiagonalInertiaAttr().Get() is not None:
    #                 inertial.set(
    #                     "diaginertia",
    #                     " ".join(map(str, physics_mass_api.GetDiagonalInertiaAttr().Get())),
    #                 )
    #
    #         if parent_body_name == "worldbody":
    #             body_relative_transform = xform_cache.GetLocalToWorldTransform(body_dict[body_name].xform.GetPrim())
    #         else:
    #             parent_body_transform = xform_cache.GetLocalToWorldTransform(body_dict[parent_body_name].xform.GetPrim())
    #             body_transformation = xform_cache.GetLocalToWorldTransform(body_dict[body_name].xform.GetPrim())
    #             body_relative_transform = body_transformation * parent_body_transform.GetInverse()
    #         body_relative_transform = body_relative_transform.RemoveScaleShear()
    #         body_relative_xyz = body_relative_transform.ExtractTranslation()
    #         body_relative_quat = body_relative_transform.ExtractRotationQuat()
    #         body.set("pos", " ".join(map(str, body_relative_xyz)))
    #         body.set(
    #             "quat",
    #             " ".join(
    #                 map(
    #                     str,
    #                     (
    #                         body_relative_quat.GetReal(),
    #                         body_relative_quat.GetImaginary()[0],
    #                         body_relative_quat.GetImaginary()[1],
    #                         body_relative_quat.GetImaginary()[2],
    #                     ),
    #                 )
    #             ),
    #         )
    #
    #         for geom_name in body_dict[body_name].geom_names:
    #             self.build_geom(geom_name, body)
    #
    #     def build_geom(self, geom_name: str, body) -> None:
    #         geom_builder = geom_dict[geom_name]
    #         geom_transformation = geom_builder.xform.GetLocalTransformation().RemoveScaleShear()
    #         geom_xyz = geom_transformation.ExtractTranslation()
    #         geom_quat = geom_transformation.ExtractRotationQuat()
    #
    #         geom = ET.SubElement(body, "geom")
    #         geom.set("name", geom_name)
    #
    #         geom.set("pos", " ".join(map(str, geom_xyz)))
    #         geom.set(
    #             "quat",
    #             " ".join(
    #                 map(
    #                     str,
    #                     (
    #                         geom_quat.GetReal(),
    #                         geom_quat.GetImaginary()[0],
    #                         geom_quat.GetImaginary()[1],
    #                         geom_quat.GetImaginary()[2],
    #                     ),
    #                 )
    #             ),
    #         )
    #
    #         if geom_builder.is_visual:
    #             geom.set("class", "visual")
    #         else:
    #             geom.set("class", "collision")
    #
    #         if geom_builder.type == GeomType.CUBE:
    #             geom.set("type", "box")
    #             geom.set(
    #                 "size",
    #                 " ".join(
    #                     map(
    #                         str,
    #                         [geom_builder.xform.GetLocalTransformation().GetRow(i).GetLength() for i in range(3)],
    #                     )
    #                 ),
    #             )
    #         elif geom_builder.type == GeomType.SPHERE:
    #             geom.set("type", "sphere")
    #             geom.set("size", str(geom_builder.geom.GetRadiusAttr().Get()))
    #         elif geom_builder.type == GeomType.CYLINDER:
    #             geom.set("type", "cylinder")
    #             geom.set(
    #                 "size",
    #                 " ".join(
    #                     map(
    #                         str,
    #                         [
    #                             geom_builder.geom.GetRadiusAttr().Get(),
    #                             geom_builder.geom.GetHeightAttr().Get() / 2,
    #                         ],
    #                     )
    #                 ),
    #             )
    #         elif geom_builder.type == GeomType.MESH:
    #             geom.set("type", "mesh")
    #             if geom_builder.mesh_builder is None:
    #                 print(f"Mesh builder for {str(geom_builder)} not found.")
    #                 return
    #
    #             mesh_builder = geom_builder.mesh_builder
    #             clear_meshes()
    #
    #             import_usd(mesh_builder.usd_file_path)
    #
    #             mesh_file_name = os.path.splitext(os.path.basename(mesh_builder.usd_file_path))[0]
    #
    #             geom_suffix = "".join(map(str, geom_builder.scale)).replace(".", "d").replace("-", "_") if any([not isclose(x, 1.0) for x in geom_builder.scale]) else ""
    #
    #             if self.with_visual and geom_builder.is_visual:
    #                 mesh_rel_path = os.path.join(
    #                     "obj",
    #                     mesh_file_name + geom_suffix + ".obj",
    #                 )
    #
    #                 mesh_file_name_visual = mesh_file_name + "_visual" + geom_suffix
    #
    #                 if mesh_rel_path not in self.mesh_rel_paths:
    #                     self.mesh_rel_paths.add(mesh_rel_path)
    #
    #                     scale = rotate_vector_by_quat(vector=geom_builder.scale, quat=geom_quat)
    #                     if not any(x < 0 for x in geom_builder.scale):
    #                         scale = tuple(abs(x) for x in scale)
    #                     if not any(x > 0 for x in geom_builder.scale):
    #                         scale = tuple(-abs(x) for x in scale)
    #
    #                     transform(scale=scale)
    #
    #                     texture_file_names = export_obj(os.path.join(self.mjcf_file_dir, mesh_rel_path))
    #
    #                     if len(texture_file_names) > 0:
    #                         texture_file_name = texture_file_names[0]
    #                         texture = ET.SubElement(self.asset, "texture")
    #                         texture.set("name", mesh_file_name_visual)
    #                         texture.set("type", "2d")
    #                         texture.set("file", texture_file_name)
    #
    #                         material = ET.SubElement(self.asset, "material")
    #                         material.set("name", mesh_file_name_visual)
    #                         material.set("texture", mesh_file_name_visual)
    #
    #                         geom.set("material", mesh_file_name_visual)
    #
    #                     mesh = ET.SubElement(self.asset, "mesh")
    #                     mesh.set("name", mesh_file_name_visual)
    #                     mesh.set("file", mesh_rel_path)
    #
    #                 geom.set("mesh", mesh_file_name_visual)
    #
    #             if self.with_collision and not geom_builder.is_visual:
    #                 mesh_rel_path = os.path.join(
    #                     "stl",
    #                     mesh_file_name + geom_suffix + ".stl",
    #                 )
    #
    #                 mesh_file_name_collision = mesh_file_name + "_collision" + geom_suffix
    #
    #                 if mesh_rel_path not in self.mesh_rel_paths:
    #                     self.mesh_rel_paths.add(mesh_rel_path)
    #
    #                     scale = rotate_vector_by_quat(vector=geom_builder.scale, quat=geom_quat)
    #                     if not any(x < 0 for x in geom_builder.scale):
    #                         scale = tuple(abs(x) for x in scale)
    #                     if not any(x > 0 for x in geom_builder.scale):
    #                         scale = tuple(-abs(x) for x in scale)
    #
    #                     transform(scale=scale)
    #
    #                     export_stl(os.path.join(self.mjcf_file_dir, mesh_rel_path))
    #
    #                     mesh = ET.SubElement(self.asset, "mesh")
    #                     mesh.set("name", mesh_file_name_collision)
    #                     mesh.set("file", mesh_rel_path)
    #
    #                 geom.set("mesh", mesh_file_name_collision)
    #
    #     def build_joint(self, joint_name: str, body_name: str) -> None:
    #         body = self.body_dict[body_name]
    #         joint = ET.SubElement(body, "joint")
    #         joint.set("name", joint_name)
    #
    #         joint_builder = joint_dict[joint_name]
    #         if joint_builder.type == JointType.NONE or joint_builder.type == JointType.FIXED:
    #             return
    #
    #         joint.set("pos", " ".join(map(str, joint_builder._pos)))
    #
    #         if joint_builder.type == JointType.PRISMATIC or joint_builder.type == JointType.REVOLUTE:
    #             if joint_builder.type == JointType.PRISMATIC:
    #                 joint.set("type", "slide")
    #                 lower = joint_builder.joint.GetLowerLimitAttr().Get()
    #                 upper = joint_builder.joint.GetUpperLimitAttr().Get()
    #             elif joint_builder.type == JointType.REVOLUTE:
    #                 joint.set("type", "hinge")
    #                 lower = radians(joint_builder.joint.GetLowerLimitAttr().Get())
    #                 upper = radians(joint_builder.joint.GetUpperLimitAttr().Get())
    #             joint.set("range", str(lower) + " " + str(upper))
    #         elif joint_builder.type == JointType.SPHERICAL:
    #             joint.set("type", "ball")
    #
    #         if joint_builder.type != JointType.SPHERICAL:
    #             axis = rotate_vector_by_quat((0, 0, 1), joint_builder._quat)
    #             joint.set("axis", " ".join(map(str, axis)))
    #
    def _build_config(self):
        stage = self.factory.world_builder.stage
        usd_mujoco_prim = stage.GetPrimAtPath("/mujoco")
        if not usd_mujoco_prim.IsValid():
            usd_mujoco = UsdMujoco.Mujoco.Define(stage, "/mujoco")
            model_name = stage.GetDefaultPrim().GetName()
            usd_mujoco.CreateModelAttr(model_name)
            usd_mujoco_prim = usd_mujoco.GetPrim()

        if not usd_mujoco_prim.HasAPI(UsdMujoco.MujocoOptionAPI):
            UsdMujoco.MujocoOptionAPI.Apply(usd_mujoco_prim)

        mujoco_asset_prim = stage.GetPrimAtPath("/mujoco/asset")
        if not mujoco_asset_prim.IsValid():
            UsdMujoco.MujocoAsset.Define(stage, "/mujoco/asset")

        model_name = UsdMujoco.Mujoco(usd_mujoco_prim).GetModelAttr().Get()
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

    def export(self):
        os.makedirs(name=os.path.dirname(self.file_path), exist_ok=True)

        rough_string = ET.tostring(self.root, "utf-8")
        parsed_string = minidom.parseString(rough_string)
        pretty_string = parsed_string.toprettyxml()

        with open(self.file_path, "w", encoding="utf-8") as file:
            file.write(pretty_string)

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
    def meshdir_abs(self) -> str:
        return self._meshdir_abs

    @property
    def root(self) -> ET.Element:
        return self._root

    @property
    def body_dict(self) -> dict:
        return self._body_dict