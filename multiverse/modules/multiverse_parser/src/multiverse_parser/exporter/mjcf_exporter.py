#!/usr/bin/env python3.10

import os
from xml.etree import ElementTree as ET
from xml.dom import minidom
from pxr import UsdPhysics
from multiverse_parser import WorldBuilder, GeomType, JointType
from multiverse_parser.factory.body_builder import body_dict
from multiverse_parser.factory.joint_builder import joint_dict
from multiverse_parser.factory.geom_builder import geom_dict
from multiverse_parser.utils import *


class MjcfExporter:
    def __init__(
        self,
        mjcf_file_path: str,
        world_builder: WorldBuilder,
        with_physics: bool,
        with_visual: bool,
        with_collision: bool,
    ) -> None:
        self.mjcf_file_path = mjcf_file_path
        mjcf_file_name = os.path.splitext(os.path.basename(self.mjcf_file_path))[0]
        self.mjcf_file_dir = os.path.join(
            os.path.dirname(self.mjcf_file_path),
            mjcf_file_name,
        )
        self.world_builder = world_builder
        self.with_physics = with_physics
        self.with_visual = with_visual
        self.with_collision = with_collision
        self.mesh_rel_paths = set()

        self.root = ET.Element("mujoco")
        self.root.set("model", self.world_builder.stage.GetDefaultPrim().GetName())

        self.compiler = ET.SubElement(self.root, "compiler")
        self.compiler.set("meshdir", mjcf_file_name + "/")
        texturedir = os.path.join(mjcf_file_name, "obj", "textures")
        self.compiler.set("texturedir", texturedir)
        self.compiler.set("angle", "radian")
        self.compiler.set("autolimits", "true")
        self.compiler.set("balanceinertia", "true")
        self.compiler.set("boundmass", "0.000001")
        self.compiler.set("boundinertia", "0.000001")

        self.default = ET.SubElement(self.root, "default")
        default_visual = ET.SubElement(self.default, "default")
        default_visual.set("class", "visual")
        default_visual_geom = ET.SubElement(default_visual, "geom")
        default_visual_geom.set("contype", "0")
        default_visual_geom.set("conaffinity", "0")

        default_collision = ET.SubElement(self.default, "default")
        default_collision.set("class", "collision")
        default_collision_geom = ET.SubElement(default_collision, "geom")
        default_collision_geom.set("rgba", "1.0 0.0 0.0 0.5")

        self.asset = ET.SubElement(self.root, "asset")
        floor_texture = ET.SubElement(self.asset, "texture")
        floor_texture.set("name", "grid")
        floor_texture.set("type", "2d")
        floor_texture.set("builtin", "checker")
        floor_texture.set("width", "512")
        floor_texture.set("height", "512")
        floor_texture.set("rgb1", ".1 .2 .3")
        floor_texture.set("rgb2", ".2 .3 .4")
        floor_material = ET.SubElement(self.asset, "material")
        floor_material.set("name", "grid")
        floor_material.set("texture", "grid")
        floor_material.set("texrepeat", "1 1")
        floor_material.set("texuniform", "true")

        worldbody = ET.SubElement(self.root, "worldbody")
        floor_geom = ET.SubElement(worldbody, "geom")
        floor_geom.set("name", "floor")
        floor_geom.set("size", "0 0 0.05")
        floor_geom.set("type", "plane")
        floor_geom.set("material", "grid")
        floor_geom.set("condim", "4")
        floor_geom.set("friction", "2 0.05 0.01")
        light = ET.SubElement(worldbody, "light")
        light.set("diffuse", ".5 .5 .5")
        light.set("pos", "0 0 5")
        light.set("dir", "0 0 -1")

        self.body_dict = {}

        self.worldbody = ET.SubElement(self.root, "worldbody")
        self.body_dict[self.world_builder.body_names[0]] = self.worldbody

        stop = False
        while not stop:
            stop = True
            for body_name in self.world_builder.body_names:
                body_builder = body_dict[body_name]
                parent_body_name = body_builder.xform.GetPrim().GetParent().GetName()
                if parent_body_name in self.body_dict and body_name not in self.body_dict and len(body_builder.joint_names) == 0:
                    stop = False
                    self.build_link(body_name=body_name, parent_body_name=parent_body_name)

            for joint_name, joint_builder in joint_dict.items():
                parent_body_name = joint_builder.parent_xform.GetPrim().GetName()
                child_body_name = joint_builder.child_xform.GetPrim().GetName()
                if parent_body_name in self.body_dict and child_body_name not in self.body_dict:
                    stop = False
                    self.build_link(body_name=child_body_name, parent_body_name=parent_body_name)

                    if with_physics:
                        self.build_joint(joint_name=joint_name, body_name=child_body_name)

        self.export()

    def build_link(self, body_name: str, parent_body_name: str) -> None:
        parent_body = self.body_dict[parent_body_name]
        body = ET.SubElement(parent_body, "body")
        self.body_dict[body_name] = body

        body.set("name", body_name)

        body_builder = body_dict[body_name]
        if self.with_physics and body_builder.xform.GetPrim().HasAPI(UsdPhysics.MassAPI):
            inertial = ET.SubElement(body, "inertial")
            physics_mass_api = UsdPhysics.MassAPI(body_builder.xform)

            if physics_mass_api.GetCenterOfMassAttr().Get() is None:
                inertial.set("pos", "0 0 0")
            else:
                inertial.set("pos", " ".join(map(str, physics_mass_api.GetCenterOfMassAttr().Get())))
            if physics_mass_api.GetMassAttr().Get() is not None:
                inertial.set("mass", str(physics_mass_api.GetMassAttr().Get()))
            if physics_mass_api.GetDiagonalInertiaAttr().Get() is not None:
                inertial.set(
                    "diaginertia",
                    " ".join(map(str, physics_mass_api.GetDiagonalInertiaAttr().Get())),
                )

        parent_body_transform = xform_cache.GetLocalToWorldTransform(body_dict[parent_body_name].xform.GetPrim())
        body_transformation = xform_cache.GetLocalToWorldTransform(body_dict[body_name].xform.GetPrim())
        body_relative_transform = body_transformation * parent_body_transform.GetInverse()
        body_relative_xyz = body_relative_transform.ExtractTranslation()
        body_relative_quat = body_relative_transform.ExtractRotationQuat()
        body.set("pos", " ".join(map(str, body_relative_xyz)))
        body.set(
            "quat",
            " ".join(
                map(
                    str,
                    (
                        body_relative_quat.GetReal(),
                        body_relative_quat.GetImaginary()[0],
                        body_relative_quat.GetImaginary()[1],
                        body_relative_quat.GetImaginary()[2],
                    ),
                )
            ),
        )

        for geom_name in body_dict[body_name].geom_names:
            geom_builder = geom_dict[geom_name]
            geom_transformation = geom_builder.xform.GetLocalTransformation()
            geom_xyz = geom_transformation.ExtractTranslation()
            geom_quat = geom_transformation.ExtractRotationQuat()

            geom = ET.SubElement(body, "geom")
            geom.set("name", geom_name)

            if geom_builder.type != GeomType.MESH:
                geom.set("pos", " ".join(map(str, geom_xyz)))
                geom.set(
                    "quat",
                    " ".join(
                        map(
                            str,
                            (
                                geom_quat.GetReal(),
                                geom_quat.GetImaginary()[0],
                                geom_quat.GetImaginary()[1],
                                geom_quat.GetImaginary()[2],
                            ),
                        )
                    ),
                )

            if geom_builder.is_visual:
                geom.set("class", "visual")
            else:
                geom.set("class", "collision")

            if geom_builder.type == GeomType.CUBE:
                geom.set("type", "box")
                geom.set(
                    "size",
                    " ".join(
                        map(
                            str,
                            [geom_builder.xform.GetLocalTransformation().GetRow(i).GetLength() for i in range(3)],
                        )
                    ),
                )
            elif geom_builder.type == GeomType.SPHERE:
                geom.set("type", "sphere")
                geom.set("size", str(geom_builder.geom.GetRadiusAttr().Get()))
            elif geom_builder.type == GeomType.CYLINDER:
                geom.set("type", "cylinder")
                geom.set(
                    "size",
                    " ".join(
                        map(
                            str,
                            [
                                geom_builder.geom.GetRadiusAttr().Get(),
                                geom_builder.geom.GetHeightAttr().Get() / 2,
                            ],
                        )
                    ),
                )
            elif geom_builder.type == GeomType.MESH:
                geom.set("type", "mesh")
                if len(geom_builder.mesh_builders) > 1:
                    print(f"More than 1 mesh exists in geom {geom_name}, take the first one.")
                elif len(geom_builder.mesh_builders) == 0:
                    print(f"Mesh not found in geom {geom_name}")

                mesh_builder = geom_builder.mesh_builders[0]
                clear_meshes()

                import_usd(mesh_builder.usd_file_path)
                geom_rpy = quat_to_rpy(geom_quat)
                transform(xyz=geom_xyz, rpy=geom_rpy)

                mesh_file_name = os.path.splitext(os.path.basename(mesh_builder.usd_file_path))[0]
                if self.with_visual and geom_builder.is_visual:
                    mesh_rel_path = os.path.join(
                        "obj",
                        mesh_file_name + ".obj",
                    )

                    mesh_file_name_visual = mesh_file_name + "_visual"

                    if mesh_rel_path not in self.mesh_rel_paths:
                        texture_file_names = export_obj(os.path.join(self.mjcf_file_dir, mesh_rel_path))
                        self.mesh_rel_paths.add(mesh_rel_path)

                        if len(texture_file_names) > 0:
                            texture_file_name = texture_file_names[0]
                            texture = ET.SubElement(self.asset, "texture")
                            texture.set("name", mesh_file_name_visual)
                            texture.set("type", "2d")
                            texture.set("file", texture_file_name)

                            material = ET.SubElement(self.asset, "material")
                            material.set("name", mesh_file_name_visual)
                            material.set("texture", mesh_file_name_visual)

                            geom.set("material", mesh_file_name_visual)

                        mesh = ET.SubElement(self.asset, "mesh")
                        mesh.set("name", mesh_file_name_visual)
                        mesh.set("file", mesh_rel_path)

                    geom.set("mesh", mesh_file_name_visual)

                if self.with_collision and not geom_builder.is_visual:
                    mesh_rel_path = os.path.join(
                        "stl",
                        mesh_file_name + ".stl",
                    )
                    if mesh_rel_path not in self.mesh_rel_paths:
                        export_stl(os.path.join(self.mjcf_file_dir, mesh_rel_path))
                        self.mesh_rel_paths.add(mesh_rel_path)
                        mesh = ET.SubElement(self.asset, "mesh")
                        mesh.set("name", mesh_file_name + "_collision")
                        mesh.set("file", mesh_rel_path)
                    geom.set("mesh", mesh_file_name + "_collision")

    def build_joint(self, joint_name: str, body_name: str) -> None:
        body = self.body_dict[body_name]
        joint = ET.SubElement(body, "joint")
        joint.set("name", joint_name)

        joint_builder = joint_dict[joint_name]
        if joint_builder.type == JointType.NONE or joint_builder.type == JointType.FIXED:
            return

        joint.set("pos", " ".join(map(str, joint_builder.pos)))

        if joint_builder.type == JointType.PRISMATIC or joint_builder.type == JointType.REVOLUTE:
            if joint_builder.type == JointType.PRISMATIC:
                joint.set("type", "slide")
            elif joint_builder.type == JointType.REVOLUTE:
                joint.set("type", "hinge")
            lower = joint_builder.joint.GetLowerLimitAttr().Get()
            upper = joint_builder.joint.GetUpperLimitAttr().Get()
            joint.set("range", str(lower) + " " + str(upper))
        elif joint_builder.type == JointType.SPHERICAL:
            joint.set("type", "ball")

        if joint_builder.type != JointType.SPHERICAL:
            if joint_builder.axis == "X":
                joint.set("axis", "1 0 0")
            elif joint_builder.axis == "Y":
                joint.set("axis", "0 1 0")
            elif joint_builder.axis == "Z":
                joint.set("axis", "0 0 1")
            elif joint_builder.axis == "-X":
                joint.set("axis", "-1 0 0")
            elif joint_builder.axis == "-Y":
                joint.set("axis", "0 -1 0")
            elif joint_builder.axis == "-Z":
                joint.set("axis", "0 0 -1")

    def export(self):
        os.makedirs(name=os.path.dirname(self.mjcf_file_path), exist_ok=True)

        rough_string = ET.tostring(self.root, "utf-8")
        reparsed = minidom.parseString(rough_string)
        pretty_xml_str = reparsed.toprettyxml(indent="\t")

        with open(self.mjcf_file_path, "w", encoding="utf-8") as file:
            file.write(pretty_xml_str)