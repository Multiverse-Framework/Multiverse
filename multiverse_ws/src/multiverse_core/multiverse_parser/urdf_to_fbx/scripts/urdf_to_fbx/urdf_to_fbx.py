#!/usr/bin/env python3.10

import os
from shutil import copy, copytree
from typing import Dict, List, Tuple, Union
from xml.etree import ElementTree

import bpy
import rospkg
from bpy.types import (
    Armature,
    BlendData,
    Bone,
    Camera,
    Image,
    Light,
    Material,
    Mesh,
    Object,
)
from mathutils import Euler, Vector
from urdf_parser_py.urdf import URDF, Joint, Link, Visual

TMP_FOLDER_PATH = "texture/"
TMP_TEXTURE_PATH = TMP_FOLDER_PATH
TMP_FILE_PATH = "tmp.dae"


def urdf_cleanup(file_path: str) -> str:
    tree = ElementTree.parse(file_path)
    root = tree.getroot()

    newroot = ElementTree.Element(root.tag)
    newroot.set("name", root.get("name"))

    for element in root:
        if element.tag == "link" or element.tag == "joint" or element.tag == "material":
            newroot.append(element)

    return ElementTree.tostring(newroot)


def fix_up_axis_and_get_materials(file_path: str, unique_name: bool):
    tree = ElementTree.parse(file_path)
    root = tree.getroot()

    tmp_file_path = file_path
    dir_path = os.path.dirname(file_path)
    mat_sampler2D_dict: Dict[str, Dict[str, str]] = {}

    mat_dict: Dict[str, str] = {}
    effect_dict: Dict[str, List[str]] = {}
    sampler2D_dict: Dict[str, str] = {}
    surface_dict: Dict[str, str] = {}
    image_dict: Dict[str, str] = {}
    if not os.path.exists(TMP_TEXTURE_PATH):
        os.makedirs(TMP_TEXTURE_PATH)

    for ele1 in root:
        if "asset" in ele1.tag:
            for ele2 in ele1:
                if "up_axis" in ele2.tag:
                    ele2.text = "Z_UP"
                    tmp_file_path = TMP_FILE_PATH

        if "library_materials" in ele1.tag:
            for ele2 in ele1:
                if "material" in ele2.tag:
                    mat_name = ele2.attrib["name"]
                    for ele3 in ele2:
                        if "instance_effect" in ele3.tag:
                            effect_id = ele3.attrib["url"]
                            if effect_id.startswith("#"):
                                effect_id = effect_id[1:]
                            mat_dict[mat_name] = effect_id

        if "library_effects" in ele1.tag:
            for ele2 in ele1:
                if "effect" in ele2.tag:
                    effect_id = ele2.attrib["id"]
                    effect_dict[effect_id] = []
                    for ele3 in ele2:
                        if "profile_COMMON" in ele3.tag:
                            for ele4 in ele3:
                                if "newparam" in ele4.tag:
                                    param_name = ele4.attrib["sid"]
                                    for ele5 in ele4:
                                        if "surface" in ele5.tag:
                                            for ele6 in ele5:
                                                if "init_from" in ele6.tag:
                                                    surface_dict[param_name] = ele6.text
                                        if "sampler2D" in ele5.tag:
                                            for ele6 in ele5:
                                                if "source" in ele6.tag:
                                                    effect_dict[effect_id].append(
                                                        param_name
                                                    )
                                                    sampler2D_dict[
                                                        param_name
                                                    ] = ele6.text

        if "library_images" in ele1.tag:
            for ele2 in ele1:
                if "image" in ele2.tag:
                    image_name = ele2.attrib["name"]
                    for ele3 in ele2:
                        if "init_from" in ele3.tag:
                            tmp_file_path = TMP_FILE_PATH
                            file_name, file_ext = os.path.splitext(ele3.text)
                            if not unique_name:
                                file_hash = str(
                                    abs(hash(os.path.dirname(file_path))) % (10**3)
                                )
                                file = "T_" + file_name + "_" + file_hash + file_ext
                            else:
                                file = "T_" + file_name + file_ext
                            copy(dir_path + "/" + ele3.text, TMP_TEXTURE_PATH + file)
                            ele3.text = TMP_TEXTURE_PATH + file
                            image_dict[image_name] = ele3.text

    for mat_name in mat_dict:
        mat_sampler2D_dict[mat_name] = {}
        effect_id = mat_dict[mat_name]
        for effect_name in effect_dict[effect_id]:
            sampler2D_name = sampler2D_dict.get(effect_name)
            image_name = surface_dict.get(sampler2D_name)
            image_path = image_dict.get(image_name)
            mat_sampler2D_dict[mat_name][effect_name] = image_path

    if tmp_file_path == TMP_FILE_PATH:
        tree.write(tmp_file_path)

    return (tmp_file_path, mat_sampler2D_dict)


def clean_up() -> None:
    if os.path.exists(TMP_FILE_PATH):
        os.remove(TMP_FILE_PATH)
    return None


def clear_data(data: BlendData, scale_unit: float) -> None:
    armature: Armature
    for armature in data.armatures:
        data.armatures.remove(armature)
    mesh: Mesh
    for mesh in data.meshes:
        data.meshes.remove(mesh)
    object: Object
    for object in data.objects:
        data.objects.remove(object)
    material: Material
    for material in data.materials:
        data.materials.remove(material)
    camera: Camera
    for camera in data.cameras:
        data.cameras.remove(camera)
    light: Light
    for light in data.lights:
        data.lights.remove(light)
    image: Image
    for image in data.images:
        data.images.remove(image)

    unit_settings = bpy.context.scene.unit_settings
    unit_settings.scale_length = scale_unit
    bpy.context.view_layer.update()

    return None


def merge_materials(should_check_material_name: bool) -> None:
    mat_uniques: List[Material] = []
    object: Object
    for object in bpy.data.objects:
        for material_slot in object.material_slots:
            mat = material_slot.material
            if mat is None or not mat.use_nodes:
                continue
            mat_base_color = mat.node_tree.nodes["Principled BSDF"].inputs.get(
                "Base Color"
            )
            is_mat_unique = True
            for mat_unique in mat_uniques:
                # Level 1: Check for equalness
                if mat == mat_unique:
                    continue

                if should_check_material_name:
                    # Level 2: Check for name equalness
                    mat_name_split = mat.name_full.split(".")
                    mat_unique_name_split = mat_unique.name_full.split(".")
                    if (
                        len(mat_name_split) == len(mat_unique_name_split)
                        and len(mat_name_split) > 1
                        and mat_name_split[-1].isnumeric()
                        and mat_unique_name_split[-1].isnumeric()
                    ):
                        mat_name_split.pop()
                        mat_unique_name_split.pop()
                    for mat_name, mat_unique_name in zip(
                        mat_name_split, mat_unique_name_split
                    ):
                        if mat_name[:59] == mat_unique_name[:59]:
                            is_mat_unique = False
                        else:
                            is_mat_unique = True
                            break
                    if is_mat_unique:
                        continue

                # Level 3: Check for content equalness
                mat_unique_base_color = mat_unique.node_tree.nodes[
                    "Principled BSDF"
                ].inputs.get("Base Color")
                if (not mat_base_color.is_linked) and (
                    not mat_unique_base_color.is_linked
                ):
                    # Merge duplicate materials based on their Base Color
                    if [i for i in mat_base_color.default_value] == [
                        i for i in mat_unique_base_color.default_value
                    ]:
                        object.material_slots[mat.name].material = mat_unique
                        bpy.data.materials.remove(mat)
                        is_mat_unique = False
                        break
                elif mat_base_color.is_linked and mat_unique_base_color.is_linked:
                    # Merge duplicate materials based on their image name
                    if (
                        mat_base_color.links[0].from_node.image.name
                        == mat_unique_base_color.links[0].from_node.image.name
                    ):
                        object.material_slots[mat.name].material = mat_unique
                        bpy.data.materials.remove(mat)
                        is_mat_unique = False
                        break

            if is_mat_unique and mat is not None:
                mat_name_split = mat.name_full.split(".")
                mat_unique = None
                while len(mat_name_split) > 1 and mat_name_split[-1].isnumeric():
                    mat_name = "".join(mat_name_split[:-1])
                    if bpy.data.materials.get(mat_name) is not None:
                        mat_unique = bpy.data.materials[mat_name]
                    mat_name_split.pop()

                if mat_unique is None:
                    mat_uniques.append(mat)
                else:
                    # Level 3: Check for content equalness
                    mat_unique_base_color = mat_unique.node_tree.nodes[
                        "Principled BSDF"
                    ].inputs.get("Base Color")
                    is_mat_unique_equal_mat = False
                    if (not mat_base_color.is_linked) and (
                        not mat_unique_base_color.is_linked
                    ):
                        # Merge duplicate materials based on their Base Color
                        if [i for i in mat_base_color.default_value] == [
                            i for i in mat_unique_base_color.default_value
                        ]:
                            is_mat_unique_equal_mat = True
                    elif mat_base_color.is_linked and mat_unique_base_color.is_linked:
                        # Merge duplicate materials based on their image name
                        if (
                            mat_base_color.links[0].from_node.image.name
                            == mat_unique_base_color.links[0].from_node.image.name
                        ):
                            is_mat_unique_equal_mat = True

                    if is_mat_unique_equal_mat:
                        object.material_slots[mat.name].material = mat_unique
                        bpy.data.materials.remove(mat)
                        mat_uniques.append(mat_unique)
                    else:
                        mat_uniques.append(mat)
        object.select_set(False)
    return None


def fix_alpha() -> None:
    for mat in bpy.data.materials:
        if hasattr(mat.node_tree, "nodes"):
            mat.node_tree.nodes["Principled BSDF"].inputs["Alpha"].default_value = 1.0


def rename_materials(base_name: str) -> None:
    for object in bpy.data.objects:
        for material_slot in object.material_slots:
            if material_slot.material is not None:
                material_slot.material.name = "M_" + base_name
    return None


class RobotBuilder:
    def __init__(
        self,
        file_path: str,
        should_check_material_name: bool,
        should_rename_materials: bool,
        should_apply_weld: bool,
        unique_name: bool,
        scale_unit: float,
    ):
        xml_string = urdf_cleanup(file_path)
        self.robot: URDF = URDF.from_xml_string(xml_string)
        self.link_pose: Dict[str, Tuple[Vector, Euler]] = {}
        self.arm_bones: Dict[str, Bone] = {}
        self.root: Object = None
        self.root_name = "root"
        self.bone_tail = ".bone"
        self.parent_links = None
        self.apply_weld = should_apply_weld
        self.unique_name = unique_name
        self.scale_unit = scale_unit
        self.build_robot()
        merge_materials(should_check_material_name)
        if should_rename_materials:
            rename_materials(self.robot.name)
        clean_up()

    def build_robot(self) -> None:
        clear_data(bpy.data, self.scale_unit)
        self.create_materials()
        self.konfigure_mesh_path()
        self.add_root_armature()
        self.build_root()
        self.build_chain()
        fix_alpha()
        return None

    def create_materials(self) -> None:
        for material in self.robot.materials:
            if material.color is not None and hasattr(material.color, "rgba"):
                if bpy.data.materials.get(material.name):
                    print("Material", material.name, "already exists")
                else:
                    mat: Material = bpy.data.materials.new(name=material.name)
                    mat.diffuse_color = material.color.rgba
        return None

    def konfigure_mesh_path(self) -> None:
        link: Link
        for link in self.robot.links:
            visual: Visual
            for visual in link.visuals:
                if hasattr(visual.geometry, "filename"):
                    rel_path: str = visual.geometry.filename
                    while os.path.dirname(rel_path) != "package:":
                        rel_path = os.path.dirname(rel_path)
                    pkg_name = os.path.basename(rel_path)
                    pkg_path = rospkg.RosPack().get_path(pkg_name)
                    abs_path = os.path.dirname(
                        pkg_path
                    ) + visual.geometry.filename.replace("package://", "/")
                    if os.path.exists(abs_path):
                        visual.geometry.filename = abs_path
        return None

    def add_root_armature(self) -> None:
        arm: Armature = bpy.data.armatures.new("armatures")
        self.arm_bones = arm.bones
        self.root = bpy.data.objects.new(self.root_name, arm)
        self.root.show_in_front = True
        bpy.context.scene.collection.objects.link(self.root)
        return None

    def add_mesh(
        self,
        mesh_name: str,
        material: Material = None,
        file_path: Union[str, List[str]] = "",
        location=Vector(),
        rotation=Euler(),
        scale=Vector((1, 1, 1)),
        link_pos=Vector(),
        link_rot=Euler(),
    ) -> Object:
        if isinstance(file_path, list):
            if file_path[0] == "cylinder":
                bpy.ops.mesh.primitive_cylinder_add(
                    depth=file_path[1], radius=file_path[2], scale=(1, 1, 1)
                )
            elif file_path[0] == "cube":
                bpy.ops.mesh.primitive_cube_add(size=1, scale=file_path[1])
            elif file_path[0] == "sphere":
                bpy.ops.mesh.primitive_uv_sphere_add(
                    radius=file_path[1], scale=(1, 1, 1)
                )
            else:
                print("Object type", file_path[0], "is not supported")
                return None
            object = bpy.context.object

            if material is None:
                material = bpy.data.materials.get("Material")
                if material is None:
                    material = bpy.data.materials.new(name="Material")
            object.data.materials.append(material)

        elif file_path:
            file_ext = os.path.splitext(file_path)[1].lower()
            if file_ext == ".dae":
                (file_path, _) = fix_up_axis_and_get_materials(
                    file_path, self.unique_name
                )
                bpy.ops.wm.collada_import(filepath=file_path)
            elif file_ext == ".obj":
                bpy.ops.import_scene.obj(
                    filepath=file_path, axis_forward="Y", axis_up="Z"
                )
            elif file_ext == ".stl":
                bpy.ops.import_mesh.stl(
                    filepath=file_path, global_scale=1 / self.scale_unit
                )

            else:
                print("File extension", file_ext, "of", file_path, "is not supported")
                return None
            camera: Camera
            for camera in bpy.data.cameras:
                bpy.data.cameras.remove(camera)
            light: Light
            for light in bpy.data.lights:
                bpy.data.lights.remove(light)
            bpy.context.view_layer.objects.active = bpy.context.selected_objects[0]
            if len(bpy.context.selected_objects) > 1:
                bpy.ops.object.join()
            if not bpy.context.object.data.uv_layers:
                bpy.ops.mesh.uv_texture_add()
            object = bpy.context.object
            if self.apply_weld:
                object.modifiers.new("Weld", "WELD")
                bpy.ops.object.modifier_apply(modifier="Weld")
            if material is not None:
                object.data.materials.append(material)

        else:
            mesh = bpy.data.meshes.new(mesh_name)
            mesh.uv_layers.new()
            object = bpy.data.objects.new(mesh_name, mesh)
            bpy.context.scene.collection.objects.link(object)

        object.name = mesh_name
        object.rotation_mode = "XYZ"
        object.rotation_euler.rotate(rotation)
        object.location.rotate(rotation)
        object.location += location
        object.scale *= scale

        # Change origin of mesh to link_pos and link_rot
        bpy.context.scene.cursor.location = link_pos
        bpy.context.scene.cursor.rotation_euler = link_rot
        bpy.ops.object.origin_set(type="ORIGIN_CURSOR")
        bpy.context.scene.cursor.location = Vector()
        bpy.context.scene.cursor.rotation_euler = Euler()

        # Apply 0.01 scale
        # object.scale *= 100
        bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
        # object.scale /= 100

        return object

    def set_link_origin(self, link: Link) -> None:
        if hasattr(link, "origin") and link.origin is not None:
            self.link_pose[link.name][0][:] = (
                self.link_pose[link.name][0] + Vector(link.origin.xyz) / self.scale_unit
            )
            self.link_pose[link.name][1].rotate(Euler(link.origin.rpy))
        return None

    def add_root_bone(self, link_name: str, bone_name: str) -> None:
        bpy.context.view_layer.objects.active = self.root
        bpy.ops.object.mode_set(mode="EDIT", toggle=False)

        head = self.link_pose[link_name][0]
        tail = Vector((0.0, 0.1 / self.scale_unit, 0.0))
        tail.rotate(self.link_pose[link_name][1])
        tail += head
        bone: Bone = self.root.data.edit_bones.new(bone_name)
        bone.head = head
        bone.tail = tail
        bpy.ops.object.mode_set(mode="OBJECT")
        return None

    def add_link_origin(
        self, pos: Vector, rot: Euler, tag: Union[Link, Joint, Visual]
    ) -> Tuple[Vector, Euler]:
        if hasattr(tag, "origin") and tag.origin is not None:
            pos_out = Vector(tag.origin.xyz) / self.scale_unit
            pos_out.rotate(rot)
            pos_out += pos
            rot_out = Euler(tag.origin.rpy)
            rot_out.rotate(rot)
            return (pos_out, rot_out)
        else:
            return (pos, rot)

    def get_link_data(
        self, link_pos: Vector, link_rot: Euler, link: Link, visual: Visual
    ):
        visual_pos, visual_rot = self.add_link_origin(link_pos, link_rot, visual)

        if hasattr(visual.geometry, "filename") and visual.geometry.filename:
            file_path = visual.geometry.filename
            mesh_name: str = link.name + "." + os.path.basename(file_path)
            if len(mesh_name) > 63:
                print(
                    "Mesh",
                    mesh_name,
                    "has more than 63 characters, the characters from 64 will be ignored",
                )
                mesh_name = mesh_name[0:63]
        else:
            if hasattr(visual.geometry, "length") and hasattr(
                visual.geometry, "radius"
            ):
                file_path = ["cylinder", visual.geometry.length, visual.geometry.radius]
                mesh_name = link.name + ".cylinder"
            elif hasattr(visual.geometry, "size"):
                file_path = ["cube", visual.geometry.size]
                mesh_name = link.name + ".cube"
            elif hasattr(visual.geometry, "radius"):
                file_path = ["sphere", visual.geometry.radius]
                mesh_name = link.name + ".sphere"
            else:
                file_path = ""
                mesh_name = link.name + ".empty"

        if hasattr(visual.geometry, "scale") and visual.geometry.scale:
            scale = Vector(visual.geometry.scale)
        else:
            scale = Vector((1, 1, 1))

        if hasattr(visual, "material") and hasattr(visual.material, "name"):
            material = bpy.data.materials.get(visual.material.name)
            if material is None:
                material = bpy.data.materials.new(visual.material.name)
                if (
                    hasattr(visual.material, "color")
                    and visual.material.color
                    and visual.material.color.rgba
                ):
                    material.use_nodes = True
                    principled_node = material.node_tree.nodes.get("Principled BSDF")
                    principled_node.inputs[0].default_value = visual.material.color.rgba
        else:
            material = None

        return (mesh_name, file_path, visual_pos, visual_rot, scale, material)

    def bind_mesh_to_bone(self, mesh_name: str, bone_name: str) -> None:
        bpy.ops.object.mode_set(mode="POSE")

        object = bpy.context.scene.objects.get(mesh_name)
        object.select_set(True)
        self.arm_bones.active = self.arm_bones[bone_name]
        self.arm_bones[bone_name].select = True
        bpy.ops.object.parent_set(type="BONE")
        object.select_set(False)
        self.arm_bones[bone_name].select = False

        bpy.ops.object.mode_set(mode="OBJECT")
        return None

    def add_bone(
        self,
        link: Link,
        joint: Joint,
        joint_pos: Vector,
        joint_rot: Euler,
        bone_name: str,
    ) -> None:
        bpy.context.view_layer.objects.active = self.root
        bpy.ops.object.mode_set(mode="EDIT", toggle=False)

        head = joint_pos
        tail = Vector((0.0, 0.0, 0.1 / self.scale_unit))
        if (
            hasattr(joint, "axis")
            and joint.axis is not None
            and Vector(joint.axis).magnitude != 0
        ):
            tail = Vector(joint.axis).normalized() * 0.1 / self.scale_unit
        tail.rotate(joint_rot)

        bone: Bone = self.root.data.edit_bones.new(bone_name)
        bone.head = head
        bone.tail = head + tail

        if self.robot.parent_map[link.name][1] == self.robot.get_root():
            bone.parent = self.root.data.edit_bones["root" + self.bone_tail]
        else:
            parent_joint = self.robot.parent_map[self.robot.parent_map[link.name][1]][0]
            parent_joint_name = (
                parent_joint
                + "."
                + str(self.robot.joint_map[parent_joint].type)
                + self.bone_tail
            )
            bone.parent = self.root.data.edit_bones[parent_joint_name]

        bpy.ops.object.mode_set(mode="OBJECT")
        return None

    def build_root(self) -> None:
        root_link: Link = self.robot.link_map[self.robot.get_root()]
        self.link_pose[root_link.name] = (Vector(), Euler())
        self.set_link_origin(root_link)

        if root_link.visuals:
            objects = []
            visual: Visual
            for visual in root_link.visuals:
                (
                    mesh_name,
                    file_path,
                    visual_pos,
                    visual_rot,
                    scale,
                    material,
                ) = self.get_link_data(
                    self.link_pose[root_link.name][0],
                    self.link_pose[root_link.name][1],
                    root_link,
                    visual,
                )

                pos_tmp = self.link_pose[root_link.name][0].copy()
                pos_tmp.rotate(self.link_pose[root_link.name][1])
                visual_pos += pos_tmp

                rot_tmp = self.link_pose[root_link.name][1].copy()
                rot_tmp.rotate(visual_rot)
                visual_rot = rot_tmp

                object = self.add_mesh(
                    mesh_name,
                    material,
                    file_path,
                    visual_pos,
                    visual_rot,
                    scale,
                    self.link_pose[root_link.name][0],
                    self.link_pose[root_link.name][1],
                )
                objects.append(object)

            for object in objects:
                object.select_set(True)

            if len(bpy.context.selected_objects) > 1:
                bpy.context.view_layer.objects.active = bpy.context.selected_objects[0]
                bpy.ops.object.join()

            bone_name = self.root_name + self.bone_tail
            self.add_root_bone(root_link.name, bone_name)

            objects[0].name = root_link.name
            self.bind_mesh_to_bone(root_link.name, bone_name)

        else:
            bone_name = self.root_name + self.bone_tail
            self.add_root_bone(root_link.name, bone_name)

        self.parent_links = [root_link]
        return None

    def build_chain(self) -> None:
        while self.robot.child_map:
            # Make new parent links
            links = self.parent_links

            # Iterate through all parent links
            for link in links:
                self.set_link_origin(link)

                # Iterate through all children of parent link
                if self.robot.child_map.get(link.name):
                    for child_map in self.robot.child_map[link.name]:
                        child_pos = self.link_pose[link.name][0].copy()
                        child_rot = self.link_pose[link.name][1].copy()

                        child_joint = self.robot.joint_map[child_map[0]]
                        child_pos, child_rot = self.add_link_origin(
                            child_pos, child_rot, child_joint
                        )
                        joint_pos = child_pos.copy()
                        joint_rot = child_rot.copy()

                        child_link = self.robot.link_map[child_map[1]]
                        child_pos, child_rot = self.add_link_origin(
                            child_pos, child_rot, child_link
                        )

                        self.link_pose[child_link.name] = (child_pos, child_rot)

                        if child_link.visuals:
                            visual: Visual
                            objects = []
                            for visual in child_link.visuals:
                                (
                                    mesh_name,
                                    file_path,
                                    visual_pos,
                                    visual_rot,
                                    scale,
                                    material,
                                ) = self.get_link_data(
                                    child_pos, child_rot, child_link, visual
                                )
                                object = self.add_mesh(
                                    mesh_name,
                                    material,
                                    file_path,
                                    visual_pos,
                                    visual_rot,
                                    scale,
                                    self.link_pose[child_link.name][0],
                                    self.link_pose[child_link.name][1],
                                )
                                objects.append(object)

                            for object in objects:
                                object.select_set(True)

                            if len(bpy.context.selected_objects) > 1:
                                bpy.context.view_layer.objects.active = (
                                    bpy.context.selected_objects[0]
                                )
                                bpy.ops.object.join()

                            bone_name = (
                                child_joint.name
                                + "."
                                + str(child_joint.type)
                                + self.bone_tail
                            )
                            self.add_bone(
                                child_link, child_joint, joint_pos, joint_rot, bone_name
                            )

                            objects[0].name = child_link.name
                            self.bind_mesh_to_bone(child_link.name, bone_name)
                        else:
                            bone_name = (
                                child_joint.name
                                + "."
                                + str(child_joint.type)
                                + self.bone_tail
                            )
                            self.add_bone(
                                child_link, child_joint, child_pos, child_rot, bone_name
                            )

                        self.parent_links.append(child_link)

                    del self.robot.child_map[link.name]

                # Remove finish link from parent links
                self.parent_links.remove(link)
        return None


def export_fbx(filepath):
    bpy.ops.export_scene.fbx(
        filepath=filepath,
        object_types={"ARMATURE", "MESH"},
        mesh_smooth_type="FACE",
        add_leaf_bones=False,
    )
    if os.path.exists(TMP_TEXTURE_PATH):
        copytree(
            TMP_TEXTURE_PATH,
            os.path.dirname(filepath) + "/" + TMP_TEXTURE_PATH,
            dirs_exist_ok=True,
        )


def urdf_to_fbx(
    in_urdf,
    out_fbx,
    merge_materials_with_name,
    rename_materials,
    apply_weld,
    unique_name,
    scale_unit,
):
    RobotBuilder(
        in_urdf,
        merge_materials_with_name,
        rename_materials,
        apply_weld,
        unique_name,
        scale_unit,
    )
    export_fbx(out_fbx)
    bpy.ops.wm.save_as_mainfile(filepath=out_fbx.replace('.fbx', '.blend'))
