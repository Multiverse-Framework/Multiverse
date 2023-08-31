#!/usr/bin/env python3.10

import os
import bpy
import shutil
from PIL import Image


def clean_up_meshes(file_path: str) -> None:
    bpy.context.view_layer.objects.active = bpy.context.selected_objects[0]
    if len([selected_object for selected_object in bpy.context.selected_objects if selected_object.type == "MESH"]) > 1:
        bpy.ops.object.join()

    selected_object = bpy.context.object
    selected_object.name = os.path.splitext(os.path.basename(file_path))[0]

    if selected_object.scale[0] * selected_object.scale[1] * selected_object.scale[2] < 0:
        bpy.ops.object.mode_set(mode="EDIT")
        bpy.ops.mesh.select_all(action="SELECT")
        bpy.ops.mesh.flip_normals()
        bpy.ops.object.mode_set(mode="OBJECT")

    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)


def export_usd(out_usd: str) -> None:
    clean_up_meshes(out_usd)
    bpy.ops.wm.usd_export(filepath=out_usd, selected_objects_only=True, overwrite_textures=True)


def export_obj(out_obj: str) -> []:
    clean_up_meshes(out_obj)
    out_obj_dir = os.path.dirname(out_obj)
    os.makedirs(name=os.path.join(out_obj_dir, "textures"), exist_ok=True)
    bpy.ops.export_scene.obj(filepath=out_obj, use_selection=True, axis_forward="Y", axis_up="Z", path_mode="RELATIVE")
    out_mtl = out_obj.replace(".obj", ".mtl")
    with open(out_mtl, "r") as file:
        lines = file.readlines()

    png_file_names = []
    for i, line in enumerate(lines):
        if line.startswith("map_Kd"):
            texture_path = os.path.join(out_obj_dir, line.split("map_Kd")[1].strip())
            texture_file_name = os.path.basename(texture_path)
            new_texture_path = os.path.join("textures", texture_file_name)
            shutil.copy2(texture_path, os.path.join(out_obj_dir, new_texture_path))
            lines[i] = f"map_Kd {new_texture_path}\n"
            img = Image.open(texture_path)
            png_file_name = texture_file_name.replace(".jpg", ".png")
            img.save(os.path.join(out_obj_dir, new_texture_path).replace(".jpg", ".png"), "PNG")
            png_file_names.append(png_file_name)

    with open(out_mtl, "w") as file:
        file.writelines(lines)

    return png_file_names


def export_stl(out_stl: str) -> None:
    clean_up_meshes(out_stl)
    os.makedirs(name=os.path.dirname(out_stl), exist_ok=True)
    selected_object = bpy.context.object
    selected_object.modifiers.new("Weld", "WELD")
    bpy.ops.object.modifier_apply(modifier="Weld")
    bpy.ops.export_mesh.stl(filepath=out_stl, use_selection=True, axis_forward="Y", axis_up="Z")
