#!/usr/bin/env python3.10

import os
import bpy


def clean_up_meshes(file_path: str) -> None:
    bpy.context.view_layer.objects.active = bpy.context.selected_objects[0]
    if len(bpy.context.selected_objects) > 1:
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


def export_obj(out_obj: str) -> None:
    clean_up_meshes(out_obj)
    os.makedirs(name=os.path.dirname(out_obj), exist_ok=True)
    bpy.ops.export_scene.obj(filepath=out_obj, use_selection=True, axis_forward="Y", axis_up="Z", path_mode="RELATIVE")


def export_stl(out_stl: str) -> None:
    clean_up_meshes(out_stl)
    os.makedirs(name=os.path.dirname(out_stl), exist_ok=True)
    selected_object = bpy.context.object
    selected_object.modifiers.new("Weld", "WELD")
    bpy.ops.object.modifier_apply(modifier="Weld")
    bpy.ops.export_mesh.stl(filepath=out_stl, use_selection=True, axis_forward="Y", axis_up="Z")
