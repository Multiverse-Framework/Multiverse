#!/usr/bin/env python3.10

import os
import bpy


def clean_up_meshes(obj_name: str) -> None:
    bpy.context.view_layer.objects.active = bpy.context.selected_objects[0]
    if len(bpy.context.selected_objects) > 1:
        bpy.ops.object.join()

    selected_object = bpy.context.object
    selected_object.name = obj_name

    if selected_object.scale[0] * selected_object.scale[1] * selected_object.scale[2] < 0:
        bpy.ops.object.mode_set(mode="EDIT")
        bpy.ops.mesh.select_all(action="SELECT")
        bpy.ops.mesh.flip_normals()
        bpy.ops.object.mode_set(mode="OBJECT")

    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)


def export_usd(out_usd: str) -> None:
    clean_up_meshes(os.path.splitext(os.path.basename(out_usd))[0])
    bpy.ops.wm.usd_export(filepath=out_usd, selected_objects_only=True, overwrite_textures=True)
