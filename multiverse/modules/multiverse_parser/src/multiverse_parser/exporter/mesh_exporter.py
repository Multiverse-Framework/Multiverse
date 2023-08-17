#!/usr/bin/env python3.10

import os
import bpy


def join_meshes() -> None:
    bpy.context.view_layer.objects.active = bpy.context.selected_objects[0]
    if len(bpy.context.selected_objects) > 1:
        bpy.ops.object.join()


def export_usd(out_usd: str) -> None:
    join_meshes()
    selected_object = bpy.context.object
    selected_object.name = os.path.splitext(os.path.basename(out_usd))[0]
    bpy.ops.wm.usd_export(
        filepath=out_usd, selected_objects_only=True, overwrite_textures=True
    )
