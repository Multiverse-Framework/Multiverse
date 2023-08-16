#!/usr/bin/env python3.10

import bpy

def export_usd(out_usd: str) -> None:
    bpy.ops.wm.usd_export(filepath=out_usd, selected_objects_only=False, overwrite_textures=True)