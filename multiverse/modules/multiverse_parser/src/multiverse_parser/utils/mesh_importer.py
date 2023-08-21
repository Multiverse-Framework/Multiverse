#!/usr/bin/env python3.10

import bpy


def import_usd(in_usd: str) -> None:
    bpy.ops.wm.usd_import(filepath=in_usd, scale=1.0)


def import_dae(in_dae: str) -> None:
    bpy.ops.wm.collada_import(filepath=in_dae)


def import_obj(in_obj: str) -> None:
    bpy.ops.import_scene.obj(filepath=in_obj, axis_forward="Y", axis_up="Z")


def import_stl(in_stl: str) -> None:
    bpy.ops.import_mesh.stl(filepath=in_stl)
