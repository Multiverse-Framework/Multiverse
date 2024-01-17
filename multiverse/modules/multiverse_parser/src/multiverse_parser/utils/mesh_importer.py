#!/usr/bin/env python3

from typing import List

clean_up_meshes_script = """
for armature in bpy.data.armatures:
    bpy.data.armatures.remove(armature)
for mesh in bpy.data.meshes:
    bpy.data.meshes.remove(mesh)
for from_obj in bpy.data.objects:
    bpy.data.objects.remove(from_obj)
for material in bpy.data.materials:
    bpy.data.materials.remove(material)
for camera in bpy.data.cameras:
    bpy.data.cameras.remove(camera)
for light in bpy.data.lights:
    bpy.data.lights.remove(light)
for image in bpy.data.images:
    bpy.data.images.remove(image)
"""


def import_usd(in_usds: List[str]) -> str:
    return f"{clean_up_meshes_script}" \
           f"for in_usd in {in_usds}:\n" \
           f"   bpy.ops.wm.usd_import(filepath=in_usd, scale=1.0)"


def import_dae(in_dae: str) -> str:
    return f"{clean_up_meshes_script}" \
           f"bpy.ops.wm.collada_import(filepath='{in_dae}')"


def import_obj(in_obj: str) -> str:
    return f"{clean_up_meshes_script}" \
           f"bpy.ops.wm.obj_import(filepath='{in_obj}', up_axis='Z', forward_axis='Y')"


def import_stl(in_stl: str) -> str:
    return f"{clean_up_meshes_script}" \
           f"bpy.ops.wm.stl_import(filepath='{in_stl}', up_axis='Z', forward_axis='Y')"
