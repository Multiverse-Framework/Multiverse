#!/usr/bin/env python3.10

import bpy


def clear_meshes() -> None:
    for armature in bpy.data.armatures:
        bpy.data.armatures.remove(armature)
    for mesh in bpy.data.meshes:
        bpy.data.meshes.remove(mesh)
    for obj in bpy.data.objects:
        bpy.data.objects.remove(obj)
    for material in bpy.data.materials:
        bpy.data.materials.remove(material)
    for camera in bpy.data.cameras:
        bpy.data.cameras.remove(camera)
    for light in bpy.data.lights:
        bpy.data.lights.remove(light)
    for image in bpy.data.images:
        bpy.data.images.remove(image)

    return None


def import_usd(in_usd: str) -> None:
    clear_meshes()
    bpy.ops.wm.usd_import(filepath=in_usd, scale=1.0)


def import_dae(in_dae: str) -> None:
    clear_meshes()
    bpy.ops.wm.collada_import(filepath=in_dae)


def import_obj(in_obj: str) -> None:
    clear_meshes()
    bpy.ops.wm.obj_import(filepath=in_obj, forward_axis="Y", up_axis="Z")


def import_stl(in_stl: str) -> None:
    clear_meshes()
    bpy.ops.import_mesh.stl(filepath=in_stl)
