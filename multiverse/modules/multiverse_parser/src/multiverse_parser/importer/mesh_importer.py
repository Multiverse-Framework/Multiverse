#!/usr/bin/env python3.10

import bpy
from bpy.types import (
    Armature,
    BlendData,
    Camera,
    Image,
    Light,
    Material,
    Mesh,
    Object,
)


def clear_data(data: BlendData) -> None:
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

    return None


def import_dae(in_dae: str) -> None:
    clear_data(bpy.data)
    bpy.ops.wm.collada_import(filepath=in_dae)


def import_obj(in_obj: str) -> None:
    clear_data(bpy.data)
    bpy.ops.import_scene.obj(filepath=in_obj, axis_forward="Y", axis_up="Z")


def import_stl(in_stl: str) -> None:
    clear_data(bpy.data)
    bpy.ops.import_mesh.stl(filepath=in_stl)
