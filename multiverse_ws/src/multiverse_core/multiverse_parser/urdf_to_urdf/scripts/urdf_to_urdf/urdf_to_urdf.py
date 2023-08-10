#!/usr/bin/env python3.10

import bpy
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

def urdf_to_urdf(in_urdf: str, out_urdf: str):
    clear_data(bpy.data, 0.01)
    bpy.ops.wm.collada_import(filepath='/media/giangnguyen/Storage/Multiverse/multiverse_ws/src/multiverse_robots/universal_robots/ur_description/meshes/ur5e/visual/base.dae')
    
    bpy.ops.export_scene.obj(filepath='/media/giangnguyen/Storage/Multiverse/base.obj', use_selection=True)