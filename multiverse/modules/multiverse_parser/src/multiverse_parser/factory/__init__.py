import os
import importlib.util
import random, string

TMP = "tmp"
TMP_DIR = TMP + "/usd"
TMP_USD_FILE_DIR = os.path.join(
    os.path.dirname(importlib.util.find_spec("multiverse_parser").origin),
    ".cache",
    "".join(random.choices(string.ascii_letters + string.digits, k=10)),
)
TMP_USD_FILE_PATH = os.path.join(TMP_USD_FILE_DIR, TMP + ".usda")
TMP_USD_MESH_PATH = os.path.join(TMP_USD_FILE_DIR, TMP_DIR)

def clear_data() -> None:
    from bpy import data

    for armature in data.armatures:
        data.armatures.remove(armature)
    for mesh in data.meshes:
        data.meshes.remove(mesh)
    for object in data.objects:
        data.objects.remove(object)
    for material in data.materials:
        data.materials.remove(material)
    for camera in data.cameras:
        data.cameras.remove(camera)
    for light in data.lights:
        data.lights.remove(light)
    for image in data.images:
        data.images.remove(image)

    return None
