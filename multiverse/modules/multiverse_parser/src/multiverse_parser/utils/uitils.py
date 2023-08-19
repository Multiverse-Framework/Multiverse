import numpy
from scipy.spatial.transform import Rotation
import bpy


def diagonalize_inertia(inertia_tensor):
    eigenvalues, eigenvectors = numpy.linalg.eigh(inertia_tensor)
    eigenvectors = eigenvectors / numpy.linalg.norm(eigenvectors, axis=0)
    rotation_quat = Rotation.from_matrix(eigenvectors).as_quat()
    rotation_quat = (rotation_quat[3], rotation_quat[0], rotation_quat[1], rotation_quat[2])
    diagonal_inertia = numpy.diag(eigenvalues)
    diagonal_inertia = (diagonal_inertia[0][0], diagonal_inertia[1][1], diagonal_inertia[2][2])

    return diagonal_inertia, rotation_quat


def clear_meshes() -> None:
    for armature in bpy.data.armatures:
        bpy.data.armatures.remove(armature)
    for mesh in bpy.data.meshes:
        bpy.data.meshes.remove(mesh)
    for object in bpy.data.objects:
        bpy.data.objects.remove(object)
    for material in bpy.data.materials:
        bpy.data.materials.remove(material)
    for camera in bpy.data.cameras:
        bpy.data.cameras.remove(camera)
    for light in bpy.data.lights:
        bpy.data.lights.remove(light)
    for image in bpy.data.images:
        bpy.data.images.remove(image)

    return None


def modify_name(in_name: str, replacement: str = None) -> str:
    out_name = in_name.replace(" ", "").replace("-", "_")
    if out_name == "" and replacement is not None:
        out_name = replacement
    return out_name
