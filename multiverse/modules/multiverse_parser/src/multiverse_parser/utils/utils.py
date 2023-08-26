import numpy
from scipy.spatial.transform import Rotation
import bpy
import tf
from pxr import UsdGeom, Gf

xform_cache = UsdGeom.XformCache()


def diagonalize_inertia(inertia_tensor):
    eigenvalues, eigenvectors = numpy.linalg.eigh(inertia_tensor)
    eigenvectors = eigenvectors / numpy.linalg.norm(eigenvectors, axis=0)
    rotation_quat = Rotation.from_matrix(eigenvectors).as_quat()
    rotation_quat = (
        rotation_quat[3],
        rotation_quat[0],
        rotation_quat[1],
        rotation_quat[2],
    )
    diagonal_inertia = numpy.diag(eigenvalues)
    diagonal_inertia = (
        diagonal_inertia[0][0],
        diagonal_inertia[1][1],
        diagonal_inertia[2][2],
    )

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


def convert_quat(quat) -> tuple:
    if isinstance(quat, Gf.Quatf) or isinstance(quat, Gf.Quatd):
        real = quat.GetReal()
        imaginary = quat.GetImaginary()
        quat = (imaginary[0], imaginary[1], imaginary[2], real)
    return quat


def quat_to_rpy(quat) -> tuple:
    quat = convert_quat(quat)
    return tf.transformations.euler_from_quaternion(quat)


def rotate_vector_by_quat(vector, quat) -> tuple:
    quat = convert_quat(quat)
    point = numpy.array(list(vector) + [1.0])
    matrix = tf.transformations.quaternion_matrix(quat)
    rotated_point = numpy.dot(matrix, point)
    rotated_vector = rotated_point[:3] / rotated_point[3]

    return tuple(rotated_vector)


def transform(xyz: tuple = (0.0, 0.0, 0.0), rpy: tuple = (0.0, 0.0, 0.0), scale: tuple = (1.0, 1.0, 1.0)) -> None:
    bpy.context.view_layer.objects.active = bpy.context.selected_objects[0]
    selected_object = bpy.context.object
    selected_object.location = xyz
    selected_object.rotation_euler = rpy
    selected_object.scale = scale
