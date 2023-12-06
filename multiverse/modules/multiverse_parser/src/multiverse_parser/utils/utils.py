#!/usr/bin/env python3.10

import numpy
from scipy.spatial.transform import Rotation
# import bpy
# import tf
# import os, shutil
from pxr import UsdGeom, Gf, UsdShade

xform_cache = UsdGeom.XformCache()


#

# def diagonalize_inertia(inertia_tensor):
#     eigenvalues, eigenvectors = numpy.linalg.eigh(inertia_tensor)
#     eigenvectors = eigenvectors / numpy.linalg.norm(eigenvectors, axis=0)
#     rotation_quat = Rotation.from_matrix(eigenvectors).as_quat()
#     rotation_quat = (
#         rotation_quat[3],
#         rotation_quat[0],
#         rotation_quat[1],
#         rotation_quat[2],
#     )
#     diagonal_inertia = numpy.diag(eigenvalues)
#     diagonal_inertia = (
#         diagonal_inertia[0][0],
#         diagonal_inertia[1][1],
#         diagonal_inertia[2][2],
#     )
#
#     return diagonal_inertia, rotation_quat
#
#
# def clear_meshes() -> None:
#     for armature in bpy.data.armatures:
#         bpy.data.armatures.remove(armature)
#     for mesh in bpy.data.meshes:
#         bpy.data.meshes.remove(mesh)
#     for object in bpy.data.objects:
#         bpy.data.objects.remove(object)
#     for material in bpy.data.materials:
#         bpy.data.materials.remove(material)
#     for camera in bpy.data.cameras:
#         bpy.data.cameras.remove(camera)
#     for light in bpy.data.lights:
#         bpy.data.lights.remove(light)
#     for image in bpy.data.images:
#         bpy.data.images.remove(image)
#
#     return None
#
#
def modify_name(in_name: str, replacement: str = None) -> str:
    out_name = in_name
    for special_char in [" ", "-", "~", ".", "/"]:
        if special_char in in_name:
            print(f"Name {in_name} contains {special_char}, replacing with _")
            out_name = out_name.replace(special_char, "_")
    if out_name == "":
        if replacement is None:
            raise ValueError(f"Name {in_name} is empty and replacement is None.")
        out_name = replacement
    return out_name


def rpy_to_quat(rpy_angles: numpy.ndarray) -> numpy.ndarray:
    """
    Convert RPY Euler angles to a quaternion
    :param rpy_angles: Tuple of (roll, pitch, yaw) angles in radians
    :return: Tuple of (w, x, y, z) quaternion
    """
    # Create a Rotation object from the RPY Euler angles
    rotation = Rotation.from_euler('xyz', rpy_angles)  # 'xyz' order means RPY

    # Get the quaternion representation from the Rotation object
    quaternion = rotation.as_quat(canonical=False)

    return numpy.array([quaternion[3], quaternion[0], quaternion[1], quaternion[2]])


def calculate_triangle_inertia(v1: numpy.ndarray, v2: numpy.ndarray, v3: numpy.ndarray,
                               density: float) -> numpy.ndarray:
    # Ensure the input arrays have the same length
    if len(v1) != 3 or len(v2) != 3 or len(v3) != 3:
        raise ValueError("Input arrays must have 3 elements each.")

    # Create NumPy arrays for the coordinates
    v1 = numpy.array(v1)
    v2 = numpy.array(v2)
    v3 = numpy.array(v3)

    # Calculate the centroid (center of mass)
    centroid = numpy.array([numpy.mean(v1), numpy.mean(v2), numpy.mean(v3)])

    # Calculate the coordinates relative to the centroid
    x_rel = v1 - centroid[0]
    y_rel = v2 - centroid[1]
    z_rel = v3 - centroid[2]

    # Calculate the inertia tensor
    I_xx = density * numpy.sum((y_rel ** 2 + z_rel ** 2))
    I_yy = density * numpy.sum((x_rel ** 2 + z_rel ** 2))
    I_zz = density * numpy.sum((x_rel ** 2 + y_rel ** 2))
    I_xy = - density * numpy.sum(x_rel * y_rel)
    I_xz = - density * numpy.sum(x_rel * z_rel)
    I_yz = - density * numpy.sum(y_rel * z_rel)

    # Create the inertia tensor matrix
    return numpy.array([[I_xx, I_xy, I_xz],
                        [I_xy, I_yy, I_yz],
                        [I_xz, I_yz, I_zz]])


def calculate_mesh_inertia(vertices: numpy.ndarray, faces: numpy.ndarray, density: float):
    # Initialize the inertia tensor to zeros
    inertia_tensor = numpy.zeros((3, 3))

    for face in faces:
        # Extract the vertices of the triangle face
        v1, v2, v3 = vertices[face]

        # Calculate the inertia tensor for the triangle face
        triangle_inertia = calculate_triangle_inertia(v1, v2, v3, density)

        # Add the contribution of the triangle to the overall inertia tensor
        inertia_tensor += triangle_inertia

    return inertia_tensor

# def convert_quat(quat) -> tuple:
#     if isinstance(quat, Gf.Quatf) or isinstance(quat, Gf.Quatd):
#         real = quat.GetReal()
#         imaginary = quat.GetImaginary()
#         quat = (imaginary[0], imaginary[1], imaginary[2], real)
#     return quat
#
#
# def quat_to_rpy(quat) -> tuple:
#     quat = convert_quat(quat)
#     return tf.transformations.euler_from_quaternion(quat)
#
#
# def rotate_vector_by_quat(vector, quat) -> tuple:
#     quat = convert_quat(quat)
#     point = numpy.array(list(vector) + [1.0])
#     matrix = tf.transformations.quaternion_matrix(quat)
#     rotated_point = numpy.dot(matrix, point)
#     rotated_vector = rotated_point[:3] / rotated_point[3]
#
#     return tuple(rotated_vector)
#
#
# def transform(xyz: tuple = (0.0, 0.0, 0.0), rpy: tuple = (0.0, 0.0, 0.0), scale: tuple = (1.0, 1.0, 1.0)) -> None:
#     bpy.context.view_layer.objects.active = bpy.context.selected_objects[0]
#     selected_object = bpy.context.object
#     selected_object.location = xyz
#     selected_object.rotation_euler = rpy
#     selected_object.scale = scale
#
#
# def copy_prim(src_prim, dest_prim, prefix=None) -> None:
#     if type(src_prim) != type(dest_prim):
#         print(f"Types of {src_prim} and {dest_prim} are different")
#         return
#
#     src_stage = src_prim.GetStage()
#     dest_stage = dest_prim.GetStage()
#
#     src_file_path = src_stage.GetRootLayer().realPath
#     dest_file_path = dest_stage.GetRootLayer().realPath
#
#     for schema_api in src_prim.GetAppliedSchemas():
#         dest_prim.ApplyAPI(schema_api)
#
#     for src_rel in src_prim.GetRelationships():
#         src_rel_name = src_rel.GetName()
#         dest_rel = dest_prim.CreateRelationship(name=src_rel_name, custom=False)
#         targets = src_rel.GetTargets()
#         if prefix is not None:
#             new_targets = []
#             for target in targets:
#                 new_targets.append(target.ReplacePrefix(src_stage.GetDefaultPrim().GetPath(), prefix))
#             dest_rel.SetTargets(new_targets)
#
#         if src_stage == dest_stage:
#             continue
#
#         for target in targets:
#             src_rel_prim = src_stage.GetPrimAtPath(target)
#
#             if UsdShade.Material(src_rel_prim):
#                 src_material = UsdShade.Material(src_rel_prim)
#                 dest_material = UsdShade.Material.Define(dest_stage, target)
#
#                 if src_material.GetSurfaceAttr().Get() is not None:
#                     dest_surface_attr = dest_material.CreateSurfaceAttr()
#                     dest_surface_attr.Set(src_material.GetSurfaceAttr().Get())
#
#                 if src_material.GetSurfaceOutput() is not None:
#                     mesh_surface_output = dest_material.CreateSurfaceOutput()
#                     for connected_sources in src_material.GetSurfaceOutput().GetConnectedSources():
#                         for connected_source in connected_sources:
#                             mesh_surface_output.ConnectToSource(connected_source)
#
#                 for src_rel_child_prim in src_rel_prim.GetChildren():
#                     if UsdShade.Shader(src_rel_child_prim):
#                         src_shader = UsdShade.Shader(src_rel_child_prim)
#                         dest_shader = UsdShade.Shader.Define(dest_stage, src_rel_child_prim.GetPath())
#
#                         for src_shader_inumpyut in src_shader.GetInumpyuts():
#                             dest_shader_inumpyut = dest_shader.CreateInumpyut(src_shader_inumpyut.GetBaseName(), src_shader_inumpyut.GetTypeName())
#                             for connected_sources in src_shader_inumpyut.GetConnectedSources():
#                                 for connected_source in connected_sources:
#                                     dest_shader_inumpyut.ConnectToSource(connected_source)
#
#                         for src_shader_output in src_shader.GetOutputs():
#                             dest_shader_output = dest_shader.CreateOutput(src_shader_output.GetBaseName(), src_shader_output.GetTypeName())
#                             for connected_sources in src_shader_output.GetConnectedSources():
#                                 for connected_source in connected_sources:
#                                     dest_shader_output.ConnectToSource(connected_source)
#
#                         for src_shader_attr in src_shader.GetPrim().GetAttributes():
#                             src_shader_attr_value = src_shader_attr.Get()
#                             if src_shader_attr.GetName() == "inumpyuts:file":
#                                 src_shader_file_path = src_shader_attr_value.resolvedPath
#                                 dest_shader_file_path = os.path.join(os.path.dirname(dest_file_path), src_shader_attr_value.path)
#
#                                 if not os.path.exists(os.path.dirname(dest_shader_file_path)):
#                                     os.makedirs(os.path.dirname(dest_shader_file_path))
#
#                                 if not os.path.exists(dest_shader_file_path):
#                                     shutil.copy(src_shader_file_path, dest_shader_file_path)
#
#                         copy_prim(src_shader.GetPrim(), dest_shader.GetPrim())
#
#             else:
#                 if prefix is not None:
#                     target = prefix.AppendPath(target)
#                 dest_rel_prim = dest_stage.DefinePrim(target, src_rel_prim.GetTypeName())
#                 copy_prim(src_rel_prim, dest_rel_prim)
#
#     for src_attr in src_prim.GetAttributes():
#         value = src_attr.Get()
#         if value is not None:
#             dest_attr = dest_prim.GetPrim().CreateAttribute(
#                 name=src_attr.GetName(), typeName=src_attr.GetTypeName(), custom=src_attr.IsCustom(), variability=src_attr.GetVariability()
#             )
#             dest_attr.Set(value)
#             prim_var = UsdGeom.Primvar(src_attr)
#             if prim_var.HasValue():
#                 mesh_prim_var = UsdGeom.Primvar(dest_attr)
#                 mesh_prim_var.SetInterpolation(prim_var.GetInterpolation())
#
#     for src_child_prim in src_prim.GetChildren():
#         dest_child_prim = dest_prim.GetStage().DefinePrim(dest_prim.GetPath().AppendPath(src_child_prim.GetName()), src_child_prim.GetTypeName())
#         copy_prim(src_child_prim, dest_child_prim, prefix)
