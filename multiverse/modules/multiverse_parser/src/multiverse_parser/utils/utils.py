#!/usr/bin/env python3.10

import numpy
from scipy.spatial.transform import Rotation
# import bpy
# import tf
# import os, shutil
from pxr import UsdGeom, Gf, UsdShade

xform_cache = UsdGeom.XformCache()


def diagonalize_inertia(inertia_tensor):
    # Perform Singular Value Decomposition
    U, S, Vh = numpy.linalg.svd(inertia_tensor)

    diagonal_inertia = numpy.diag(numpy.diag(S))

    R = numpy.dot(U, Vh)
    rotation_quat = Rotation.from_matrix(R).as_quat(canonical=False)

    return diagonal_inertia, rotation_quat

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


def calculate_tet3_inertia_moment(v1: numpy.ndarray, v2: numpy.ndarray, v3: numpy.ndarray, i: int) -> float:
    return v1[i] ** 2 + v2[i] ** 2 + v3[i] ** 2 + v1[i] * v2[i] + v2[i] * v3[i] + v3[i] * v1[i]


def calculate_tet3_inertia_product(v1: numpy.ndarray, v2: numpy.ndarray, v3: numpy.ndarray, i: int, j: int) -> float:
    return (2 * v1[i] * v1[j] + 2 * v2[i] * v2[j] + 2 * v3[i] * v3[j] +
            v1[i] * v2[j] + v2[i] * v1[j] + v1[i] * v3[j] + v3[i] * v1[j] + v2[i] * v3[j] + v3[i] * v2[j])


def calculate_mesh_inertial(vertices: numpy.ndarray, faces: numpy.ndarray, density: float) -> \
        (float, numpy.ndarray, numpy.ndarray):
    # Initialize the mass to zero
    mass = 0.0

    # Initialize the inertia tensor to zeros
    Ixx = Iyy = Izz = Ixy = Ixz = Iyz = 0.0

    # Initialize the center of mass to zeros
    center_of_mass = numpy.zeros((1, 3))

    for face in faces:
        # Extract the vertices of the triangle face
        v1, v2, v3 = vertices[face]

        det = numpy.dot(v1, numpy.cross(v2, v3))

        triangle_volume = det / 6.0
        triangle_mass = density * triangle_volume
        mass += triangle_mass

        triangle_center_of_mass = (v1 + v2 + v3) / 4.0
        center_of_mass += triangle_center_of_mass * triangle_mass

        v100 = calculate_tet3_inertia_moment(v1, v2, v3, 0)
        v010 = calculate_tet3_inertia_moment(v1, v2, v3, 1)
        v001 = calculate_tet3_inertia_moment(v1, v2, v3, 2)

        Ixx += det * (v010 + v001)
        Iyy += det * (v100 + v001)
        Izz += det * (v100 + v010)
        Ixy += det * calculate_tet3_inertia_product(v1, v2, v3, 0, 1)
        Ixz += det * calculate_tet3_inertia_product(v1, v2, v3, 0, 2)
        Iyz += det * calculate_tet3_inertia_product(v1, v2, v3, 1, 2)

    center_of_mass /= mass

    Ixx = density * Ixx / 60.0
    Iyy = density * Iyy / 60.0
    Izz = density * Izz / 60.0
    Ixy = density * Ixy / 120.0
    Ixz = density * Ixz / 120.0
    Iyz = density * Iyz / 120.0

    inertia_tensor = numpy.array([
        [Ixx, -Ixy, -Ixz],
        [-Ixy, Iyy, -Iyz],
        [-Ixz, -Iyz, Izz]
    ])

    return mass, inertia_tensor, center_of_mass


def shift_inertia_tensor(mass: float,
                         inertia_tensor: numpy.ndarray,
                         pos: numpy.ndarray = numpy.zeros((1, 3)),
                         quat: numpy.ndarray = numpy.array([0.0, 0.0, 0.0, 1.0])) -> numpy.ndarray:
    # https://phys.libretexts.org/Bookshelves/Classical_Mechanics/Variational_Principles_in_Classical_Mechanics_(Cline)/13%3A_Rigid-body_Rotation/13.08%3A_Parallel-Axis_Theorem
    inertia_tensor_parallel = mass * (
        numpy.array([[pos[0][1] ** 2 + pos[0][2] ** 2, - pos[0][0] * pos[0][1], - pos[0][0] * pos[0][2]],
                     [- pos[0][0] * pos[0][1], pos[0][2] ** 2 + pos[0][0] ** 2, - pos[0][1] * pos[0][2]],
                     [- pos[0][0] * pos[0][2], - pos[0][1] * pos[0][2], pos[0][0] ** 2 + pos[0][1] ** 2]]))

    rotation_matrix = Rotation.from_quat(quat).as_matrix()
    return rotation_matrix @ inertia_tensor @ rotation_matrix.T + inertia_tensor_parallel

def shift_center_of_mass(center_of_mass: numpy.ndarray,
                         pos: numpy.ndarray = numpy.zeros((1, 3)),
                         quat: numpy.ndarray = numpy.array([0.0, 0.0, 0.0, 1.0])) -> numpy.ndarray:
    rotation = Rotation.from_quat(quat)
    return rotation.apply(center_of_mass) + pos

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
