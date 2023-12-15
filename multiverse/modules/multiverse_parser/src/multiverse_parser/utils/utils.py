#!/usr/bin/env python3

import numpy
from scipy.spatial.transform import Rotation

from pxr import UsdGeom

xform_cache = UsdGeom.XformCache()


def diagonalize_inertia(inertia_tensor) -> (numpy.ndarray, numpy.ndarray):
    diagonal_inertia, eigenvectors = numpy.linalg.eigh(inertia_tensor)

    if numpy.linalg.det(eigenvectors) < 0:
        eigenvectors[:, 0] = -eigenvectors[:, 0]

    rotation_quat = Rotation.from_matrix(eigenvectors).as_quat()

    return diagonal_inertia, rotation_quat


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
