import numpy
from scipy.spatial.transform import Rotation


def diagonalize_inertia(inertia_tensor):
    eigenvalues, eigenvectors = numpy.linalg.eigh(inertia_tensor)
    eigenvectors = eigenvectors / numpy.linalg.norm(eigenvectors, axis=0)
    rotation_quat = Rotation.from_matrix(eigenvectors).as_quat()
    rotation_quat = (rotation_quat[3], rotation_quat[0], rotation_quat[1], rotation_quat[2])
    diagonal_inertia = numpy.diag(eigenvalues)
    diagonal_inertia = (diagonal_inertia[0][0], diagonal_inertia[1][1], diagonal_inertia[2][2])

    return diagonal_inertia, rotation_quat
