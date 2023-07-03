import numpy as np
from stl import mesh

# Define the vertices and faces of your geometry
vertices = np.array([
    [0.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [1.0, 1.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0],
    [1.0, 0.0, 1.0],
    [1.0, 1.0, 1.0],
    [0.0, 1.0, 1.0]
], dtype=np.float32)

faces = np.array([
    [0, 1, 2],
    [0, 2, 3],
    [0, 4, 5],
    [0, 5, 1],
    [1, 5, 6],
    [1, 6, 2],
    [2, 6, 7],
    [2, 7, 3],
    [3, 7, 4],
    [3, 4, 0],
    [4, 7, 6],
    [4, 6, 5]
], dtype=np.int32)

# Calculate the normal vectors for each face
normals = np.cross(
    vertices[faces[:, 1]] - vertices[faces[:, 0]],
    vertices[faces[:, 2]] - vertices[faces[:, 0]]
)
normals /= np.linalg.norm(normals, axis=1)[:, np.newaxis]

# Create the STL mesh object with normals
cube_mesh = mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))
cube_mesh.vectors = vertices[faces]
cube_mesh.normals = normals

# Save the mesh to an STL file
cube_mesh.save('cube.stl')
