import unittest

import math
import os
import tracemalloc
import random
from typing import List

from plotly import graph_objects
import numpy
from scipy.spatial.transform import Rotation

from multiverse_parser import (WorldBuilder,
                               JointBuilder, JointType, JointProperty,
                               GeomType, GeomProperty,
                               MeshBuilder, MeshProperty, MaterialProperty)
from multiverse_parser.utils import calculate_mesh_inertial, shift_inertia_tensor, diagonalize_inertia

from pxr import Usd, UsdGeom, Sdf


class Shape:
    _vertices: numpy.ndarray
    _faces: numpy.ndarray
    _pos: numpy.ndarray
    _quat: numpy.ndarray
    _density: float
    _mass: float
    _inertia_tensor: numpy.ndarray
    _center_of_mass: numpy.ndarray
    _mass_analytical: float = 0.0
    _center_of_mass_analytical = numpy.zeros((1, 3))
    _inertia_tensor_analytical_center = numpy.zeros((3, 3))

    def __init__(self,
                 density: float,
                 pos: numpy.ndarray = numpy.zeros((1, 3)),
                 quat: numpy.ndarray = numpy.array([1.0, 0.0, 0.0, 0.0])):
        self._density = density
        self._pos = pos
        self._quat = quat
        self._mass = 0
        self._inertia_tensor = numpy.zeros((3, 3))
        self._center_of_mass = numpy.zeros((1, 3))
        self._create_vertices_and_faces()
        self.apply_transform(self._pos, self._quat)

    def build(self):
        self.calculate_inertial()
        self.calculate_inertial_analytical()

    def _create_vertices_and_faces(self):
        raise NotImplementedError

    def calculate_inertial(self):
        self._mass, self._inertia_tensor, self._center_of_mass = calculate_mesh_inertial(vertices=self._vertices,
                                                                                         faces=self._faces,
                                                                                         density=self._density)

    def calculate_inertial_analytical(self):
        raise NotImplementedError

    def apply_transform(self,
                        pos: numpy.ndarray = numpy.zeros((1, 3)),
                        quat: numpy.ndarray = numpy.array([0.0, 0.0, 0.0, 1.0])):
        rotation = Rotation.from_quat(quat)
        self._vertices = pos + rotation.apply(self._vertices)

    def plot(self, display_wireframe: bool = True):
        fig = graph_objects.Figure()

        self.add_plot_content(fig=fig, display_wireframe=display_wireframe)

        limit = self.limit
        fig.update_layout(scene=dict(xaxis=dict(range=[-limit, limit]),
                                     yaxis=dict(range=[-limit, limit]),
                                     zaxis=dict(range=[-limit, limit])))
        fig.update_layout(scene_aspectmode='cube')
        fig.show()

    def add_plot_content(self, fig: graph_objects.Figure, display_wireframe: bool = True):
        fig.add_trace(graph_objects.Mesh3d(x=self._vertices[:, 0],
                                           y=self._vertices[:, 1],
                                           z=self._vertices[:, 2],
                                           i=self._faces[:, 0],
                                           j=self._faces[:, 1],
                                           k=self._faces[:, 2],
                                           color='lightpink',
                                           opacity=1.0))
        if display_wireframe:
            lines = []
            for face in self._faces:
                lines.append([self._vertices[face[0]], self._vertices[face[1]]])
                lines.append([self._vertices[face[1]], self._vertices[face[2]]])
                lines.append([self._vertices[face[2]], self._vertices[face[0]]])
            lines = numpy.array(lines)
            fig.add_trace(graph_objects.Scatter3d(x=lines[:, :, 0].flatten(),
                                                  y=lines[:, :, 1].flatten(),
                                                  z=lines[:, :, 2].flatten(),
                                                  mode='lines',
                                                  line=dict(color='black', width=1)))

    @property
    def vertices(self):
        return self._vertices

    @property
    def faces(self):
        return self._faces

    @property
    def limit(self):
        return max(max(self._vertices[:, 0]) - min(self._vertices[:, 0]),
                   max(self._vertices[:, 1]) - min(self._vertices[:, 1]),
                   max(self._vertices[:, 2]) - min(self._vertices[:, 2]))

    @property
    def mass(self):
        return self._mass

    @property
    def inertia_tensor(self):
        return self._inertia_tensor

    @property
    def center_of_mass(self):
        return self._center_of_mass

    @property
    def mass_analytical(self):
        return self._mass_analytical

    @property
    def inertia_tensor_analytical(self):
        return shift_inertia_tensor(mass=self._mass_analytical,
                                    inertia_tensor=self._inertia_tensor_analytical_center,
                                    pos=self._center_of_mass_analytical,
                                    quat=self._quat)

    @property
    def center_of_mass_analytical(self):
        return self._center_of_mass_analytical


class MultiShape:
    _shapes: List[Shape]
    _pos: numpy.ndarray
    _quat: numpy.ndarray
    _mass: float
    _inertia_tensor: numpy.ndarray
    _center_of_mass: numpy.ndarray

    def __init__(self,
                 pos: numpy.ndarray = numpy.zeros((1, 3)),
                 quat: numpy.ndarray = numpy.array([0.0, 0.0, 0.0, 1.0])):
        self._shapes = []
        self._pos = pos
        self._quat = quat
        self._mass = 0
        self._inertia_tensor = numpy.zeros((3, 3))
        self._center_of_mass = numpy.zeros((1, 3))

    def add_shape(self, shape):
        self._shapes.append(shape)

    def build(self):
        for shape in self._shapes[:]:
            shape.apply_transform(self._pos, self._quat)
            shape.build()
            self._mass += shape.mass
            self._center_of_mass += shape.mass * shape.center_of_mass
            self._inertia_tensor += shape.inertia_tensor
        if self._mass > 0.0:
            self._center_of_mass /= self._mass

    def plot(self, display_wireframe: bool = True):
        fig = graph_objects.Figure()

        limit = self.add_plot_content(fig=fig, display_wireframe=display_wireframe)

        fig.update_layout(scene=dict(xaxis=dict(range=[-limit, limit]),
                                     yaxis=dict(range=[-limit, limit]),
                                     zaxis=dict(range=[-limit, limit])))
        fig.update_layout(scene_aspectmode='cube')
        fig.show()

    def add_plot_content(self, fig: graph_objects.Figure, display_wireframe: bool = True):
        self.limit = 0
        for shape in self._shapes:
            shape.add_plot_content(fig=fig, display_wireframe=display_wireframe)
            self.limit = max(self.limit, shape.limit)
        return self.limit

    @property
    def mass(self):
        return self._mass

    @property
    def inertia_tensor(self):
        return self._inertia_tensor

    @property
    def center_of_mass(self):
        return self._center_of_mass


class Box(Shape):
    def __init__(self, a: float, b: float, c: float,
                 density: float,
                 pos: numpy.ndarray = numpy.zeros((1, 3)),
                 quat: numpy.ndarray = numpy.array([1.0, 0.0, 0.0, 0.0])):
        self._a_half = a / 2
        self._b_half = b / 2
        self._c_half = c / 2
        super().__init__(density=density, pos=pos, quat=quat)

    def _create_vertices_and_faces(self):
        self._vertices = numpy.array([
            [-self._a_half, -self._b_half, -self._c_half],
            [-self._a_half, self._b_half, -self._c_half],
            [self._a_half, self._b_half, -self._c_half],
            [self._a_half, -self._b_half, -self._c_half],
            [-self._a_half, -self._b_half, self._c_half],
            [-self._a_half, self._b_half, self._c_half],
            [self._a_half, self._b_half, self._c_half],
            [self._a_half, -self._b_half, self._c_half]
        ])

        self._faces = numpy.array([
            [0, 1, 2],
            [0, 2, 3],
            [1, 5, 6],
            [1, 6, 2],
            [5, 4, 7],
            [5, 7, 6],
            [4, 0, 3],
            [4, 3, 7],
            [3, 2, 6],
            [3, 6, 7],
            [4, 5, 1],
            [4, 1, 0]
        ])

    def calculate_inertial_analytical(self):
        self._mass_analytical = self._density * (2 * self._a_half) * (2 * self._b_half) * (2 * self._c_half)
        self._center_of_mass_analytical = self._pos
        self._inertia_tensor_analytical_center = self._mass_analytical * numpy.array(
            [[1.0 / 3 * (self._b_half ** 2 + self._c_half ** 2), 0.0, 0.0],
             [0.0, 1.0 / 3 * (self._a_half ** 2 + self._c_half ** 2), 0.0],
             [0.0, 0.0, 1.0 / 3 * (self._a_half ** 2 + self._b_half ** 2)]])


class Sphere(Shape):
    def __init__(self, radius: float,
                 density: float,
                 pos: numpy.ndarray = numpy.zeros((1, 3)),
                 quat: numpy.ndarray = numpy.array([1.0, 0.0, 0.0, 0.0]),
                 num_segments: int = 32, num_slices: int = 128):
        self._radius = radius
        self._num_segments = num_segments
        self._num_slices = num_slices
        super().__init__(density=density, pos=pos, quat=quat)

    def _create_vertices_and_faces(self):
        segment_angle = math.pi / self._num_segments
        slice_angle = 2 * math.pi / self._num_slices

        # Define the vertices of a sphere
        vertices = []
        for i in range(self._num_segments + 1):
            if i == 0:
                vertices.append([0.0, 0.0, self._radius])
                continue
            elif i == self._num_segments:
                vertices.append([0.0, 0.0, -self._radius])
                continue
            for j in reversed(range(self._num_slices)):
                x = self._radius * math.sin(segment_angle * i) * math.cos(slice_angle * j)
                y = self._radius * math.sin(segment_angle * i) * math.sin(slice_angle * j)
                z = self._radius * math.cos(segment_angle * i)
                vertices.append([x, y, z])

        # Define the faces of the sphere (vertices' indices for each face)
        faces = []
        for i in range(self._num_segments):
            if i == 0:
                for j in range(self._num_slices):
                    faces.append([0,
                                  (j + 1) % self._num_slices + 1,
                                  j + 1])
            elif i == self._num_segments - 1:
                for j in range(self._num_slices):
                    faces.append([(self._num_segments - 1) * self._num_slices + 1,
                                  (i - 1) * self._num_slices + j + 1,
                                  (i - 1) * self._num_slices + (j + 1) % self._num_slices + 1])
            else:
                for j in range(self._num_slices):
                    faces.append([(i - 1) * self._num_slices + j + 1,
                                  (i - 1) * self._num_slices + (j + 1) % self._num_slices + 1,
                                  i * self._num_slices + (j + 1) % self._num_slices + 1])
                    faces.append([(i - 1) * self._num_slices + j + 1,
                                  i * self._num_slices + (j + 1) % self._num_slices + 1,
                                  i * self._num_slices + j + 1])

        self._vertices = numpy.array(vertices)
        self._faces = numpy.array(faces)

    def calculate_inertial_analytical(self):
        self._mass_analytical = self._density * 4 / 3 * math.pi * self._radius ** 3
        self._center_of_mass_analytical = self._pos
        self._inertia_tensor_analytical_center = (
                numpy.array([[2.0 / 5, 0.0, 0.0],
                             [0.0, 2.0 / 5, 0.0],
                             [0.0, 0.0, 2.0 / 5]]) * self._mass_analytical * self._radius ** 2)


class Cylinder(Shape):
    def __init__(self, radius: float, height: float,
                 density: float,
                 pos: numpy.ndarray = numpy.zeros((1, 3)),
                 quat: numpy.ndarray = numpy.array([1.0, 0.0, 0.0, 0.0]),
                 num_segments: int = 32, num_slices: int = 128):
        self._radius = radius
        self._height = height
        self._num_segments = num_segments
        self._num_slices = num_slices
        super().__init__(density=density, pos=pos, quat=quat)

    def _create_vertices_and_faces(self):
        segment_height = self._height / self._num_segments
        slice_angle = 2 * math.pi / self._num_slices

        # Define the vertices of a cylinder
        vertices = []
        for i in range(self._num_segments + 2):
            if i == 0:
                vertices.append([0.0, 0.0, -self._height / 2])
            elif i == self._num_segments + 1:
                vertices.append([0.0, 0.0, self._height / 2])
                break
            for j in range(self._num_slices):
                x = self._radius * math.cos(slice_angle * j)
                y = self._radius * math.sin(slice_angle * j)
                z = -self._height / 2 + segment_height * i
                vertices.append([x, y, z])

        # Define the faces of the sphere (vertices' indices for each face)
        faces = []
        for i in range(self._num_segments + 2):
            if i == 0:
                for j in range(self._num_slices):
                    faces.append([0,
                                  (j + 1) % self._num_slices + 1,
                                  j + 1])
            elif i == self._num_segments + 1:
                for j in range(self._num_slices):
                    faces.append([(self._num_segments + 1) * self._num_slices + 1,
                                  (i - 1) * self._num_slices + j + 1,
                                  (i - 1) * self._num_slices + (j + 1) % self._num_slices + 1])
            else:
                for j in range(self._num_slices):
                    faces.append([(i - 1) * self._num_slices + j + 1,
                                  (i - 1) * self._num_slices + (j + 1) % self._num_slices + 1,
                                  i * self._num_slices + (j + 1) % self._num_slices + 1])
                    faces.append([(i - 1) * self._num_slices + j + 1,
                                  i * self._num_slices + (j + 1) % self._num_slices + 1,
                                  i * self._num_slices + j + 1])

        self._vertices = numpy.array(vertices)
        self._faces = numpy.array(faces)

    def calculate_inertial_analytical(self):
        self._mass_analytical = self._density * math.pi * self._radius ** 2 * self._height
        self._center_of_mass_analytical = self._pos
        self._inertia_tensor_analytical_center = numpy.array(
            [[1.0 / 12 * self._mass_analytical * (3 * self._radius ** 2 + self._height ** 2), 0.0, 0.0],
             [0.0, 1.0 / 12 * self._mass_analytical * (3 * self._radius ** 2 + self._height ** 2), 0.0],
             [0.0, 0.0, 1.0 / 2 * self._mass_analytical * self._radius ** 2]])


class Mesh(Shape):
    def __init__(self, vertices: numpy.ndarray, faces: numpy.ndarray,
                 density: float,
                 pos: numpy.ndarray = numpy.zeros((1, 3)),
                 quat: numpy.ndarray = numpy.array([1.0, 0.0, 0.0, 0.0])):
        self._vertices = vertices
        self._faces = faces
        super().__init__(density=density, pos=pos, quat=quat)

    def _create_vertices_and_faces(self):
        pass

    def calculate_inertial_analytical(self):
        pass


class UsdMesh(MultiShape):
    mesh_property: MeshProperty

    def __init__(self,
                 mesh_file_path: str,
                 density: float,
                 pos: numpy.ndarray = numpy.zeros((1, 3)), quat: numpy.ndarray = numpy.array([0.0, 0.0, 0.0, 1.0])):
        stage = Usd.Stage.Open(mesh_file_path)
        mesh_property = MeshProperty.from_prim(mesh_prim=stage.GetDefaultPrim())
        mesh = MeshBuilder(stage=stage, mesh_property=mesh_property).build()

        xform_pos = mesh.GetLocalTransformation().ExtractTranslation()
        xform_quat = mesh.GetLocalTransformation().ExtractRotationQuat()
        xform_quat = list(xform_quat.GetImaginary()) + [xform_quat.GetReal()]

        xform_pos = numpy.array(xform_pos)
        xform_quat = numpy.array(xform_quat)

        pos = Rotation.from_quat(quat).apply(xform_pos) + pos
        quat = Rotation.from_quat(xform_quat) * Rotation.from_quat(quat)

        self._density = density
        super().__init__(pos=pos, quat=quat.as_quat())
        self.mesh_property = MeshProperty.from_prim(mesh_prim=mesh.GetPrim())

    def apply_transform(self,
                        pos: numpy.ndarray = numpy.zeros((1, 3)),
                        quat: numpy.ndarray = numpy.array([0.0, 0.0, 0.0, 1.0])):
        pass

    def build(self):
        mesh_property = self.mesh_property
        vertices = mesh_property.points
        face_vertex_counts = mesh_property.face_vertex_counts
        face_vertex_indices = mesh_property.face_vertex_indices
        normals = mesh_property.normals

        faces = []
        face_idx = 0
        for face_vertex_count in face_vertex_counts:
            face = face_vertex_indices[face_idx:(face_idx + face_vertex_count)]
            face_normals = normals[face_idx:(face_idx + face_vertex_count)]
            if not numpy.isclose(numpy.linalg.norm(face_normals), 0.0):
                edge_1 = vertices[face[1]] - vertices[face[0]]
                edge_2 = vertices[face[2]] - vertices[face[1]]
                edge_3 = vertices[face[0]] - vertices[face[2]]
                normal_1 = numpy.cross(edge_1, edge_2)
                normal_1 /= numpy.linalg.norm(normal_1)
                normal_2 = numpy.cross(edge_2, edge_3)
                normal_2 /= numpy.linalg.norm(normal_2)
                normal_3 = numpy.cross(edge_3, edge_1)
                normal_3 /= numpy.linalg.norm(normal_3)

                if (not numpy.allclose(face_normals, normal_1)
                        or not numpy.allclose(face_normals, normal_2)
                        or not numpy.allclose(face_normals, normal_3)):
                    print(f"Flip face {face}.")
                    face = face[::-1]

            faces.append(face)
            face_idx += face_vertex_count
        faces = numpy.array(faces)

        self.add_shape(shape=Mesh(vertices=vertices, faces=faces, density=self._density))
        super().build()


class FactoryTestCase(unittest.TestCase):
    plot: bool = False
    resource_path: str

    @classmethod
    def setUpClass(cls):
        tracemalloc.start()
        cls.resource_path = os.path.join(os.path.dirname(__file__), "..", "resources")

    def test_diagonal_inertia(self):
        inertia_tensor = numpy.diag([random.uniform(0.1, 1.0) for _ in range(3)])
        pos = numpy.array([[random.uniform(-1.0, 1.0) for _ in range(3)]])
        rotation = Rotation.from_euler('xyz', [random.uniform(0.0, 360.0) for _ in range(3)], degrees=True)
        quat = rotation.as_quat()
        inertia_tensor_rotate = shift_inertia_tensor(mass=1.0,
                                                     inertia_tensor=inertia_tensor,
                                                     pos=pos,
                                                     quat=quat)
        diag_inertia, rotation_quat = diagonalize_inertia(inertia_tensor_rotate)
        inertia_tensor_recover = shift_inertia_tensor(mass=1.0,
                                                      inertia_tensor=inertia_tensor_rotate,
                                                      quat=Rotation.from_quat(rotation_quat).inv().as_quat(
                                                      ))
        inertia_tensor_rotate_recover = shift_inertia_tensor(mass=1.0,
                                                             inertia_tensor=inertia_tensor_recover,
                                                             quat=rotation_quat)
        numpy.testing.assert_array_almost_equal(inertia_tensor_rotate, inertia_tensor_rotate_recover)

    def test_world_builder(self):
        file_path = os.path.join(self.resource_path, "output", "test_world_builder", "test.usda")
        world_builder = WorldBuilder(usd_file_path=file_path)
        self.assertIsInstance(world_builder._stage, Usd.Stage)

        body_builder_0 = world_builder.add_body(body_name="body_0")
        self.assertEqual(body_builder_0.xform.GetPath(), "/body_0")

        body_builder_1 = world_builder.add_body(body_name="body_1", parent_body_name="body_0")
        self.assertEqual(body_builder_1.xform.GetPath(), "/body_0/body_1")

        body_builder_2 = world_builder.add_body(body_name="body_2", parent_body_name="body_0")
        self.assertEqual(body_builder_2.xform.GetPath(), "/body_0/body_2")

        body_builder_3 = world_builder.add_body(body_name="body_3", parent_body_name="body_2")
        self.assertEqual(body_builder_3.xform.GetPath(), "/body_0/body_2/body_3")

        body_builder_4 = world_builder.add_body(body_name="body_1", parent_body_name="body_2")
        self.assertEqual(body_builder_4, body_builder_1)

        with self.assertRaises(ValueError):
            world_builder.add_body(body_name="body_4", parent_body_name="body_abcxyz")

        body_builder_1.set_transform(pos=numpy.array([1.0, 0.0, 0.0]),
                                     quat=numpy.array([math.sqrt(2) / 2, 0.0, 0.0, math.sqrt(2) / 2]),
                                     scale=numpy.array([1.0, 1.0, 1.0]))
        body_1_xform = body_builder_1.xform
        body_1_local_transform = body_1_xform.GetLocalTransformation()
        body_1_local_pos = body_1_local_transform.ExtractTranslation()
        self.assertEqual(body_1_local_pos, (1.0, 0.0, 0.0))

        body_builder_3.set_transform(pos=numpy.array([1.0, 0.0, 0.0]),
                                     quat=numpy.array([math.sqrt(2) / 2, 0.0, 0.0, math.sqrt(2) / 2]),
                                     scale=numpy.array([1.0, 1.0, 1.0]),
                                     relative_to_xform=body_1_xform)

        physics_mass_api_1 = body_builder_3.set_inertial(mass=1.0,
                                                         center_of_mass=numpy.array([0.0, 0.0, 0.0]),
                                                         diagonal_inertia=numpy.array([1.0, 1.0, 1.0]))
        self.assertEqual(physics_mass_api_1.GetMassAttr().Get(), 1.0)
        self.assertEqual(physics_mass_api_1.GetCenterOfMassAttr().Get(), (0.0, 0.0, 0.0))
        self.assertEqual(physics_mass_api_1.GetDiagonalInertiaAttr().Get(), (1.0, 1.0, 1.0))
        self.assertEqual(physics_mass_api_1.GetPrincipalAxesAttr().Get().GetReal(), 1.0)
        self.assertEqual(physics_mass_api_1.GetPrincipalAxesAttr().Get().GetImaginary(), (0.0, 0.0, 0.0))

        geom_property_1 = GeomProperty(geom_type=GeomType.CUBE)
        geom_builder_1 = body_builder_1.add_geom(geom_name="geom_1", geom_property=geom_property_1)
        geom_1 = geom_builder_1.gprim
        self.assertEqual(geom_1.GetPath(), "/body_0/body_1/geom_1")

        geom_builder_1.set_transform(pos=numpy.array([0.1, 0.2, 3.0]),
                                     quat=numpy.array([math.sqrt(2) / 2, 0.0, 0.0, math.sqrt(2) / 2]),
                                     scale=numpy.array([1.0, 1.0, 1.0]))
        geom_1_local_transform = geom_1.GetLocalTransformation()
        geom_1_local_pos = geom_1_local_transform.ExtractTranslation()
        self.assertEqual(geom_1_local_pos, (0.1, 0.2, 3.0))

        geom_builder_1.set_attribute(prefix="primvars", displayColor=[(0, 0, 0)])
        geom_builder_1.set_attribute(prefix="primvars", displayOpacity=[1])

        joint_property = JointProperty(joint_parent_prim=body_builder_1.xform.GetPrim(),
                                       joint_child_prim=body_builder_3.xform.GetPrim(),
                                       joint_type=JointType.FIXED)
        joint_builder = body_builder_1.add_joint(joint_name="joint_0", joint_property=joint_property)
        self.assertEqual(joint_builder.path, "/body_0/body_1/joint_0")
        self.assertEqual(joint_builder.type, JointType.FIXED)

        geom_property_2 = GeomProperty(geom_type=GeomType.MESH)
        geom_builder_2 = body_builder_1.add_geom(geom_name="geom_2", geom_property=geom_property_2)
        usd_mesh_file_path = os.path.join(self.resource_path,
                                          "input",
                                          "milk_box",
                                          "meshes",
                                          "usd",
                                          "milk_box.usda")
        mesh_property = MeshProperty.from_mesh_file_path(mesh_file_path=usd_mesh_file_path,
                                                         mesh_path=Sdf.Path("/SM_MilkBox"))
        mesh_builder_2 = geom_builder_2.add_mesh(mesh_name="milk_box",
                                                 mesh_property=mesh_property)
        self.assertEqual(mesh_builder_2.mesh.GetPrim().GetPath(), "/milk_box")
        geom_2 = geom_builder_1.stage.GetPrimAtPath("/body_0/body_1/geom_2")
        self.assertTrue(geom_2.IsValid())
        self.assertTrue(geom_2.IsA(UsdGeom.Mesh))

        geom_inertial = geom_builder_2.calculate_inertial()
        self.assertEqual(geom_inertial.mass, 1.1399999493733048)

        usd_material_file_path = usd_mesh_file_path
        material_property = MaterialProperty.from_material_file_path(material_file_path=usd_material_file_path,
                                                                     material_path="/_materials/M_MilkBox")
        geom_builder_2.add_material(material_name="milk_box_mat",
                                    material_property=material_property)

    def test_inertia_of_box(self):
        a = random.uniform(0.1, 0.3)
        b = random.uniform(0.1, 0.4)
        c = random.uniform(0.1, 0.5)
        density = 1.0

        box = Box(a=a, b=b, c=c, density=density)
        box.build()
        if self.plot:
            box.plot()

        self.assertAlmostEqual(box.mass, box.mass_analytical)
        numpy.testing.assert_array_almost_equal(box.center_of_mass, box.center_of_mass_analytical)
        numpy.testing.assert_array_almost_equal(box.inertia_tensor, box.inertia_tensor_analytical)

    def test_inertia_of_box_with_fixed_pos(self):
        a = 1.0
        b = 1.0
        c = 1.0
        pos = numpy.array([[0.5, 0.5, 0.5]])
        density = 1.0

        box = Box(a=a, b=b, c=c, density=density, pos=pos)
        box.build()
        if self.plot:
            box.plot()

        self.assertAlmostEqual(box.mass, box.mass_analytical)
        numpy.testing.assert_array_almost_equal(box.center_of_mass, box.center_of_mass_analytical)
        numpy.testing.assert_array_almost_equal(box.inertia_tensor, box.inertia_tensor_analytical)

    def test_inertia_of_box_with_fixed_pos_and_quat(self):
        a = 1.0
        b = 2.0
        c = 3.0
        pos = numpy.array([[3.0, 1.0, 2.0]])
        quat = Rotation.from_euler('xyz', [30, 45, 60], degrees=True).as_quat()
        density = 1.0

        box = Box(a=a, b=b, c=c, density=density, pos=pos, quat=quat)
        box.build()
        if self.plot:
            box.plot()

        self.assertAlmostEqual(box.mass, box.mass_analytical)
        numpy.testing.assert_array_almost_equal(box.center_of_mass, box.center_of_mass_analytical)
        numpy.testing.assert_array_almost_equal(box.inertia_tensor, box.inertia_tensor_analytical)

    def test_inertia_of_box_with_random_pos(self):
        a = random.uniform(0.1, 0.3)
        b = random.uniform(0.1, 0.4)
        c = random.uniform(0.1, 0.5)
        pos = numpy.array([[random.uniform(-0.5, 0.5), random.uniform(-0.3, 0.3), random.uniform(-0.2, 0.2)]])
        random_angles = [random.uniform(0, 360), random.uniform(0, 360), random.uniform(0, 360)]
        quat = Rotation.from_euler('xyz', random_angles, degrees=True).as_quat()
        density = 2.0

        box = Box(a=a, b=b, c=c, density=density, pos=pos, quat=quat)
        box.build()
        if self.plot:
            box.plot()

        self.assertAlmostEqual(box.mass, box.mass_analytical)
        numpy.testing.assert_array_almost_equal(box.center_of_mass, box.center_of_mass_analytical)
        numpy.testing.assert_array_almost_equal(box.inertia_tensor, box.inertia_tensor_analytical)

    def test_inertia_of_sphere(self):
        radius = random.uniform(0.5, 1.0)
        density = 1.0

        sphere = Sphere(radius=radius,
                        density=density,
                        num_segments=random.randint(6, 32),
                        num_slices=16)
        sphere.build()
        if self.plot:
            sphere.plot()

        self.assertAlmostEqual(sphere.mass, sphere.mass_analytical, places=0)
        numpy.testing.assert_array_almost_equal(sphere.center_of_mass, sphere.center_of_mass_analytical)
        numpy.testing.assert_array_almost_equal(sphere.inertia_tensor, sphere.inertia_tensor_analytical, decimal=1)

    def test_inertia_of_sphere_with_random_pos_and_quat(self):
        radius = random.uniform(0.5, 1.0)
        density = 1.0
        pos = numpy.array([[random.uniform(-0.5, 0.5), random.uniform(-0.3, 0.3), random.uniform(-0.2, 0.2)]])
        random_angles = [random.uniform(0, 360), random.uniform(0, 360), random.uniform(0, 360)]
        quat = Rotation.from_euler('xyz', random_angles, degrees=True).as_quat()

        sphere = Sphere(radius=radius,
                        density=density,
                        pos=pos,
                        quat=quat,
                        num_segments=random.randint(6, 32),
                        num_slices=random.randint(16, 32))
        sphere.build()
        if self.plot:
            sphere.plot()

        self.assertAlmostEqual(sphere.mass, sphere.mass_analytical, places=0)
        numpy.testing.assert_array_almost_equal(sphere.center_of_mass, sphere.center_of_mass_analytical)
        numpy.testing.assert_array_almost_equal(sphere.inertia_tensor, sphere.inertia_tensor_analytical, decimal=1)

    def test_inertia_of_cylinder(self):
        radius = random.uniform(0.5, 1.0)
        height = random.uniform(1.0, 2.0)
        density = 1.0

        cylinder = Cylinder(radius=radius,
                            height=height,
                            density=density,
                            num_segments=random.randint(6, 32),
                            num_slices=random.randint(16, 32))
        cylinder.build()
        if self.plot:
            cylinder.plot()

        self.assertAlmostEqual(cylinder.mass, cylinder.mass_analytical, places=0)
        numpy.testing.assert_array_almost_equal(cylinder.center_of_mass, cylinder.center_of_mass_analytical)
        numpy.testing.assert_array_almost_equal(cylinder.inertia_tensor, cylinder.inertia_tensor_analytical, decimal=1)

    def test_inertia_of_cylinder_with_random_pos(self):
        radius = random.uniform(0.5, 1.0)
        height = random.uniform(1.0, 2.0)
        density = 1.0
        pos = numpy.array([[random.uniform(-0.5, 0.5), random.uniform(-0.3, 0.3), random.uniform(-0.2, 0.2)]])
        random_angles = [random.uniform(0, 360), random.uniform(0, 360), random.uniform(0, 360)]
        quat = Rotation.from_euler('xyz', random_angles, degrees=True).as_quat()

        cylinder = Cylinder(radius=radius,
                            height=height,
                            density=density,
                            pos=pos,
                            quat=quat,
                            num_segments=random.randint(6, 32),
                            num_slices=random.randint(16, 32))
        cylinder.build()
        if self.plot:
            cylinder.plot()

        self.assertAlmostEqual(cylinder.mass, cylinder.mass_analytical, places=0)
        numpy.testing.assert_array_almost_equal(cylinder.center_of_mass, cylinder.center_of_mass_analytical)
        numpy.testing.assert_array_almost_equal(cylinder.inertia_tensor, cylinder.inertia_tensor_analytical, decimal=1)

    def test_multiple_boxes_with_random_pos_and_quat(self):
        pos = numpy.array([[random.uniform(-0.5, 0.5), random.uniform(-0.3, 0.3), random.uniform(-0.2, 0.2)]])
        random_angles = [random.uniform(0, 360), random.uniform(0, 360), random.uniform(0, 360)]
        quat = Rotation.from_euler('xyz', random_angles, degrees=True).as_quat()
        offset = Rotation.from_quat(quat).apply(numpy.array([[0.0, 0.125, 0.0]]))

        pos_1 = pos + offset
        pos_2 = pos - offset

        density = 1.0
        box_1 = Box(a=0.5, b=0.25, c=0.5,
                    density=density,
                    pos=pos_1,
                    quat=quat)
        box_2 = Box(a=0.5, b=0.25, c=0.5,
                    density=density,
                    pos=pos_2,
                    quat=quat)

        multi_shape = MultiShape()
        multi_shape.add_shape(box_1)
        multi_shape.add_shape(box_2)
        multi_shape.build()
        if self.plot:
            multi_shape.plot()

        box_combined = Box(a=0.5, b=0.5, c=0.5,
                           density=density,
                           pos=pos,
                           quat=quat)
        box_combined.build()
        if self.plot:
            box_combined.plot()

        self.assertAlmostEqual(multi_shape.mass, box_combined.mass)
        numpy.testing.assert_array_almost_equal(multi_shape.center_of_mass, box_combined.center_of_mass)
        numpy.testing.assert_array_almost_equal(multi_shape.inertia_tensor, box_combined.inertia_tensor)

    def test_multiple_shapes(self):
        density = 1.0
        box_1 = Box(a=0.5, b=0.1, c=0.5,
                    density=density,
                    pos=numpy.array([[0.0, -0.5, 0.0]]))
        box_2 = Box(a=0.5, b=0.1, c=0.5,
                    density=density,
                    pos=numpy.array([[0.0, 0.5, 0.0]]))
        cylinder = Cylinder(radius=0.05, height=1.0,
                            density=density,
                            quat=Rotation.from_euler(seq='x', angles=90, degrees=True).as_quat(),
                            num_segments=6,
                            num_slices=16)

        vertices = numpy.concatenate((box_1.vertices, box_2.vertices, cylinder.vertices), axis=0)
        faces = numpy.concatenate((box_1.faces,
                                   box_2.faces + len(box_1.vertices),
                                   cylinder.faces + len(box_1.vertices) + len(box_2.vertices)), axis=0)

        multi_shape = MultiShape()
        multi_shape.add_shape(box_1)
        multi_shape.add_shape(box_2)
        multi_shape.add_shape(cylinder)
        multi_shape.build()
        if self.plot:
            multi_shape.plot()

        shape_combined = Mesh(vertices=vertices, faces=faces,
                              density=density)
        shape_combined.build()
        if self.plot:
            shape_combined.plot()

        self.assertAlmostEqual(multi_shape.mass, shape_combined.mass)
        numpy.testing.assert_array_almost_equal(multi_shape.center_of_mass, shape_combined.center_of_mass)
        numpy.testing.assert_array_almost_equal(multi_shape.inertia_tensor, shape_combined.inertia_tensor)

    def test_multiple_shapes_with_random_pos_and_quat(self):
        pos = numpy.array([[random.uniform(-0.5, 0.5), random.uniform(-0.3, 0.3), random.uniform(-0.2, 0.2)]])
        random_angles = [random.uniform(0, 360), random.uniform(0, 360), random.uniform(0, 360)]
        quat = Rotation.from_euler('xyz', random_angles, degrees=True).as_quat()

        density = 1.0
        box_1 = Box(a=0.5, b=0.1, c=0.5,
                    density=density,
                    pos=numpy.array([[0.0, -0.5, 0.0]]))
        box_2 = Box(a=0.5, b=0.1, c=0.5,
                    density=density,
                    pos=numpy.array([[0.0, 0.5, 0.0]]))
        cylinder = Cylinder(radius=0.05, height=1.0,
                            density=density,
                            quat=Rotation.from_euler(seq='x', angles=90, degrees=True).as_quat(),
                            num_segments=6,
                            num_slices=16)

        vertices = numpy.concatenate((box_1.vertices, box_2.vertices, cylinder.vertices), axis=0)
        faces = numpy.concatenate((box_1.faces,
                                   box_2.faces + len(box_1.vertices),
                                   cylinder.faces + len(box_1.vertices) + len(box_2.vertices)), axis=0)

        multi_shape = MultiShape(pos=pos, quat=quat)
        multi_shape.add_shape(box_1)
        multi_shape.add_shape(box_2)
        multi_shape.add_shape(cylinder)
        multi_shape.build()
        if self.plot:
            multi_shape.plot()

        shape_combined = Mesh(density=density,
                              vertices=vertices,
                              faces=faces,
                              pos=pos,
                              quat=quat)
        shape_combined.build()
        if self.plot:
            shape_combined.plot()

        self.assertAlmostEqual(multi_shape.mass, shape_combined.mass)
        numpy.testing.assert_array_almost_equal(multi_shape.center_of_mass, shape_combined.center_of_mass)
        numpy.testing.assert_array_almost_equal(multi_shape.inertia_tensor, shape_combined.inertia_tensor)

    def test_inertia_of_mesh_1(self, pos=numpy.array([[0.0, 0.0, 0.0]]), quat=numpy.array([0.0, 0.0, 0.0, 1.0])):
        mesh_file_path = os.path.join(self.resource_path, "input", "milk_box", "meshes", "usd", "milk_box.usda")
        density = 1.0

        usd_mesh = UsdMesh(mesh_file_path=mesh_file_path,
                           density=density,
                           pos=pos, quat=quat)
        usd_mesh.build()
        if self.plot:
            usd_mesh.plot()

        a = 0.06
        b = 0.095
        c = 0.2
        box = Box(a=a, b=b, c=c,
                  density=density,
                  pos=pos, quat=quat)
        box.build()
        if self.plot:
            box.plot()

        self.assertAlmostEqual(usd_mesh.mass, box.mass)
        numpy.testing.assert_array_almost_equal(usd_mesh.center_of_mass, box.center_of_mass)
        numpy.testing.assert_array_almost_equal(usd_mesh.inertia_tensor, box.inertia_tensor)

    def test_inertia_of_mesh_2(self):
        pos = numpy.array([[random.uniform(-0.05, 0.05),
                            random.uniform(-0.03, 0.03),
                            random.uniform(-0.02, 0.02)]])
        random_angles = [random.uniform(0, 360), random.uniform(0, 360), random.uniform(0, 360)]
        quat = Rotation.from_euler('xyz', random_angles, degrees=True).as_quat()

        self.test_inertia_of_mesh_1(pos=pos, quat=quat)

    def test_inertia_of_mesh_3(self, pos=numpy.array([[0.0, 0.0, 0.0]]), quat=numpy.array([0.0, 0.0, 0.0, 1.0])):
        mesh_file_path = os.path.join(self.resource_path,
                                      "input",
                                      "furnitures",
                                      "Assets",
                                      "Game",
                                      "Meshes",
                                      "MeshesLiving",
                                      "SM_Esstisch_payload.usda")
        density = 7800.0

        usd_mesh = UsdMesh(mesh_file_path=mesh_file_path,
                           density=density,
                           pos=pos, quat=quat)
        usd_mesh.build()
        if self.plot:
            usd_mesh.plot(display_wireframe=False)

    def test_inertia_of_mesh_4(self):
        mesh_file_path_0 = os.path.join(self.resource_path,
                                      "input",
                                      "furnitures",
                                      "Assets",
                                      "Game",
                                      "Meshes",
                                      "MeshesLiving",
                                      "SM_Schreibtisch_Oben_payload.usda")
        mesh_file_path_1 = os.path.join(self.resource_path,
                                      "input",
                                      "furnitures",
                                      "Assets",
                                      "Game",
                                      "Meshes",
                                      "MeshesLiving",
                                      "SM_Schreibtisch_Unten_payload.usda")
        density = 1000.0

        usd_mesh_0 = UsdMesh(mesh_file_path=mesh_file_path_0,
                             density=density,
                             pos=numpy.array([[0.0, 0.0, 0.4423281801131945]]),
                             quat=Rotation.from_euler(seq='x', angles=180, degrees=True).as_quat())
        usd_mesh_1 = UsdMesh(mesh_file_path=mesh_file_path_1,
                             density=density,
                             pos=numpy.array([[0.0, 0.0, 0.03113409208382618]]),
                             quat=Rotation.from_euler(seq='x', angles=180, degrees=True).as_quat())

        usd_mesh = MultiShape()
        usd_mesh.add_shape(usd_mesh_0)
        usd_mesh.add_shape(usd_mesh_1)
        usd_mesh.build()

        if self.plot:
            usd_mesh.plot(display_wireframe=False)

    @classmethod
    def tearDownClass(cls):
        tracemalloc.stop()


if __name__ == '__main__':
    unittest.main()
