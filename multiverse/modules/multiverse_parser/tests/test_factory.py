import unittest

import math
import os
import tracemalloc
import random

import matplotlib.pyplot as plt
import numpy
from pxr import Usd
from multiverse_parser import (WorldBuilder,
                               JointBuilder, JointType, JointProperty,
                               GeomType, GeomProperty,
                               MeshBuilder)
from multiverse_parser.utils import calculate_mesh_inertial


class Shape:
    _vertices: numpy.ndarray
    _faces: numpy.ndarray
    _pos: numpy.ndarray = numpy.zeros((1, 3))
    _quat: numpy.ndarray = numpy.array([1.0, 0.0, 0.0, 0.0])
    _density: float
    _mass: float = 0.0
    _inertia_tensor = numpy.zeros((3, 3))
    _center_of_mass = numpy.zeros((1, 3))
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

    def build(self):
        raise NotImplementedError

    def plot(self):
        raise NotImplementedError

    def _plot(self, xlim: float = 1.0, ylim: float = 1.0, zlim: float = 1.0):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.set_aspect("equal")
        ax.set_xlim(-xlim, xlim)
        ax.set_ylim(-ylim, ylim)
        ax.set_zlim(-zlim, zlim)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        # ax.plot(vertices[:, 0], vertices[:, 1], vertices[:, 2], "o")
        for face in self._faces:
            ax.plot(self._vertices[face, 0], self._vertices[face, 1], self._vertices[face, 2], "-")
        plt.show()

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
        # https://phys.libretexts.org/Bookshelves/Classical_Mechanics/Variational_Principles_in_Classical_Mechanics_(Cline)/13%3A_Rigid-body_Rotation/13.08%3A_Parallel-Axis_Theorem
        inertia_tensor_analytical_parallel = self._mass_analytical * (
            numpy.array(
                [[self._center_of_mass_analytical[0][1] ** 2 + self._center_of_mass_analytical[0][2] ** 2,
                  - self._center_of_mass_analytical[0][0] * self._center_of_mass_analytical[0][1],
                  - self._center_of_mass_analytical[0][0] * self._center_of_mass_analytical[0][2]],
                 [- self._center_of_mass_analytical[0][0] * self._center_of_mass_analytical[0][1],
                  self._center_of_mass_analytical[0][2] ** 2 + self._center_of_mass_analytical[0][0] ** 2,
                  - self._center_of_mass_analytical[0][1] * self._center_of_mass_analytical[0][2]],
                 [- self._center_of_mass_analytical[0][0] * self._center_of_mass_analytical[0][2],
                  - self._center_of_mass_analytical[0][1] * self._center_of_mass_analytical[0][2],
                  self._center_of_mass_analytical[0][0] ** 2 + self._center_of_mass_analytical[0][1] ** 2]]
            ))
        return self._inertia_tensor_analytical_center + inertia_tensor_analytical_parallel

    @property
    def center_of_mass_analytical(self):
        return self._center_of_mass_analytical


class Box(Shape):
    def __init__(self, a: float, b: float, c: float,
                 density: float,
                 pos: numpy.ndarray = numpy.zeros((1, 3)),
                 quat: numpy.ndarray = numpy.array([1.0, 0.0, 0.0, 0.0])):
        super().__init__(density=density, pos=pos, quat=quat)
        self._a_half = a / 2
        self._b_half = b / 2
        self._c_half = c / 2

    def build(self):
        self._vertices = numpy.array([
            [-self._a_half, -self._b_half, -self._c_half],
            [-self._a_half, self._b_half, -self._c_half],
            [self._a_half, self._b_half, -self._c_half],
            [self._a_half, -self._b_half, -self._c_half],
            [-self._a_half, -self._b_half, self._c_half],
            [-self._a_half, self._b_half, self._c_half],
            [self._a_half, self._b_half, self._c_half],
            [self._a_half, -self._b_half, self._c_half]
        ]) + self._pos

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
        self._mass, self._inertia_tensor, self._center_of_mass = calculate_mesh_inertial(vertices=self._vertices,
                                                                                         faces=self._faces,
                                                                                         density=self._density)
        self._mass_analytical = self._density * (2 * self._a_half) * (2 * self._b_half) * (2 * self._c_half)
        self._center_of_mass_analytical = self._pos
        self._inertia_tensor_analytical_center = self._mass_analytical * numpy.array(
            [[1.0 / 3 * (self._b_half ** 2 + self._c_half ** 2), 0.0, 0.0],
             [0.0, 1.0 / 3 * (self._a_half ** 2 + self._c_half ** 2), 0.0],
             [0.0, 0.0, 1.0 / 3 * (self._a_half ** 2 + self._b_half ** 2)]])

    def plot(self):
        self._plot(xlim=math.ceil(self._a_half + numpy.abs(self._pos[0][0])),
                   ylim=math.ceil(self._b_half + numpy.abs(self._pos[0][1])),
                   zlim=math.ceil(self._c_half + numpy.abs(self._pos[0][2])))


class Sphere(Shape):
    def __init__(self, radius: float,
                 density: float,
                 pos: numpy.ndarray = numpy.zeros((1, 3)),
                 quat: numpy.ndarray = numpy.array([1.0, 0.0, 0.0, 0.0]),
                 num_segments: int = 32, num_slices: int = 128):
        super().__init__(density=density, pos=pos, quat=quat)
        self._radius = radius
        self._num_segments = num_segments
        self._num_slices = num_slices

    def build(self):
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

        self._vertices = numpy.array(vertices) + self._pos
        self._faces = numpy.array(faces)

        self._mass, self._inertia_tensor, self._center_of_mass = calculate_mesh_inertial(vertices=self._vertices,
                                                                                         faces=self._faces,
                                                                                         density=self._density)

        self._mass_analytical = self._density * 4 / 3 * math.pi * self._radius ** 3
        self._center_of_mass_analytical = self._pos
        self._inertia_tensor_analytical_center = (
                numpy.array([[2.0 / 5, 0.0, 0.0],
                             [0.0, 2.0 / 5, 0.0],
                             [0.0, 0.0, 2.0 / 5]]) * self._mass_analytical * self._radius ** 2)

    def plot(self):
        self._plot(xlim=math.ceil(self._radius), ylim=math.ceil(self._radius), zlim=math.ceil(self._radius))


class Cylinder(Shape):
    def __init__(self, radius: float, height: float,
                 density: float,
                 pos: numpy.ndarray = numpy.zeros((1, 3)),
                 quat: numpy.ndarray = numpy.array([1.0, 0.0, 0.0, 0.0]),
                 num_segments: int = 32, num_slices: int = 128):
        super().__init__(density=density, pos=pos, quat=quat)
        self._radius = radius
        self._height = height
        self._num_segments = num_segments
        self._num_slices = num_slices

    def build(self):
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

        self._vertices = numpy.array(vertices) + self._pos
        self._faces = numpy.array(faces)

        self._mass, self._inertia_tensor, self._center_of_mass = calculate_mesh_inertial(vertices=self._vertices,
                                                                                         faces=self._faces,
                                                                                         density=self._density)

        self._mass_analytical = self._density * math.pi * self._radius ** 2 * self._height
        self._center_of_mass_analytical = self._pos
        self._inertia_tensor_analytical_center = numpy.array(
            [[1.0 / 12 * self._mass_analytical * (3 * self._radius ** 2 + self._height ** 2), 0.0, 0.0],
             [0.0, 1.0 / 12 * self._mass_analytical * (3 * self._radius ** 2 + self._height ** 2), 0.0],
             [0.0, 0.0, 1.0 / 2 * self._mass_analytical * self._radius ** 2]])

    def plot(self):
        self._plot(xlim=math.ceil(self._radius), ylim=math.ceil(self._radius), zlim=math.ceil(self._height / 2))


class Mesh(Shape):
    _mesh_builder: MeshBuilder

    def __init__(self, mesh_file_path: str, density: float):
        self._mesh_builder = MeshBuilder(mesh_file_path=mesh_file_path)
        pos = self._mesh_builder.xform.GetLocalTransformation().ExtractTranslation()
        quat = self._mesh_builder.xform.GetLocalTransformation().ExtractRotation().GetQuaternion()
        super().__init__(density=density, pos=numpy.array([pos]), quat=numpy.array([quat]))
        self._density = density
        self._extent = numpy.ones((1, 3))

    def build(self):

        for mesh in self._mesh_builder.meshes:
            mesh_name = mesh.GetPrim().GetName()
            vertices = self._mesh_builder.get_mesh_property(mesh_name=mesh_name).points
            self._extent = numpy.array([[max(vertices[:, 0]) - min(vertices[:, 0]),
                                         max(vertices[:, 1]) - min(vertices[:, 1]),
                                         max(vertices[:, 2]) - min(vertices[:, 2])]])

            face_vertex_counts = self._mesh_builder.get_mesh_property(mesh_name=mesh_name).face_vertex_counts
            face_vertex_indices = self._mesh_builder.get_mesh_property(mesh_name=mesh_name).face_vertex_indices
            faces = []
            face_idx = 0
            for face_vertex_count in face_vertex_counts:
                faces.append(face_vertex_indices[face_idx:(face_idx + face_vertex_count)])
                face_idx += face_vertex_count
            faces = numpy.array(faces)

            mass, inertia_tensor, center_of_mass = calculate_mesh_inertial(vertices=vertices,
                                                                           faces=faces,
                                                                           density=self._density)

            self._vertices = vertices
            self._faces = faces

            self._center_of_mass = (self._center_of_mass * self._mass + center_of_mass * mass) / (self._mass + mass)
            self._mass += mass
            self._inertia_tensor += inertia_tensor

    def plot(self):
        max_extent = max(self._extent[0, 0], self._extent[0, 1], self._extent[0, 2])
        self._plot(xlim=max_extent, ylim=max_extent, zlim=max_extent)


class FactoryTestCase(unittest.TestCase):
    resource_path: str

    @classmethod
    def setUpClass(cls):
        tracemalloc.start()
        cls.resource_path = os.path.join(os.path.dirname(__file__), "..", "resources")

    def test_world_builder(self):
        file_path = os.path.join(self.resource_path, "output", "test_world_builder", "test.usda")
        world_builder = WorldBuilder(file_path=file_path)
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

        geom_property = GeomProperty(geom_name="geom_1", geom_type=GeomType.CUBE)
        geom_builder_1 = body_builder_1.add_geom(geom_property=geom_property)
        geom_1_xform = geom_builder_1.xform
        self.assertEqual(geom_1_xform.GetPath(), "/body_0/body_1/geom_1")

        geom_builder_1.set_transform(pos=numpy.array([0.1, 0.2, 3.0]),
                                     quat=numpy.array([math.sqrt(2) / 2, 0.0, 0.0, math.sqrt(2) / 2]),
                                     scale=numpy.array([1.0, 1.0, 1.0]))
        geom_1_local_transform = geom_1_xform.GetLocalTransformation()
        geom_1_local_pos = geom_1_local_transform.ExtractTranslation()
        self.assertEqual(geom_1_local_pos, (0.1, 0.2, 3.0))

        geom_builder_1.set_attribute(prefix="primvars", displayColor=[(0, 0, 0)])
        geom_builder_1.set_attribute(prefix="primvars", displayOpacity=[1])

        joint_property = JointProperty(joint_name="joint_0",
                                       joint_parent_prim=body_builder_1.xform.GetPrim(),
                                       joint_child_prim=body_builder_3.xform.GetPrim(),
                                       joint_type=JointType.FIXED)
        joint_builder = JointBuilder(joint_property=joint_property)
        self.assertEqual(joint_builder.path, "/body_0/body_1/joint_0")
        self.assertEqual(joint_builder.type, JointType.FIXED)

        mesh_builder, _ = geom_builder_1.add_mesh(mesh_file_path=os.path.join(self.resource_path, "input", "milk_box",
                                                                              "meshes", "usd", "obj", "milk_box.usda"))
        self.assertEqual(mesh_builder.xform.GetPath(), "/milk_box")
        self.assertTrue(geom_builder_1.stage.GetPrimAtPath("/body_0/body_1/geom_1/SM_MilkBox").IsValid())

        world_builder.export()
        self.assertTrue(os.path.exists(file_path))

    def test_inertia_of_box(self):
        a = random.uniform(0.1, 0.3)
        b = random.uniform(0.1, 0.4)
        c = random.uniform(0.1, 0.5)
        density = 1.0

        box = Box(a=a, b=b, c=c, density=density)
        box.build()
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
        box.plot()

        self.assertAlmostEqual(box.mass, box.mass_analytical)
        numpy.testing.assert_array_almost_equal(box.center_of_mass, box.center_of_mass_analytical)
        numpy.testing.assert_array_almost_equal(box.inertia_tensor, box.inertia_tensor_analytical)

    def test_inertia_of_box_with_random_pos(self):
        a = random.uniform(0.1, 0.3)
        b = random.uniform(0.1, 0.4)
        c = random.uniform(0.1, 0.5)
        pos = numpy.array([[random.uniform(-0.5, 0.5), random.uniform(-0.3, 0.3), random.uniform(-0.2, 0.2)]])
        density = 2.0

        box = Box(a=a, b=b, c=c, density=density, pos=pos)
        box.build()
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
        sphere.plot()

        self.assertAlmostEqual(sphere.mass, sphere.mass_analytical, places=0)
        numpy.testing.assert_array_almost_equal(sphere.center_of_mass, sphere.center_of_mass_analytical)
        numpy.testing.assert_array_almost_equal(sphere.inertia_tensor, sphere.inertia_tensor_analytical, decimal=1)

    def test_inertia_of_sphere_with_random_pos(self):
        radius = random.uniform(0.5, 1.0)
        density = 1.0
        pos = numpy.array([[0.5, 0.5, 0.5]])

        sphere = Sphere(radius=radius,
                        density=density,
                        pos=pos,
                        num_segments=random.randint(6, 32),
                        num_slices=16)
        sphere.build()
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
        cylinder.plot()

        self.assertAlmostEqual(cylinder.mass, cylinder.mass_analytical, places=0)
        numpy.testing.assert_array_almost_equal(cylinder.center_of_mass, cylinder.center_of_mass_analytical)
        numpy.testing.assert_array_almost_equal(cylinder.inertia_tensor, cylinder.inertia_tensor_analytical, decimal=1)

    def test_inertia_of_cylinder_with_random_pos(self):
        radius = random.uniform(0.5, 1.0)
        height = random.uniform(1.0, 2.0)
        density = 1.0
        pos = numpy.array([[random.uniform(-0.5, 0.5), random.uniform(-0.3, 0.3), random.uniform(-0.2, 0.2)]])

        cylinder = Cylinder(radius=radius,
                            height=height,
                            density=density,
                            pos=pos,
                            num_segments=random.randint(6, 32),
                            num_slices=random.randint(16, 32))
        cylinder.build()
        cylinder.plot()

        self.assertAlmostEqual(cylinder.mass, cylinder.mass_analytical, places=0)
        numpy.testing.assert_array_almost_equal(cylinder.center_of_mass, cylinder.center_of_mass_analytical)
        numpy.testing.assert_array_almost_equal(cylinder.inertia_tensor, cylinder.inertia_tensor_analytical, decimal=1)

    def test_inertia_of_mesh_1(self):
        mesh_file_path = os.path.join(self.resource_path, "input", "milk_box", "meshes", "usd", "obj", "milk_box.usda")
        density = 1.0

        mesh = Mesh(mesh_file_path=mesh_file_path, density=density)
        mesh.build()
        mesh.plot()

        a = 0.06
        b = 0.095
        c = 0.2
        mass_analytical = a * b * c * density
        center_of_mass_analytical = numpy.zeros((1, 3))
        inertia_tensor_analytical = numpy.array([[1.0 / 12 * mass_analytical * (b ** 2 + c ** 2), 0.0, 0.0],
                                                 [0.0, 1.0 / 12 * mass_analytical * (a ** 2 + c ** 2), 0.0],
                                                 [0.0, 0.0, 1.0 / 12 * mass_analytical * (a ** 2 + b ** 2)]])

        self.assertAlmostEqual(mesh.mass, mass_analytical)
        numpy.testing.assert_array_almost_equal(mesh.center_of_mass, center_of_mass_analytical)
        numpy.testing.assert_array_almost_equal(mesh.inertia_tensor, inertia_tensor_analytical)

    @classmethod
    def tearDownClass(cls):
        tracemalloc.stop()


if __name__ == '__main__':
    unittest.main()
