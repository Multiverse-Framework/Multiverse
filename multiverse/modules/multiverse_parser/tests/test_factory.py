import math
import unittest

import os
import tracemalloc

import numpy
from pxr import Usd
from multiverse_parser import (WorldBuilder,
                               JointBuilder, JointType, JointProperty,
                               GeomType, GeomProperty)
from multiverse_parser.utils import calculate_mesh_inertial


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

    def test_inertia_of_cube(self):
        # Define the vertices of a cube
        half_length = 2.0
        density = 2.0
        vertices = numpy.array([
            [-half_length, -half_length, -half_length],
            [half_length, -half_length, -half_length],
            [half_length, half_length, -half_length],
            [-half_length, half_length, -half_length],
            [-half_length, -half_length, half_length],
            [half_length, -half_length, half_length],
            [half_length, half_length, half_length],
            [-half_length, half_length, half_length]
        ])

        # Define the faces of the cube (vertices' indices for each face)
        faces = numpy.array([
            [0, 1, 2],
            [0, 2, 3],
            [1, 5, 6],
            [6, 2, 1],
            [5, 4, 7],
            [7, 6, 5],
            [4, 0, 3],
            [3, 7, 4],
            [3, 2, 6],
            [6, 7, 3],
            [4, 5, 1],
            [1, 0, 4],
        ])
        mass, inertia_tensor, center_of_mass = calculate_mesh_inertial(vertices=vertices, faces=faces, density=density)

        mass_analytical = density * (2 * half_length) ** 3
        center_of_mass_analytical = numpy.array([0.0, 0.0, 0.0])
        inertia_tensor_analytical = numpy.array([[1.0 / 6, 0.0, 0.0],
                                                 [0.0, 1.0 / 6, 0.0],
                                                 [0.0, 0.0, 1.0 / 6]]) * mass_analytical * (2 * half_length) ** 2

        self.assertTrue(numpy.allclose(mass, mass_analytical))
        self.assertTrue(numpy.allclose(center_of_mass, center_of_mass_analytical))
        self.assertTrue(numpy.allclose(inertia_tensor, inertia_tensor_analytical))

    def test_inertia_of_sphere(self):
        num_segments = 32
        num_slices = 128
        radius = 1.0
        density = 1.0

        segment_angle = math.pi / num_segments
        slice_angle = 2 * math.pi / num_slices

        # Define the vertices of a sphere
        vertices = []
        for i in range(num_segments + 1):
            if i == 0:
                vertices.append([0.0, 0.0, radius])
                continue
            elif i == num_segments:
                vertices.append([0.0, 0.0, -radius])
                continue
            for j in range(num_slices):
                x = radius * math.sin(segment_angle * i) * math.cos(slice_angle * j)
                y = radius * math.sin(segment_angle * i) * math.sin(slice_angle * j)
                z = radius * math.cos(segment_angle * i)
                vertices.append([x, y, z])

        # Define the faces of the sphere (vertices' indices for each face)
        faces = []
        for i in range(num_segments):
            if i == 0:
                for j in range(num_slices):
                    faces.append([0,
                                  j + 1,
                                  (j + 1) % num_slices + 1])
            elif i == num_segments - 1:
                for j in range(num_slices):
                    faces.append([(num_segments - 1) * num_slices + 1,
                                  (i - 1) * num_slices + j + 1,
                                  (i - 1) * num_slices + (j + 1) % num_slices + 1])
            else:
                for j in range(num_slices):
                    faces.append([(i - 1) * num_slices + j + 1,
                                  (i - 1) * num_slices + (j + 1) % num_slices + 1,
                                  i * num_slices + (j + 1) % num_slices + 1])
                    faces.append([(i - 1) * num_slices + j + 1,
                                  i * num_slices + (j + 1) % num_slices + 1,
                                  i * num_slices + j + 1])

        vertices = numpy.array(vertices)
        faces = numpy.array(faces)

        # Plot the sphere and faces
        import matplotlib.pyplot as plt
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.set_aspect("equal")
        ax.set_xlim(-radius, radius)
        ax.set_ylim(-radius, radius)
        ax.set_zlim(-radius, radius)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        # ax.plot(vertices[:, 0], vertices[:, 1], vertices[:, 2], "o")
        for face in faces:
            ax.plot(vertices[face, 0], vertices[face, 1], vertices[face, 2], "-")
        plt.show()

        mass, inertia_tensor, center_of_mass = calculate_mesh_inertial(vertices=vertices, faces=faces, density=density)

        mass_analytical = density * 4 / 3 * math.pi * radius ** 3
        center_of_mass_analytical = numpy.array([0.0, 0.0, 0.0])
        inertia_tensor_analytical = numpy.array([[2.0 / 5, 0.0, 0.0],
                                                 [0.0, 2.0 / 5, 0.0],
                                                 [0.0, 0.0, 2.0 / 5]]) * mass_analytical * radius ** 2

        self.assertAlmostEqual(mass, mass_analytical, places=1)
        self.assertTrue(numpy.allclose(center_of_mass, center_of_mass_analytical))
        self.assertTrue(numpy.allclose(inertia_tensor, inertia_tensor_analytical, rtol=1e-1))

    def test_inertia_of_cylinder(self):
        num_segments = 10
        num_slices = 128
        radius = 1.0
        height = 2.0
        density = 1.0

        segment_height = height / num_segments
        slice_angle = 2 * math.pi / num_slices

        # Define the vertices of a cylinder
        vertices = []
        for i in range(num_segments + 2):
            if i == 0:
                vertices.append([0.0, 0.0, -height / 2])
            elif i == num_segments + 1:
                vertices.append([0.0, 0.0, height / 2])
                break
            for j in range(num_slices):
                x = radius * math.cos(slice_angle * j)
                y = radius * math.sin(slice_angle * j)
                z = -height / 2 + segment_height * i
                vertices.append([x, y, z])

        # Define the faces of the sphere (vertices' indices for each face)
        faces = []
        for i in range(num_segments + 2):
            if i == 0:
                for j in range(num_slices):
                    faces.append([0,
                                  j + 1,
                                  (j + 1) % num_slices + 1])
            elif i == num_segments + 1:
                for j in range(num_slices):
                    faces.append([(num_segments + 1) * num_slices + 1,
                                  (i - 1) * num_slices + j + 1,
                                  (i - 1) * num_slices + (j + 1) % num_slices + 1])
            else:
                for j in range(num_slices):
                    faces.append([(i - 1) * num_slices + j + 1,
                                  (i - 1) * num_slices + (j + 1) % num_slices + 1,
                                  i * num_slices + (j + 1) % num_slices + 1])
                    faces.append([(i - 1) * num_slices + j + 1,
                                  i * num_slices + (j + 1) % num_slices + 1,
                                  i * num_slices + j + 1])

        vertices = numpy.array(vertices)
        faces = numpy.array(faces)

        # Plot the sphere and faces
        import matplotlib.pyplot as plt
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.set_aspect("equal")
        ax.set_xlim(-radius, radius)
        ax.set_ylim(-radius, radius)
        ax.set_zlim(-radius, radius)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        ax.plot(vertices[:, 0], vertices[:, 1], vertices[:, 2], "o")
        for face in faces:
            ax.plot(vertices[face, 0], vertices[face, 1], vertices[face, 2], "-")
        plt.show()

        mass, inertia_tensor, center_of_mass = calculate_mesh_inertial(vertices=vertices, faces=faces, density=density)

        mass_analytical = density * math.pi * radius ** 2 * height
        center_of_mass_analytical = numpy.array([0.0, 0.0, 0.0])
        inertia_tensor_analytical = numpy.array([[1.0 / 12 * mass_analytical * (3 * radius ** 2 + height ** 2), 0.0, 0.0],
                                                 [0.0, 1.0 / 12 * mass_analytical * (3 * radius ** 2 + height ** 2), 0.0],
                                                 [0.0, 0.0, 1.0 / 2 * mass_analytical * radius ** 2]])

        self.assertAlmostEqual(mass, mass_analytical, places=2)
        self.assertTrue(numpy.allclose(center_of_mass, center_of_mass_analytical))
        self.assertTrue(numpy.allclose(inertia_tensor, inertia_tensor_analytical, rtol=1e-3))

    @classmethod
    def tearDownClass(cls):
        tracemalloc.stop()


if __name__ == '__main__':
    unittest.main()
