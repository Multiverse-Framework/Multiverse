import unittest

import os
import tracemalloc

import numpy

from multiverse_parser import UrdfImporter, InertiaSource
from multiverse_parser import MjcfExporter

from pxr import Usd


class UrdfToMjcfTestCase(unittest.TestCase):
    resource_path: str

    @classmethod
    def setUpClass(cls):
        tracemalloc.start()
        cls.resource_path = os.path.join(os.path.dirname(__file__), "..", "resources")

    def test_urdf_to_mjcf_1(self):
        input_urdf_path = os.path.join(self.resource_path, "input", "tiago_dual", "urdf", "tiago_dual_1.urdf")
        factory = UrdfImporter(file_path=input_urdf_path,
                               with_physics=True,
                               with_visual=True,
                               with_collision=True,
                               default_rgba=numpy.array([1.0, 0.0, 0.0, 0.1]))
        self.assertEqual(factory.source_file_path, input_urdf_path)
        self.assertEqual(factory._config.model_name, "tiago_dual")

        usd_file_path = factory.import_model()
        self.assertTrue(os.path.exists(usd_file_path))

        stage = Usd.Stage.Open(usd_file_path)
        default_prim = stage.GetDefaultPrim()
        self.assertEqual(default_prim.GetName(), "tiago_dual")

        output_mjcf_path = os.path.join(self.resource_path, "output", "test_urdf_to_mjcf",  "tiago_dual.xml")
        exporter = MjcfExporter(file_path=output_mjcf_path,
                                factory=factory)
        exporter.build()
        exporter.export()

    def test_urdf_to_mjcf_2(self):
        input_urdf_path = os.path.join(self.resource_path, "input", "ur5e", "urdf", "ur5e.urdf")
        factory = UrdfImporter(file_path=input_urdf_path,
                               with_physics=True,
                               with_visual=True,
                               with_collision=True,
                               default_rgba=numpy.array([1.0, 0.0, 0.0, 0.1]))
        self.assertEqual(factory.source_file_path, input_urdf_path)
        self.assertEqual(factory._config.model_name, "ur5e")

        usd_file_path = factory.import_model()
        self.assertTrue(os.path.exists(usd_file_path))

        output_mjcf_path = os.path.join(self.resource_path, "output", "test_urdf_to_mjcf",  "ur5e.xml")
        exporter = MjcfExporter(file_path=output_mjcf_path,
                                factory=factory)
        exporter.build()
        exporter.export()

    def test_urdf_to_mjcf_3(self):
        input_urdf_path = "/media/giangnguyen/Storage/Multiverse/multiverse/modules/multiverse_parser/resources/output/test_mjcf_to_urdf/anymal_b.urdf"
        factory = UrdfImporter(file_path=input_urdf_path,
                               with_physics=True,
                               with_visual=True,
                               with_collision=True,
                               default_rgba=numpy.array([1.0, 0.0, 0.0, 0.1]))
        self.assertEqual(factory.source_file_path, input_urdf_path)
        self.assertEqual(factory._config.model_name, "anymal_b")

        usd_file_path = factory.import_model()
        self.assertTrue(os.path.exists(usd_file_path))

        output_mjcf_path = os.path.join(self.resource_path, "output", "test_urdf_to_mjcf",  "anymal_b.xml")
        exporter = MjcfExporter(file_path=output_mjcf_path,
                                factory=factory)
        exporter.build()
        exporter.export()

    @classmethod
    def tearDownClass(cls):
        tracemalloc.stop()


if __name__ == '__main__':
    unittest.main()
