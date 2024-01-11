import unittest

import os
import tracemalloc

import numpy

from multiverse_parser import MjcfImporter, InertiaSource, MjcfExporter

from pxr import Usd


class MjcfToMjcfTestCase(unittest.TestCase):
    resource_path: str

    @classmethod
    def setUpClass(cls):
        tracemalloc.start()
        cls.resource_path = os.path.join(os.path.dirname(__file__), "..", "resources")

    def test_mjcf_to_mjcf(self):
        input_mjcf_path = os.path.join(self.resource_path, "input", "ur5e", "mjcf", "ur5e_1.xml")
        factory = MjcfImporter(file_path=input_mjcf_path,
                               with_physics=True,
                               with_visual=True,
                               with_collision=True)
        factory.config.default_rgba = numpy.array([1.0, 0.0, 0.0, 0.1])
        self.assertEqual(factory.source_file_path, input_mjcf_path)
        self.assertEqual(factory._config.model_name, "ur5e")

        usd_file_path = factory.import_model()
        self.assertTrue(os.path.exists(usd_file_path))

        stage = Usd.Stage.Open(usd_file_path)
        default_prim = stage.GetDefaultPrim()
        self.assertEqual(default_prim.GetName(), "ur5e")

        output_mjcf_path = os.path.join(self.resource_path, "output", "test_mjcf_to_mjcf", "ur5e.xml")
        exporter = MjcfExporter(file_path=output_mjcf_path,
                                factory=factory)
        exporter.build()
        exporter.export()

    def test_mjcf_to_mjcf_2(self):
        input_mjcf_path = "/media/giangnguyen/Storage/mujoco_menagerie/anybotics_anymal_c/anymal_c.xml"
        factory = MjcfImporter(file_path=input_mjcf_path,
                               with_physics=True,
                               with_visual=True,
                               with_collision=True)
        factory.config.default_rgba = numpy.array([1.0, 0.0, 0.0, 0.1])
        self.assertEqual(factory.source_file_path, input_mjcf_path)
        self.assertEqual(factory._config.model_name, "anymal_c")

        usd_file_path = factory.import_model()
        self.assertTrue(os.path.exists(usd_file_path))

        stage = Usd.Stage.Open(usd_file_path)
        default_prim = stage.GetDefaultPrim()
        self.assertEqual(default_prim.GetName(), "anymal_c")

        output_usd_path = os.path.join(self.resource_path, "output", "test_mjcf_to_mjcf", "anymal_c.usda")
        factory.save_tmp_model(file_path=output_usd_path)

        output_mjcf_path = os.path.join(self.resource_path, "output", "test_mjcf_to_mjcf", "anymal_c.xml")
        exporter = MjcfExporter(file_path=output_mjcf_path,
                                factory=factory)
        exporter.build()
        exporter.export()

    def test_mjcf_to_mjcf_3(self):
        input_mjcf_path = "/media/giangnguyen/Storage/Multiverse/multiverse/modules/multiverse_parser/resources/input/milk_box/mjcf/milk_box.xml"
        factory = MjcfImporter(file_path=input_mjcf_path,
                               with_physics=True,
                               with_visual=True,
                               with_collision=True)
        factory.config.default_rgba = numpy.array([1.0, 0.0, 0.0, 0.1])
        self.assertEqual(factory.source_file_path, input_mjcf_path)
        self.assertEqual(factory._config.model_name, "milk_box")

        usd_file_path = factory.import_model()
        self.assertTrue(os.path.exists(usd_file_path))

        stage = Usd.Stage.Open(usd_file_path)
        default_prim = stage.GetDefaultPrim()
        self.assertEqual(default_prim.GetName(), "milk_box")

        output_usd_path = os.path.join(self.resource_path, "output", "test_mjcf_to_mjcf", "milk_box.usda")
        factory.save_tmp_model(file_path=output_usd_path)

        output_mjcf_path = os.path.join(self.resource_path, "output", "test_mjcf_to_mjcf", "milk_box.xml")
        exporter = MjcfExporter(file_path=output_mjcf_path,
                                factory=factory)
        exporter.build()
        exporter.export()

    @classmethod
    def tearDownClass(cls):
        tracemalloc.stop()


if __name__ == '__main__':
    unittest.main()
