import unittest

import os
import tracemalloc

import numpy

from multiverse_parser import MjcfImporter, InertiaSource

from pxr import Usd


class UrdfToUsdTestCase(unittest.TestCase):
    resource_path: str

    @classmethod
    def setUpClass(cls):
        tracemalloc.start()
        cls.resource_path = os.path.join(os.path.dirname(__file__), "..", "resources")

    def test_mjcf_to_usd_1(self):
        input_mjcf_path = os.path.join(self.resource_path, "input", "ur5e", "mjcf", "ur5e_1.xml")
        factory = MjcfImporter(file_path=input_mjcf_path,
                               with_physics=True,
                               with_visual=True,
                               with_collision=True,
                               inertia_source=InertiaSource.FROM_SRC)
        factory.config.default_rgba = numpy.array([1.0, 0.0, 0.0, 0.1])
        self.assertEqual(factory.source_file_path, input_mjcf_path)
        self.assertEqual(factory._config.model_name, "ur5e")

        usd_file_path = factory.import_model()
        self.assertTrue(os.path.exists(usd_file_path))

        stage = Usd.Stage.Open(usd_file_path)
        default_prim = stage.GetDefaultPrim()
        self.assertEqual(default_prim.GetName(), "ur5e")

        output_usd_path = os.path.join(self.resource_path, "output", "test_mjcf_to_usd", "ur5e_1.usda")
        factory.save_tmp_model(usd_file_path=output_usd_path)
        self.assertTrue(os.path.exists(output_usd_path))

    def test_mjcf_to_usd_2(self):
        input_mjcf_path = os.path.join(self.resource_path, "input", "ur5e", "mjcf", "ur5e_2.xml")
        factory = MjcfImporter(file_path=input_mjcf_path,
                               with_physics=True,
                               with_visual=True,
                               with_collision=False,
                               inertia_source=InertiaSource.FROM_MESH)

        factory.import_model()

        output_usd_path = os.path.join(self.resource_path, "output", "test_mjcf_to_usd", "ur5e_2.usda")
        factory.save_tmp_model(usd_file_path=output_usd_path)
        self.assertTrue(os.path.exists(output_usd_path))

    def test_mjcf_to_usd_3(self):
        input_mjcf_path = os.path.join(self.resource_path, "input", "ur5e", "mjcf", "ur5e_1.xml")
        factory = MjcfImporter(file_path=input_mjcf_path,
                               with_physics=True,
                               with_visual=True,
                               with_collision=False,
                               inertia_source=InertiaSource.FROM_MESH)

        factory.import_model()

        output_usd_path = os.path.join(self.resource_path, "output", "test_mjcf_to_usd", "ur5e_3.usda")
        factory.save_tmp_model(usd_file_path=output_usd_path)
        self.assertTrue(os.path.exists(output_usd_path))

    def test_mjcf_to_usd_with_invalid_file_path(self):
        with self.assertRaises(FileNotFoundError):
            MjcfImporter(file_path="abcxyz",
                         with_physics=True,
                         with_visual=True,
                         with_collision=True)

    @classmethod
    def tearDownClass(cls):
        tracemalloc.stop()


if __name__ == '__main__':
    unittest.main()
