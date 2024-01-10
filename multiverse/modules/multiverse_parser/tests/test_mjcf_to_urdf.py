import unittest

import os
import tracemalloc

import numpy

from multiverse_parser import MjcfImporter, InertiaSource, UrdfExporter

from pxr import Usd


class MjcfToUrdfTestCase(unittest.TestCase):
    resource_path: str

    @classmethod
    def setUpClass(cls):
        tracemalloc.start()
        cls.resource_path = os.path.join(os.path.dirname(__file__), "..", "resources")

    def test_mjcf_to_urdf(self):
        input_mjcf_path = os.path.join(self.resource_path, "input", "ur5e", "mjcf", "ur5e_1.xml")
        factory = MjcfImporter(file_path=input_mjcf_path,
                               with_physics=True,
                               with_visual=True,
                               with_collision=False)
        factory.config.default_rgba = numpy.array([1.0, 0.0, 0.0, 0.1])
        self.assertEqual(factory.source_file_path, input_mjcf_path)
        self.assertEqual(factory._config.model_name, "ur5e")

        usd_file_path = factory.import_model()
        self.assertTrue(os.path.exists(usd_file_path))

        stage = Usd.Stage.Open(usd_file_path)
        default_prim = stage.GetDefaultPrim()
        self.assertEqual(default_prim.GetName(), "ur5e")

        output_usd_path = os.path.join(self.resource_path, "output", "test_mjcf_to_urdf", "ur5e.usda")
        factory.save_tmp_model(file_path=output_usd_path)

        output_urdf_path = os.path.join(self.resource_path, "output", "test_mjcf_to_urdf", "ur5e.urdf")
        exporter = UrdfExporter(file_path=output_urdf_path,
                                factory=factory,
                                relative_to_ros_package=False)
        exporter.build()
        exporter.export()

    def test_mjcf_to_urdf_2(self):
        input_mjcf_path = "/media/giangnguyen/Storage/mujoco_menagerie/shadow_hand/left_hand.xml"
        factory = MjcfImporter(file_path=input_mjcf_path,
                               with_physics=True,
                               with_visual=False,
                               with_collision=False)
        factory.config.default_rgba = numpy.array([1.0, 0.0, 0.0, 0.1])
        self.assertEqual(factory.source_file_path, input_mjcf_path)
        self.assertEqual(factory._config.model_name, "left_shadow_hand")

        usd_file_path = factory.import_model()
        self.assertTrue(os.path.exists(usd_file_path))

        stage = Usd.Stage.Open(usd_file_path)
        default_prim = stage.GetDefaultPrim()
        self.assertEqual(default_prim.GetName(), "left_shadow_hand")

        output_usd_path = os.path.join(self.resource_path, "output", "test_mjcf_to_urdf", "left_shadow_hand.usda")
        factory.save_tmp_model(file_path=output_usd_path)

        output_urdf_path = os.path.join(self.resource_path, "output", "test_mjcf_to_urdf", "left_shadow_hand.urdf")
        exporter = UrdfExporter(file_path=output_urdf_path,
                                factory=factory,
                                relative_to_ros_package=False)
        exporter.build()
        exporter.export()

    @classmethod
    def tearDownClass(cls):
        tracemalloc.stop()


if __name__ == '__main__':
    unittest.main()
