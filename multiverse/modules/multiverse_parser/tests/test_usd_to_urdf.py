import unittest

import os
import tracemalloc

import numpy

from multiverse_parser import UsdImporter, UrdfExporter


class UsdToUrdfTestCase(unittest.TestCase):
    resource_path: str

    @classmethod
    def setUpClass(cls):
        tracemalloc.start()
        cls.resource_path = os.path.join(os.path.dirname(__file__), "..", "resources")

    def test_usd_to_urdf(self):
        input_usd_path = os.path.join(self.resource_path, "input", "apartment", "usd", "Apartment.usda")
        factory = UsdImporter(usd_file_path=input_usd_path,
                              with_physics=True,
                              with_visual=True,
                              with_collision=True)
        factory.config.default_rgba = numpy.array([0.9, 0.9, 0.9, 1.0])

        usd_file_path = factory.import_model()
        self.assertTrue(os.path.exists(usd_file_path))

        output_urdf_path = os.path.join(self.resource_path, "output", "test_usd_to_urdf", "apartment", "Apartment.urdf")
        exporter = UrdfExporter(file_path=output_urdf_path,
                                factory=factory,
                                relative_to_ros_package=None)
        exporter.build()
        exporter.export()

    @classmethod
    def tearDownClass(cls):
        tracemalloc.stop()


if __name__ == '__main__':
    unittest.main()
