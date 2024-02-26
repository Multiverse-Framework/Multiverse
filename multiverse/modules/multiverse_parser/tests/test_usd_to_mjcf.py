import unittest

import os
import tracemalloc

import numpy

from multiverse_parser import UsdImporter, MjcfExporter


class UsdToMjcfTestCase(unittest.TestCase):
    resource_path: str

    @classmethod
    def setUpClass(cls):
        tracemalloc.start()
        cls.resource_path = os.path.join(os.path.dirname(__file__), "..", "resources")

    def test_usd_to_mjcf(self):
        input_usd_path = os.path.join(self.resource_path, "input", "apartment", "usd", "Apartment.usda")
        factory = UsdImporter(file_path=input_usd_path,
                              with_physics=True,
                              with_visual=True,
                              with_collision=True,
                              add_xform_for_each_geom=True)
        factory.config.default_rgba = numpy.array([0.9, 0.9, 0.9, 1.0])

        usd_file_path = factory.import_model()
        self.assertTrue(os.path.exists(usd_file_path))

        output_urdf_path = os.path.join(self.resource_path, "output", "test_usd_to_mjcf", "apartment", "Apartment.xml")
        exporter = MjcfExporter(file_path=output_urdf_path,
                                factory=factory)
        exporter.build()
        exporter.export()

    @classmethod
    def tearDownClass(cls):
        tracemalloc.stop()


if __name__ == '__main__':
    unittest.main()
