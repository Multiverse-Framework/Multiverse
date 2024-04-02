import unittest

import os
import tracemalloc

import numpy

from multiverse_parser import UsdImporter
from multiverse_parser import InertiaSource

class UsdToUsdTestCase(unittest.TestCase):
    resource_path: str

    @classmethod
    def setUpClass(cls):
        tracemalloc.start()
        cls.resource_path = os.path.join(os.path.dirname(__file__), "..", "resources")

    def test_usd_to_usd(self):
        input_usd_path = os.path.join(self.resource_path, "input", "ApartmentECAI", "ApartmentECAI2024Reduce.usda")
        factory = UsdImporter(file_path=input_usd_path,
                              with_physics=True,
                              with_visual=True,
                              with_collision=True,
                              add_xform_for_each_geom=True,
                              inertia_source=InertiaSource.FROM_VISUAL_MESH)
        factory.config.default_rgba = numpy.array([0.9, 0.9, 0.9, 0.0])

        usd_file_path = factory.import_model()
        self.assertTrue(os.path.exists(usd_file_path))

        output_usd_path = os.path.join(self.resource_path, "output", "test_usd_to_usd", "ApartmentECAI2024Reduce", "ApartmentECAI2024Reduce.usda")
        factory.save_tmp_model(usd_file_path=output_usd_path)

    @classmethod
    def tearDownClass(cls):
        tracemalloc.stop()


if __name__ == '__main__':
    unittest.main()
