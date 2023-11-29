import unittest

import os
import tracemalloc
from pxr import Usd
from multiverse_parser import WorldBuilder, BodyBuilder


class FactoryTestCase(unittest.TestCase):
    resource_path: str

    @classmethod
    def setUpClass(cls):
        tracemalloc.start()
        cls.resource_path = os.path.join(os.path.dirname(__file__), "..", "resources")

    def test_world_builder(self):
        world_builder = WorldBuilder()
        self.assertIsInstance(world_builder.stage, Usd.Stage)

        body_builder_0 = world_builder.add_body(body_name="body_0")
        self.assertEqual(body_builder_0.path, "/body_0")

        body_builder_1 = world_builder.add_body(body_name="body_1", parent_body_name="body_0")
        self.assertEqual(body_builder_1.path, "/body_0/body_1")

        body_builder_2 = world_builder.add_body(body_name="body_2", parent_body_name="body_0")
        self.assertEqual(body_builder_2.path, "/body_0/body_2")

        body_builder_3 = world_builder.add_body(body_name="body_3", parent_body_name="body_2")
        self.assertEqual(body_builder_3.path, "/body_0/body_2/body_3")

        body_builder_4 = world_builder.add_body(body_name="body_1", parent_body_name="body_2")
        self.assertEqual(body_builder_4, body_builder_1)

        with self.assertRaises(ValueError):
            world_builder.add_body(body_name="body_4", parent_body_name="body_abcxyz")

        usd_file_path = world_builder.export()
        self.assertTrue(os.path.exists(usd_file_path))

    @classmethod
    def tearDownClass(cls):
        tracemalloc.stop()




if __name__ == '__main__':
    unittest.main()
