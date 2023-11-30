import math
import unittest

import os
import tracemalloc
from pxr import Usd
from multiverse_parser import WorldBuilder, BodyBuilder, JointBuilder, JointType


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

        body_builder_1.set_transform(pos=(1.0, 0.0, 0.0),
                                     quat=(math.sqrt(2) / 2, 0.0, 0.0, math.sqrt(2) / 2),
                                     scale=(1.0, 1.0, 1.0))
        body_1_xform = body_builder_1.xform
        body_1_local_transform = body_1_xform.GetLocalTransformation()
        body_1_local_pos = body_1_local_transform.ExtractTranslation()
        self.assertEqual(body_1_local_pos, (1.0, 0.0, 0.0))

        body_builder_3.set_transform(pos=(0.0, 1.0, 0.0),
                                     quat=(-math.sqrt(2) / 2, 0.0, 0.0, math.sqrt(2) / 2),
                                     scale=(1.0, 1.0, 1.0),
                                     relative_to_xform=body_1_xform)

        joint_builder = JointBuilder(stage=world_builder._stage,
                                     name="joint_0",
                                     parent_prim=body_builder_1.xform.GetPrim(),
                                     child_prim=body_builder_3.xform.GetPrim(),
                                     joint_type=JointType.FIXED)
        self.assertEqual(joint_builder.path, "/body_0/body_1/joint_0")
        self.assertEqual(joint_builder.type, JointType.FIXED)

        world_builder.export()
        self.assertTrue(os.path.exists(file_path))

    @classmethod
    def tearDownClass(cls):
        tracemalloc.stop()


if __name__ == '__main__':
    unittest.main()
