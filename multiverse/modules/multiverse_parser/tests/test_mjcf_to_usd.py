import unittest

import tracemalloc

from multiverse_parser import MjcfImporter

import os
import numpy
from multiverse_parser import InertiaSource
from pxr import Usd, UsdGeom, UsdPhysics


class MultiverseParserTestCase(unittest.TestCase):
    resource_path: str

    @classmethod
    def setUpClass(cls):
        tracemalloc.start()
        cls.resource_path = os.path.join(os.path.dirname(__file__), "..", "resources")

    @classmethod
    def tearDownClass(cls):
        tracemalloc.stop()

    def validate_output(self, factory, model_name: str, model_path: str):
        factory.config.default_rgba = numpy.array([0.9, 0.9, 0.9, 1.0])
        self.assertEqual(factory._config.model_name, model_name)

        usd_file_path = factory.import_model()
        self.assertTrue(os.path.exists(usd_file_path))

        stage = Usd.Stage.Open(usd_file_path)
        default_prim = stage.GetDefaultPrim()
        self.assertEqual(default_prim.GetName(), model_name)

        factory.save_tmp_model(usd_file_path=model_path)
        self.assertTrue(os.path.exists(model_path))

    def validate_visual_collision(self, importer, input_mjcf_path, fixed_base, with_physics):
        input_mjcf_file = os.path.basename(input_mjcf_path)
        model_name = os.path.splitext(input_mjcf_file)[0]

        is_visible = [True, True, False, False]
        is_collidable = [True, False, True, False]
        inertia_sources = [InertiaSource.FROM_SRC, InertiaSource.FROM_VISUAL_MESH, InertiaSource.FROM_COLLISION_MESH,
                           InertiaSource.FROM_SRC]

        for i in range(4):
            factory = importer(file_path=input_mjcf_path,
                               fixed_base=fixed_base,
                               with_physics=with_physics,
                               with_visual=is_visible[i],
                               with_collision=is_collidable[i],
                               inertia_source=inertia_sources[i])
            output_usd_path = os.path.join(self.resource_path, "output", "test_mjcf_to_usd",
                                           f"{model_name}_{i}_{'with_physics' if with_physics else 'no_physics'}.usda")
            self.validate_output(factory, model_name, output_usd_path)
            stage = Usd.Stage.Open(output_usd_path)
            for prim in stage.Traverse():
                if prim.IsA(UsdGeom.Xform):
                    if prim == stage.GetDefaultPrim() and fixed_base:
                        continue
                    self.assertEqual(prim.HasAPI(UsdPhysics.MassAPI), with_physics)
                    self.assertEqual(prim.HasAPI(UsdPhysics.RigidBodyAPI), with_physics)
                if not prim.IsA(UsdGeom.Gprim):
                    continue
                if not is_visible[i]:
                    self.assertEqual(prim.GetProperty("visibility").Get(), UsdGeom.Tokens.invisible)
                if not is_collidable[i]:
                    self.assertFalse(prim.HasAPI(UsdPhysics.CollisionAPI))


class MjcfToUsdTestCase(MultiverseParserTestCase):
    def test_mjcf_to_usd_milk_box(self):
        input_mjcf_path = os.path.join(self.resource_path, "input", "milk_box", "mjcf", "milk_box.xml")
        self.validate_visual_collision(MjcfImporter, input_mjcf_path, fixed_base=False, with_physics=True)
        self.validate_visual_collision(MjcfImporter, input_mjcf_path, fixed_base=False, with_physics=False)

    def test_mjcf_to_usd_ur5e(self):
        input_mjcf_path = os.path.join(self.resource_path, "input", "ur5e", "mjcf", "ur5e.xml")
        self.validate_visual_collision(MjcfImporter, input_mjcf_path, fixed_base=True, with_physics=True)
        self.validate_visual_collision(MjcfImporter, input_mjcf_path, fixed_base=True, with_physics=False)

    def test_mjcf_to_usd_with_invalid_file_path(self):
        with self.assertRaises(FileNotFoundError):
            MjcfImporter(file_path="abcxyz",
                         with_physics=True,
                         with_visual=True,
                         with_collision=True,
                         fixed_base=False)


if __name__ == '__main__':
    unittest.main()
