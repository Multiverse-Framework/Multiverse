import unittest

import tracemalloc

from multiverse_parser import MjcfImporter, UrdfImporter, UsdImporter
from multiverse_parser import MjcfExporter, UrdfExporter

import os
import numpy
from multiverse_parser import InertiaSource, Configuration

from pxr import Usd, UsdGeom, UsdPhysics
from mujoco import MjModel, mjMINVAL
from urdf_parser_py import urdf


class MultiverseParserTestCase(unittest.TestCase):
    output_dir: str
    resource_path: str

    @classmethod
    def setUpClass(cls):
        tracemalloc.start()
        cls.resource_path = os.path.join(os.path.dirname(__file__), "..", "resources")

    @classmethod
    def tearDownClass(cls):
        tracemalloc.stop()


class MultiverseImporterTestCase(MultiverseParserTestCase):
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

    def validate_visual_collision(self, importer, input_path, fixed_base, with_physics, **kwargs):
        input_file = os.path.basename(input_path)
        model_name = os.path.splitext(input_file)[0]

        is_visible = [True, True, False, False]
        is_collidable = [True, False, True, False]
        inertia_sources = [InertiaSource.FROM_SRC, InertiaSource.FROM_VISUAL_MESH, InertiaSource.FROM_COLLISION_MESH,
                           InertiaSource.FROM_SRC]

        for i in range(4):
            factory = importer(file_path=input_path,
                               fixed_base=fixed_base,
                               with_physics=with_physics,
                               with_visual=is_visible[i],
                               with_collision=is_collidable[i],
                               inertia_source=inertia_sources[i],
                               **kwargs)
            output_usd_path = os.path.join(self.resource_path, "output", self.output_dir,
                                           f"{model_name}_{i}_{'with_physics' if with_physics else 'no_physics'}.usda")
            self.validate_output(factory, model_name, output_usd_path)
            stage = Usd.Stage.Open(output_usd_path)
            for prim in stage.Traverse():
                if not with_physics:
                    self.assertFalse(prim.IsA(UsdPhysics.Joint))
                if prim.IsA(UsdGeom.Xform):
                    if (prim == stage.GetDefaultPrim()
                            and (any([childPrim.IsA(UsdGeom.Xform) for childPrim in prim.GetChildren()])
                                 or fixed_base)):
                        continue
                    if not prim.GetParent().IsA(UsdGeom.Xform):
                        self.assertEqual(prim.HasAPI(UsdPhysics.MassAPI), with_physics)
                        self.assertEqual(prim.HasAPI(UsdPhysics.RigidBodyAPI), with_physics)
                if not prim.IsA(UsdGeom.Gprim):
                    continue
                if not is_visible[i]:
                    self.assertEqual(prim.GetProperty("visibility").Get(), UsdGeom.Tokens.invisible)
                if not is_collidable[i]:
                    self.assertFalse(prim.HasAPI(UsdPhysics.CollisionAPI))


class MultiverseExporterTestCase(MultiverseParserTestCase):
    model_ext: str

    def validate_visual_collision(self, importer, exporter, input_path, fixed_base, with_physics, **kwargs):
        model_name = os.path.splitext(os.path.basename(input_path))[0]

        is_visible = [True, True, False, False]
        is_collidable = [True, False, True, False]
        inertia_sources = [InertiaSource.FROM_SRC, InertiaSource.FROM_VISUAL_MESH, InertiaSource.FROM_COLLISION_MESH,
                           InertiaSource.FROM_SRC]

        for i in range(4):
            factory = importer(file_path=input_path,
                               fixed_base=fixed_base,
                               with_physics=with_physics,
                               with_visual=is_visible[i],
                               with_collision=is_collidable[i],
                               inertia_source=inertia_sources[i],
                               **kwargs)
            factory.import_model()
            output_path = os.path.join(self.resource_path, "output", self.output_dir,
                                       f"{model_name}_{i}_{'with_physics' if with_physics else 'no_physics'}."
                                       f"{self.model_ext}")
            exporter_ = exporter(file_path=output_path,
                                 factory=factory)
            exporter_.build()
            exporter_.export(keep_usd=False)

            self.validate_output(output_path, factory.config)

    def validate_output(self, output_path: str, config: Configuration):
        raise NotImplementedError


class MjcfExporterTestCase(MultiverseExporterTestCase):
    model_ext = "xml"

    def validate_output(self, output_path: str, config: Configuration):
        mj_model = MjModel.from_xml_path(filename=output_path)
        for geom_id in range(mj_model.ngeom):
            geom = mj_model.geom(geom_id)
            if not config.with_visual:
                numpy.testing.assert_array_almost_equal(geom.rgba, config.default_rgba)
            if not config.with_collision:
                self.assertEqual(geom.contype[0], 0)
                self.assertEqual(geom.conaffinity[0], 0)

        for body_id in range(1, mj_model.nbody):
            body = mj_model.body(body_id)
            if body_id == 1 and config.fixed_base:
                continue
            if config.with_physics:
                self.assertGreater(body.mass[0], mjMINVAL)
            else:
                self.assertEqual(body.dofnum, 0)


class UrdfExporterTestCase(MultiverseExporterTestCase):
    model_ext = "urdf"

    def validate_output(self, output_path: str, config: Configuration):
        urdf_model = urdf.URDF.from_xml_file(output_path)
        for link in urdf_model.links:
            if not config.with_visual:
                self.assertEqual(link.visual, None)
            if not config.with_collision:
                self.assertEqual(link.collision, None)
            if not config.with_physics:
                self.assertEqual(link.inertial, None)

        for joint in urdf_model.joints:
            if not config.with_physics:
                self.assertEqual(joint.type, "fixed")


class UsdToUsdTestCase(MultiverseImporterTestCase):
    output_dir = "test_usd_to_usd"

    def test_usd_to_usd_milk_box(self):
        input_usd_path = os.path.join(self.resource_path, "input", "milk_box", "usd", "milk_box.usda")
        self.validate_visual_collision(UsdImporter, input_usd_path, fixed_base=False, with_physics=True)
        self.validate_visual_collision(UsdImporter, input_usd_path, fixed_base=False, with_physics=False)

    def test_usd_to_usd_furniture(self):
        input_usd_path = os.path.join(self.resource_path, "input", "furniture", "furniture.usda")
        self.validate_visual_collision(UsdImporter, input_usd_path, fixed_base=True, with_physics=True,
                                       add_xform_for_each_geom=True)
        self.validate_visual_collision(UsdImporter, input_usd_path, fixed_base=True, with_physics=False,
                                       add_xform_for_each_geom=True)


class MjcfToUsdTestCase(MultiverseImporterTestCase):
    output_dir = "test_mjcf_to_usd"

    def test_mjcf_to_usd_milk_box(self):
        input_mjcf_path = os.path.join(self.resource_path, "input", "milk_box", "mjcf", "milk_box.xml")
        self.validate_visual_collision(MjcfImporter, input_mjcf_path, fixed_base=False, with_physics=True)
        self.validate_visual_collision(MjcfImporter, input_mjcf_path, fixed_base=False, with_physics=False)

    def test_mjcf_to_usd_ur5e(self):
        input_mjcf_path = os.path.join(self.resource_path, "input", "ur5e", "mjcf", "ur5e.xml")
        self.validate_visual_collision(MjcfImporter, input_mjcf_path, fixed_base=True, with_physics=True)
        self.validate_visual_collision(MjcfImporter, input_mjcf_path, fixed_base=True, with_physics=False)

    def test_mjcf_to_usd_anymal_c(self):
        input_mjcf_path = os.path.join(self.resource_path, "input", "anymal_c", "anymal_c.xml")
        self.validate_visual_collision(MjcfImporter, input_mjcf_path, fixed_base=False, with_physics=True)
        self.validate_visual_collision(MjcfImporter, input_mjcf_path, fixed_base=False, with_physics=False)

    @unittest.skip("This test is skipped.")
    def test_mjcf_to_usd_preparing_soup(self):
        input_mjcf_path = os.path.join(self.resource_path, "input", "preparing_soup", "preparing_soup.xml")
        self.validate_visual_collision(MjcfImporter, input_mjcf_path, fixed_base=False, with_physics=True)
        self.validate_visual_collision(MjcfImporter, input_mjcf_path, fixed_base=False, with_physics=False)


class UrdfToUsdTestCase(MultiverseImporterTestCase):
    output_dir = "test_urdf_to_usd"

    def test_urdf_to_usd_milk_box(self):
        input_urdf_path = os.path.join(self.resource_path, "input", "milk_box", "urdf", "milk_box.urdf")
        self.validate_visual_collision(UrdfImporter, input_urdf_path, fixed_base=False, with_physics=True)
        self.validate_visual_collision(UrdfImporter, input_urdf_path, fixed_base=False, with_physics=False)

    def test_urdf_to_usd_ur5e(self):
        input_urdf_path = os.path.join(self.resource_path, "input", "ur5e", "urdf", "ur5e.urdf")
        self.validate_visual_collision(UrdfImporter, input_urdf_path, fixed_base=True, with_physics=True)
        self.validate_visual_collision(UrdfImporter, input_urdf_path, fixed_base=True, with_physics=False)

    def test_urdf_to_usd_tiago_dual(self):
        input_urdf_path = os.path.join(self.resource_path, "input", "tiago_dual", "urdf", "tiago_dual.urdf")
        self.validate_visual_collision(UrdfImporter, input_urdf_path, fixed_base=True, with_physics=True)
        self.validate_visual_collision(UrdfImporter, input_urdf_path, fixed_base=True, with_physics=False)


class UsdToMjcfTestCase(MjcfExporterTestCase):
    output_dir = "test_usd_to_mjcf"

    def test_usd_to_mjcf_milk_box(self):
        input_usd_path = os.path.join(self.resource_path, "input", "milk_box", "usd", "milk_box.usda")
        self.validate_visual_collision(UsdImporter, MjcfExporter, input_usd_path,
                                       fixed_base=False, with_physics=True)
        self.validate_visual_collision(UsdImporter, MjcfExporter, input_usd_path,
                                       fixed_base=False, with_physics=False)

    def test_usd_to_mjcf_furniture(self):
        input_usd_path = os.path.join(self.resource_path, "input", "furniture", "furniture.usda")
        self.validate_visual_collision(UsdImporter, MjcfExporter, input_usd_path,
                                       fixed_base=True, with_physics=True)
        self.validate_visual_collision(UsdImporter, MjcfExporter, input_usd_path,
                                       fixed_base=True, with_physics=False)


class MjcfToMjcfTestCase(MjcfExporterTestCase):
    output_dir = "test_mjcf_to_mjcf"

    def test_mjcf_to_mjcf_milk_box(self):
        input_mjcf_path = os.path.join(self.resource_path, "input", "milk_box", "mjcf", "milk_box.xml")
        self.validate_visual_collision(MjcfImporter, MjcfExporter, input_mjcf_path,
                                       fixed_base=False, with_physics=True)
        self.validate_visual_collision(MjcfImporter, MjcfExporter, input_mjcf_path,
                                       fixed_base=False, with_physics=False)

    def test_mjcf_to_mjcf_ur5e(self):
        input_mjcf_path = os.path.join(self.resource_path, "input", "ur5e", "mjcf", "ur5e.xml")
        self.validate_visual_collision(MjcfImporter, MjcfExporter, input_mjcf_path,
                                       fixed_base=True, with_physics=True)
        self.validate_visual_collision(MjcfImporter, MjcfExporter, input_mjcf_path,
                                       fixed_base=True, with_physics=False)

    def test_mjcf_to_mjcf_anymal_c(self):
        input_mjcf_path = os.path.join(self.resource_path, "input", "anymal_c", "anymal_c.xml")
        self.validate_visual_collision(MjcfImporter, MjcfExporter, input_mjcf_path,
                                       fixed_base=False, with_physics=True)
        self.validate_visual_collision(MjcfImporter, MjcfExporter, input_mjcf_path,
                                       fixed_base=False, with_physics=False)

    @unittest.skip("This test is skipped.")
    def test_mjcf_to_mjcf_preparing_soup(self):
        input_mjcf_path = os.path.join(self.resource_path, "input", "preparing_soup", "preparing_soup.xml")
        self.validate_visual_collision(MjcfImporter, MjcfExporter, input_mjcf_path,
                                       fixed_base=False, with_physics=True)
        self.validate_visual_collision(MjcfImporter, MjcfExporter, input_mjcf_path,
                                       fixed_base=False, with_physics=False)


class UrdfToMjcfTestCase(MjcfExporterTestCase):
    output_dir = "test_urdf_to_mjcf"

    def test_urdf_to_mjcf_milk_box(self):
        input_urdf_path = os.path.join(self.resource_path, "input", "milk_box", "urdf", "milk_box.urdf")
        self.validate_visual_collision(UrdfImporter, MjcfExporter, input_urdf_path,
                                       fixed_base=False, with_physics=True)
        self.validate_visual_collision(UrdfImporter, MjcfExporter, input_urdf_path,
                                       fixed_base=False, with_physics=False)

    def test_urdf_to_mjcf_ur5e(self):
        input_urdf_path = os.path.join(self.resource_path, "input", "ur5e", "urdf", "ur5e.urdf")
        self.validate_visual_collision(UrdfImporter, MjcfExporter, input_urdf_path,
                                       fixed_base=True, with_physics=True)
        self.validate_visual_collision(UrdfImporter, MjcfExporter, input_urdf_path,
                                       fixed_base=True, with_physics=False)

    def test_urdf_to_mjcf_tiago_dual(self):
        input_urdf_path = os.path.join(self.resource_path, "input", "tiago_dual", "urdf", "tiago_dual.urdf")
        self.validate_visual_collision(UrdfImporter, MjcfExporter, input_urdf_path,
                                       fixed_base=True, with_physics=True)
        self.validate_visual_collision(UrdfImporter, MjcfExporter, input_urdf_path,
                                       fixed_base=True, with_physics=False)


class UsdToUrdfTestCase(UrdfExporterTestCase):
    output_dir = "test_usd_to_urdf"

    def test_usd_to_urdf_milk_box(self):
        input_usd_path = os.path.join(self.resource_path, "input", "milk_box", "usd", "milk_box.usda")
        self.validate_visual_collision(UsdImporter, UrdfExporter, input_usd_path,
                                       fixed_base=False, with_physics=True)
        self.validate_visual_collision(UsdImporter, UrdfExporter, input_usd_path,
                                       fixed_base=False, with_physics=False)

    def test_usd_to_urdf_furniture(self):
        input_usd_path = os.path.join(self.resource_path, "input", "furniture", "furniture.usda")
        self.validate_visual_collision(UsdImporter, UrdfExporter, input_usd_path,
                                       fixed_base=True, with_physics=True)
        self.validate_visual_collision(UsdImporter, UrdfExporter, input_usd_path,
                                       fixed_base=True, with_physics=False)


class MjcfToUrdfTestCase(UrdfExporterTestCase):
    output_dir = "test_mjcf_to_urdf"

    def test_mjcf_to_urdf_milk_box(self):
        input_mjcf_path = os.path.join(self.resource_path, "input", "milk_box", "mjcf", "milk_box.xml")
        self.validate_visual_collision(MjcfImporter, UrdfExporter, input_mjcf_path,
                                       fixed_base=False, with_physics=True)
        self.validate_visual_collision(MjcfImporter, UrdfExporter, input_mjcf_path,
                                       fixed_base=False, with_physics=False)

    def test_mjcf_to_urdf_ur5e(self):
        input_mjcf_path = os.path.join(self.resource_path, "input", "ur5e", "mjcf", "ur5e.xml")
        self.validate_visual_collision(MjcfImporter, UrdfExporter, input_mjcf_path,
                                       fixed_base=True, with_physics=True)
        self.validate_visual_collision(MjcfImporter, UrdfExporter, input_mjcf_path,
                                       fixed_base=True, with_physics=False)

    def test_mjcf_to_urdf_anymal_c(self):
        input_mjcf_path = os.path.join(self.resource_path, "input", "anymal_c", "anymal_c.xml")
        self.validate_visual_collision(MjcfImporter, UrdfExporter, input_mjcf_path,
                                       fixed_base=False, with_physics=True)
        self.validate_visual_collision(MjcfImporter, UrdfExporter, input_mjcf_path,
                                       fixed_base=False, with_physics=False)

    def test_mjcf_to_urdf_panda(self):
        input_mjcf_path = os.path.join(self.resource_path, "input", "mujoco_menagerie", "franka_emika_panda", "panda.xml")
        self.validate_visual_collision(MjcfImporter, UrdfExporter, input_mjcf_path,
                                       fixed_base=False, with_physics=True)
        self.validate_visual_collision(MjcfImporter, UrdfExporter, input_mjcf_path,
                                       fixed_base=False, with_physics=False)


class UrdfToUrdfTestCase(UrdfExporterTestCase):
    output_dir = "test_urdf_to_urdf"

    def test_urdf_to_urdf_milk_box(self):
        input_urdf_path = os.path.join(self.resource_path, "input", "milk_box", "urdf", "milk_box.urdf")
        self.validate_visual_collision(UrdfImporter, UrdfExporter, input_urdf_path,
                                       fixed_base=False, with_physics=True)
        self.validate_visual_collision(UrdfImporter, UrdfExporter, input_urdf_path,
                                       fixed_base=False, with_physics=False)

    def test_urdf_to_urdf_ur5e(self):
        input_urdf_path = os.path.join(self.resource_path, "input", "ur5e", "urdf", "ur5e.urdf")
        self.validate_visual_collision(UrdfImporter, UrdfExporter, input_urdf_path,
                                       fixed_base=True, with_physics=True)
        self.validate_visual_collision(UrdfImporter, UrdfExporter, input_urdf_path,
                                       fixed_base=True, with_physics=False)

    def test_urdf_to_mjcf_tiago_dual(self):
        input_urdf_path = os.path.join(self.resource_path, "input", "tiago_dual", "urdf", "tiago_dual.urdf")
        self.validate_visual_collision(UrdfImporter, UrdfExporter, input_urdf_path,
                                       fixed_base=True, with_physics=True)
        self.validate_visual_collision(UrdfImporter, UrdfExporter, input_urdf_path,
                                       fixed_base=True, with_physics=False)


if __name__ == '__main__':
    unittest.main()
