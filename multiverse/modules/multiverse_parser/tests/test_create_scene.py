import unittest

import os
import shutil
import tracemalloc
import re

import numpy

from multiverse_parser import Factory, Configuration, InertiaSource
from multiverse_parser import (WorldBuilder,
                               JointBuilder, JointType, JointProperty,
                               GeomType, GeomProperty,
                               MeshBuilder, MeshProperty, MaterialProperty)
from multiverse_parser import UrdfImporter, MjcfImporter, UrdfExporter, MjcfExporter
from multiverse_parser.utils import *
from multiverse_parser.utils.mesh_importer import clean_up_meshes_script

from pxr import Usd, UsdGeom, UsdShade


def camel_case_to_snake_case(name: str) -> str:
    name = name[0].lower() + name[1:]
    return ''.join(['_' + i.lower() if i.isupper() else i for i in name]).lstrip('_')


density_dict = {
    "glass": 2500.0,
    "meat": 1000.0,
    "bread": 250.0,
    "fruit": 1000.0,
    "banana": 850.0,
    "grain_buckwheat": 900.0,
    "orange": 900.0,
    "pineapple": 550.0,
    "iceberg_lettuce": 150.0,
    "knife": 7900.0,
    "ceramic": 2500.0,
    "mixer": 8000.0,
    "fork": 7800.0,
    "scissors": 7850.0,
    "target": 90.0,
    "cloth": 1600.0,
    "onion": 520.0,
    "wood": 700.0,
    "cheese": 1320.0,
    "alu": 2700.0,
    "garlic": 960.0,
    "kitchen_pliers": 7800.0,
    "steel": 7850.0,
    "cactus": 1000.0,
    "plastic": 950.0,
    "butter": 920.0,
    "flower": 350.0,
    "sponge": 50.0,
    "pepper": 570.0,
}

object_dict = {
    # "beer_bottle": "glass",
    # "long_saussage": "meat",
    # "bread_1": "bread",
    # "bread_2": "bread",
    # "dark_brown_avocado": "fruit",
    # "green_avocado": "fruit",
    # "brown_banana": "banana",
    # "green_banana": "banana",
    # "grain_buckwheat": "grain_buckwheat",
    "green_orange": "orange",
    # "green_pineapple": "pineapple",
    # "iceberg_lettuce_1": "iceberg_lettuce",
    # "iceberg_lettuce_2": "iceberg_lettuce",
    # "kitchen_knife_1": "knife",
    # "kitchen_knife_2": "knife",
    "kiwi_1": "fruit",
    "kiwi_2": "fruit",
    # "large_bowl": "ceramic",
    # "large_plate": "ceramic",
    "lemon": "fruit",
    "lime": "fruit",
    # "medium_bowl": "ceramic",
    # "medium_plate": "ceramic",
    # "mixer": "mixer",
    # "pie_fork": "fork",
    "scissors_1": "scissors",
    # "scissors_2": "scissors",
    # "short_sausage": "meat",
    # "small_bowl": "ceramic",
    # "small_plate": "ceramic",
    # "target": "target",
    "tomato": "fruit",
    "green_tomato": "fruit",
    "yellow_tomato": "fruit",
    # "watermelon": "fruit",
    # "white_towel": "cloth",
    # "yellow_banana": "banana",
    "yellow_orange": "orange",
    # "yellow_pineapple": "pineapple",
    # "yellow_peach": "fruit",
    # "blue_vase": "ceramic",
    # "yellow_vase": "ceramic",
    # "bread_roll_1": "bread",
    # "bread_roll_2": "bread",
    # "bread_roll_3": "bread",
    # "round_red_onion": "onion",
    # "round_white_onion": "onion",
    # "table_knife": "knife",
    # "oven_mitt": "cloth",
    # "cutting_board": "wood",
    # "cucumber": "fruit",
    # "ladle": "knife",
    # "cold_cutting_1": "meat",
    # "cold_cutting_2": "meat",
    # "cold_cutting_3": "meat",
    # "cheese": "cheese",
    # "cheese_2": "cheese",
    # "cheese_3": "cheese",
    # "cherry": "fruit",
    "coke_can": "alu",
    # "fork": "fork",
    # "garlic": "garlic",
    # "honey_peach": "fruit",
    # "flowerpot": "ceramic",
    # "kitchen_pliers": "kitchen_pliers",
    # "paicha": "fork",
    # "pan": "steel",
    # "peeler": "steel",
    # "pickled_cucumber": "fruit",
    # "cactus": "cactus",
    # "brush": "plastic",
    # "butter": "butter",
    # "butter_knife": "steel",
    "gerbera": "flower",
    "green_grape": "fruit",
    # "horticultural_scissors": "steel",
    # "teaspoon": "steel",
    # "tulip": "flower",
    "yellow_apple": "fruit",
    # "green_apple": "fruit",
    "red_apple": "fruit",
    "rose": "flower",
    # "purple_grape": "fruit",
    # "sponge": "sponge",
    "pepper_black": "pepper",
    # "pepper_brown": "pepper",
    # "pepper_white": "pepper",
    # "black_towel": "cloth",
    # "bread_knife": "steel",
    # "green_banana": "banana",
}


class CreateSceneTestCase(unittest.TestCase):
    plot: bool = False
    resource_path: str

    @classmethod
    def setUpClass(cls):
        tracemalloc.start()
        cls.resource_path = os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "multiverse_assets")

    def test_create_object(self):
        for object_name in object_dict:
            folder_path = f"/home/giangnguyen/Downloads/CoohomModels/clean_models/{object_name}"
            material = object_dict[object_name]

            obj_name = os.path.basename(folder_path).split(".")[0]
            obj_mesh_dir_path = os.path.join(folder_path, "obj")
            mesh_file_paths_dict = {}
            for obj_mesh_file_name in [obj_mesh_file_name for obj_mesh_file_name in os.listdir(obj_mesh_dir_path)
                                       if obj_mesh_file_name.endswith(".obj")]:
                obj_mesh_file_path = os.path.join(obj_mesh_dir_path, obj_mesh_file_name)
                body_name = os.path.basename(obj_mesh_file_name).split(".")[0]
                mesh_file_paths_dict[body_name] = [obj_mesh_file_path]

            stl_mesh_dir_path = os.path.join(folder_path, "stl")
            for stl_mesh_file_name in [stl_mesh_file_name for stl_mesh_file_name in os.listdir(stl_mesh_dir_path)
                                       if stl_mesh_file_name.endswith(".stl")]:
                stl_mesh_file_path = os.path.join(stl_mesh_dir_path, stl_mesh_file_name)
                body_name = os.path.basename(stl_mesh_file_name).split(".")[0]
                body_name = body_name.replace("UCX_", "")
                body_name = re.sub(r'_\d{3}$', '', body_name)
                if body_name in mesh_file_paths_dict:
                    mesh_file_paths_dict[body_name].append(stl_mesh_file_path)
                else:
                    raise ValueError(f"{body_name} does not exist in obj_file_paths.")

            if len(mesh_file_paths_dict) == 1:
                tmp_mesh_file_paths_dict = {}
                for body_name, mesh_file_paths in mesh_file_paths_dict.items():
                    tmp_mesh_file_paths_dict[object_name] = mesh_file_paths
                mesh_file_paths_dict = tmp_mesh_file_paths_dict

            for body_name in mesh_file_paths_dict:
                mesh_file_paths_dict[body_name].sort()
                if not mesh_file_paths_dict[body_name][0].endswith(".obj"):
                    raise ValueError(f"{body_name} does not have an obj file.")
                if len([mesh_file_path for mesh_file_path in mesh_file_paths_dict[body_name]
                        if mesh_file_path.endswith(".obj")]) != 1:
                    raise ValueError(f"{body_name} has more than one obj file.")

            obj_target_dir_path = os.path.join(self.resource_path, "objects", obj_name)
            obj_urdf_path = os.path.join(obj_target_dir_path, f"{obj_name}.urdf")
            obj_mjcf_path = os.path.join(obj_target_dir_path, f"{obj_name}.xml")

            config = Configuration(
                model_name=obj_name,
                with_physics=True,
                with_visual=True,
                with_collision=True,
                default_rgba=numpy.array([1.0, 0.0, 0.0, 0.0]),
                inertia_source=InertiaSource.FROM_MESH
            )
            factory = Factory(file_path=obj_urdf_path,
                              config=config)
            factory._world_builder = WorldBuilder(usd_file_path=factory.tmp_usd_file_path)
            world_builder = factory.world_builder
            world_builder.add_body(body_name=obj_name)

            for body_name, mesh_file_paths in mesh_file_paths_dict.items():
                body_name = camel_case_to_snake_case(body_name)
                body_builder = world_builder.add_body(body_name=body_name,
                                                      parent_body_name=obj_name)
                body_builder.enable_rigid_body()

                for mesh_file_path in mesh_file_paths:
                    tmp_usd_mesh_file_path, tmp_origin_mesh_file_path = factory.import_mesh(
                        mesh_file_path=mesh_file_path,
                        mesh_scale=numpy.array(
                            [0.1, 0.1, 0.1]))
                    mesh_stage = Usd.Stage.Open(tmp_usd_mesh_file_path)
                    default_prim = mesh_stage.GetDefaultPrim()
                    if default_prim.IsA(UsdGeom.Mesh):
                        mesh_prim = default_prim
                    else:
                        for prim in default_prim.GetChildren():
                            if prim.IsA(UsdGeom.Mesh):
                                mesh_prim = prim
                                break
                        else:
                            raise TypeError(f"{default_prim} does not contain a mesh.")

                    geom_name = os.path.basename(mesh_file_path).split(".")[0]
                    geom_name = geom_name.replace("UCX_", "")
                    is_visible = mesh_file_path.endswith(".obj") or mesh_file_path.endswith(".dae")
                    is_collidable = mesh_file_path.endswith(".stl")
                    geom_property = GeomProperty(geom_type=GeomType.MESH,
                                                 is_visible=is_visible,
                                                 is_collidable=is_collidable,
                                                 density=density_dict[material])
                    geom_builder = body_builder.add_geom(geom_name=geom_name,
                                                         geom_property=geom_property)
                    geom_builder.build()

                    mesh_property = MeshProperty.from_mesh_file_path(mesh_file_path=tmp_usd_mesh_file_path,
                                                                     mesh_path=mesh_prim.GetPath(),
                                                                     texture_coordinate_name="UVMap")

                    geom_builder.add_mesh(mesh_name=geom_name,
                                          mesh_property=mesh_property)

                    if is_visible and mesh_prim.HasAPI(UsdShade.MaterialBindingAPI):
                        material_binding_api = UsdShade.MaterialBindingAPI(mesh_prim)
                        material_paths = material_binding_api.GetDirectBindingRel().GetTargets()
                        if len(material_paths) > 1:
                            raise NotImplementedError(f"Mesh {geom_name} has more than one material.")
                        material_path = material_paths[0]
                        material_property = MaterialProperty.from_material_file_path(
                            material_file_path=tmp_usd_mesh_file_path,
                            material_path=material_path)
                        if material_property.opacity == 0.0:
                            print(f"Opacity of {material_path} is 0.0. Set to 1.0.")
                            material_property._opacity = 1.0
                        material_builder = geom_builder.add_material(material_name=material_path.name,
                                                                     material_property=material_property)

                        body_builder.compute_and_set_inertial()

            world_builder.export()

            urdf_export = UrdfExporter(file_path=obj_urdf_path,
                                       factory=factory)
            urdf_export.build()
            urdf_export.export()

            mjcf_export = MjcfExporter(file_path=obj_mjcf_path,
                                       factory=factory)
            mjcf_export.build()
            mjcf_export.export()

            obj_mesh_file_paths = []
            dae_mesh_file_paths = []
            stl_mesh_file_paths = []
            fbx_file_path = os.path.join(obj_target_dir_path, f"{obj_name}.fbx")

            for body_name, mesh_file_paths in mesh_file_paths_dict.items():
                for mesh_file_path in mesh_file_paths:
                    if mesh_file_path.endswith(".obj"):
                        obj_mesh_file_paths.append(mesh_file_path)
                    elif mesh_file_path.endswith(".dae"):
                        dae_mesh_file_paths.append(mesh_file_path)
                    elif mesh_file_path.endswith(".stl"):
                        stl_mesh_file_paths.append(mesh_file_path)
                    else:
                        raise ValueError(f"Unknown mesh file extension: {mesh_file_path}")

            cmd = clean_up_meshes_script
            cmd += import_obj(in_objs=obj_mesh_file_paths,
                              mesh_scale=numpy.array([0.1, 0.1, 0.1]),
                              clean_up=False)
            cmd += import_dae(in_daes=dae_mesh_file_paths,
                              mesh_scale=numpy.array([0.1, 0.1, 0.1]),
                              clean_up=False)
            cmd += import_stl(in_stls=stl_mesh_file_paths,
                              mesh_scale=numpy.array([0.1, 0.1, 0.1]),
                              clean_up=False)

            cmd += export_fbx(out_fbx=fbx_file_path)

            cmd = ["blender",
                   "--background",
                   "--python-expr",
                   f"import bpy"
                   f"{cmd}"]

            process = subprocess.Popen(cmd)
            process.wait()

            usd_meshes_dir_path = os.path.join(obj_target_dir_path, obj_name, "meshes", "usd")
            remove_dir_paths = [os.path.join(usd_meshes_dir_path, "from_dae"),
                                os.path.join(usd_meshes_dir_path, "from_obj"),
                                os.path.join(usd_meshes_dir_path, "from_stl")]

            for remove_dir_path in remove_dir_paths:
                if os.path.exists(remove_dir_path):
                    shutil.rmtree(remove_dir_path)

            for mesh_file_name in os.listdir(os.path.join(obj_target_dir_path, obj_name, "meshes", "stl")):
                if mesh_file_name.startswith("UCX_"):
                    os.remove(os.path.join(obj_target_dir_path, obj_name, "meshes", "stl", mesh_file_name))

    def test_create_robots(self):
        # robots_dir = "/media/giangnguyen/Storage/mujoco_menagerie"
        # for robot_name in os.listdir(robots_dir):
        #     robot_dir_path = os.path.join(robots_dir, robot_name)
        #     if not os.path.isdir(robot_dir_path):
        #         continue
        #     for robot_file_name in os.listdir(robot_dir_path):
        #         if robot_file_name.endswith(".xml") and "scene" not in robot_file_name:
        #             robot_file_path = os.path.join(robot_dir_path, robot_file_name)

        robot_file_path = "/media/giangnguyen/Storage/Multiverse/multiverse_ws/src/isr_testbed/urdf/isr-testbed.urdf"
        factory = UrdfImporter(file_path=robot_file_path,
                               with_physics=True,
                               with_visual=True,
                               with_collision=True,
                               default_rgba=numpy.array([0.9, 0.9, 0.9, 1.0]),
                               inertia_source=InertiaSource.FROM_VISUAL_MESH)
        factory.import_model()

        output_mjcf_path = os.path.join(self.resource_path, "robots", factory.config.model_name,
                                        f"{factory.config.model_name}.xml")
        mjcf_exporter = MjcfExporter(file_path=output_mjcf_path,
                                     factory=factory)
        mjcf_exporter.build()
        mjcf_exporter.export()

        output_urdf_path = os.path.join(self.resource_path, "robots", factory.config.model_name,
                                        f"{factory.config.model_name}.urdf")
        urdf_exporter = UrdfExporter(file_path=output_urdf_path,
                                     factory=factory)
        urdf_exporter.build()
        urdf_exporter.export()
