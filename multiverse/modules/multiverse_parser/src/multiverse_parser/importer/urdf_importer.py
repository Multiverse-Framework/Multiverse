#!/usr/bin/env python3.10

import os
from math import degrees
from typing import Optional, List, Union

from pxr import Gf, Usd
from urdf_parser_py import urdf

from .importer import Configuration, Importer

from ..factory import WorldBuilder, BodyBuilder, JointBuilder, JointType, GeomBuilder, GeomType, GeomProperty
from ..utils import rpy_to_quat, xform_cache


# import rospkg
# import xml.etree.ElementTree as ET
# import numpy
# import tf
# from scipy.spatial.transform import Rotation
# from math import degrees
#
# from multiverse_parser import WorldBuilder, GeomType, JointType
# from multiverse_parser.factory.body_builder import body_dict
# from multiverse_parser.utils import import_dae, import_obj, import_stl
# from multiverse_parser.utils import export_usd
# from multiverse_parser.utils import diagonalize_inertia, clear_meshes, modify_name
# from multiverse_parser.utils import xform_cache
# from pxr import Gf


def get_joint_pos_and_quat(urdf_joint) -> (tuple, tuple):
    if hasattr(urdf_joint, "origin") and urdf_joint.origin is not None:
        joint_pos = tuple(urdf_joint.origin.xyz)
        joint_rpy = tuple(urdf_joint.origin.rpy)
    else:
        joint_pos = (0.0, 0.0, 0.0)
        joint_rpy = (0.0, 0.0, 0.0)
    return joint_pos, rpy_to_quat(joint_rpy)





class UrdfImporter(Importer):
    _world_builder: WorldBuilder
    _urdf_model: urdf.URDF

    def __init__(
            self,
            file_path: str,
            with_physics: bool,
            with_visual: bool,
            with_collision: bool,
    ) -> None:
        # self.material_dict = {}
        # self.mesh_dict = {}
        # self.geom_dict = {}
        #     self.rospack = rospkg.RosPack()
        #
        #     # for urdf_material in ET.parse(urdf_file_path).getroot().findall("material"):
        #     #     self.material_dict[urdf_material.get("name")] = tuple(map(float, urdf_material.find("color").get("rgba").split()))

        with open(file_path, "r") as file:
            urdf_string = file.read()
        self._urdf_model = urdf.URDF.from_xml_string(urdf_string)

        model_name = self._urdf_model.name
        super().__init__(file_path=file_path, configuration=Configuration(
            model_name=model_name,
            with_physics=with_physics,
            with_visual=with_visual,
            with_collision=with_collision
        ))

    def import_model(self, save_file_path: Optional[str] = None) -> str:
        self._world_builder = WorldBuilder(self.tmp_file_path)

        self._world_builder.add_body(body_name=self.config.model_name)
        if self.config.model_name != self._urdf_model.get_root():
            print(f"Root link {self._urdf_model.get_root()} is not the model name {self.config.model_name}, "
                  f"add it as a root body.")
            self._world_builder.add_body(body_name=self._urdf_model.get_root(),
                                         parent_body_name=self.config.model_name)

        self.import_body_and_joint(urdf_link_name=self._urdf_model.get_root())

        self._world_builder.export()

        return self.tmp_file_path if save_file_path is None else self.save_tmp_model(file_path=save_file_path)

    def import_body_and_joint(self, urdf_link_name) -> None:
        if urdf_link_name not in self._urdf_model.child_map:
            return

        for child_joint_name, child_urdf_link_name in self._urdf_model.child_map[urdf_link_name]:
            child_urdf_joint: urdf.Joint = self._urdf_model.joint_map[child_joint_name]

            body_builder = self.import_body(body_name=urdf_link_name,
                                            child_body_name=child_urdf_link_name,
                                            joint=child_urdf_joint)

            child_urdf_link = self._urdf_model.link_map[child_urdf_link_name]
            self.import_geoms(link=child_urdf_link, body_builder=body_builder)

            if self.config.with_physics:
                self.import_joint(joint=child_urdf_joint,
                                  parent_body_name=urdf_link_name,
                                  child_body_name=child_urdf_link_name)

                self.import_body_and_joint(urdf_link_name=child_urdf_link_name)

    def import_body(self, body_name: str, child_body_name: str, joint: urdf.Joint) -> BodyBuilder:
        joint_pos, joint_quat = get_joint_pos_and_quat(joint)

        if joint.type != "fixed" and self.config.with_physics:
            body_builder = self._world_builder.add_body(body_name=child_body_name,
                                                        parent_body_name=self._urdf_model.get_root())
            body_builder.enable_rigid_body()
        else:
            body_builder = self._world_builder.add_body(body_name=child_body_name,
                                                        parent_body_name=body_name)

        relative_to_body_builder = self._world_builder.get_body_builder(body_name=body_name)
        relative_to_xform = relative_to_body_builder.xform
        body_builder.set_transform(pos=joint_pos, quat=joint_quat, relative_to_xform=relative_to_xform)

        return body_builder

    def import_geoms(self, link: urdf.Link, body_builder: BodyBuilder) -> List[GeomBuilder]:
        geom_builders = []
        geom_name = f"{link.name}_geom"
        if self.config.with_visual:
            for i, visual in enumerate(link.visuals):
                geom_builder = self._import_geom(geom_name=f"{geom_name}_visual_{i}", geom=visual,
                                                 body_builder=body_builder)
                geom_builders.append(geom_builder)
        if self.config.with_collision:
            for i, collision in enumerate(link.collisions):
                geom_builder = self._import_geom(geom_name=f"{geom_name}_collision_{i}", geom=collision,
                                                 body_builder=body_builder)
                geom_builders.append(geom_builder)
        return geom_builders

    def _import_geom(self, geom_name: str, geom: Union[urdf.Visual, urdf.Collision],
                     body_builder: BodyBuilder) -> GeomBuilder:
        if geom.origin is not None:
            geom_pos = tuple(geom.origin.xyz)
            geom_rot = tuple(geom.origin.rpy)
        else:
            geom_pos = (0.0, 0.0, 0.0)
            geom_rot = (0.0, 0.0, 0.0)
        geom_quat = rpy_to_quat(geom_rot)

        is_visible = isinstance(geom, urdf.Visual)
        is_collidable = isinstance(geom, urdf.Collision)
        if type(geom.geometry) is urdf.Box:
            geom_builder = body_builder.add_geom(geom_name=geom_name,
                                                 geom_type=GeomType.CUBE,
                                                 geom_property=GeomProperty(is_visible=is_visible,
                                                                            is_collidable=is_collidable))
            geom_builder.build()
            geom_scale = tuple(geom.geometry.size[i] / 2.0 for i in range(3))
            geom_builder.set_transform(pos=geom_pos, quat=geom_quat, scale=geom_scale)
        elif type(geom.geometry) is urdf.Sphere:
            geom_builder = body_builder.add_geom(geom_name=geom_name, geom_type=GeomType.SPHERE,
                                                 geom_property=GeomProperty(is_visible=is_visible,
                                                                            is_collidable=is_collidable))
            geom_builder.build()
            geom_builder.set_transform(pos=geom_pos, quat=geom_quat)
            geom_builder.set_attribute(radius=geom.geometry.radius)
        elif type(geom.geometry) is urdf.Cylinder:
            geom_builder = body_builder.add_geom(geom_name=geom_name, geom_type=GeomType.CYLINDER,
                                                 geom_property=GeomProperty(is_visible=is_visible,
                                                                            is_collidable=is_collidable))
            geom_builder.build()
            geom_builder.set_transform(pos=geom_pos, quat=geom_quat)
            geom_builder.set_attribute(radius=geom.geometry.radius, height=geom.geometry.length)
        elif type(geom.geometry) is urdf.Mesh:
            geom_builder = body_builder.add_geom(geom_name=geom_name, geom_type=GeomType.MESH,
                                                 geom_property=GeomProperty(is_visible=is_visible,
                                                                            is_collidable=is_collidable))
            source_mesh_file_path = self.get_mesh_file_path(urdf_file_path=geom.geometry.filename)
            if source_mesh_file_path is not None:
                tmp_mesh_file_path = self.import_mesh(mesh_file_path=source_mesh_file_path)
                geom_builder.add_mesh(mesh_name=f"SM_{geom_name}", mesh_file_path=tmp_mesh_file_path)
                geom_builder.build()
                geom_scale = (1.0, 1.0, 1.0) if geom.geometry.scale is None else tuple(geom.geometry.scale)
                geom_builder.set_transform(pos=geom_pos, quat=geom_quat, scale=geom_scale)
        else:
            raise ValueError(f"Geom type {type(geom.geometry)} not implemented.")

        return geom_builder

    def get_mesh_file_path(self, urdf_file_path: str) -> Optional[str]:
        mesh_file_path = None
        if urdf_file_path.find("package://") != -1:
            urdf_file_path = urdf_file_path.replace("package://", "")
            package_name = urdf_file_path.split("/", 2)[0]
            try:
                import rospkg
                package_path = os.path.dirname(rospkg.RosPack().get_path(package_name))
                mesh_file_path = os.path.join(package_path, urdf_file_path)
            except (ImportError, rospkg.common.ResourceNotFound):
                print(f"Package {package_name} not found or rospkg not installed, "
                      f"searching for {urdf_file_path} in {os.getcwd()}...")
                file_paths = []
                for root, _, files in os.walk(os.getcwd()):
                    if urdf_file_path in files:
                        file_paths.append(os.path.join(root, urdf_file_path))

                if len(file_paths) == 0:
                    print(f"Mesh file {urdf_file_path} not found in {os.getcwd()}.")
                    return
                elif len(file_paths) == 1:
                    print(f"Found {file_paths[0]}")
                elif len(file_paths) > 1:
                    print(f"Found {len(file_paths)} meshes {urdf_file_path} in {os.getcwd()}, "
                          f"take the first one {file_paths[0]}.")
                    mesh_file_path = file_paths[0]

        elif urdf_file_path.find("file://") != -1:
            mesh_file_path = urdf_file_path.replace("file://", "")
            if not os.path.isabs(mesh_file_path):
                mesh_file_path = os.path.join(os.path.dirname(self.source_file_path), mesh_file_path)
                if not os.path.exists(mesh_file_path):
                    print(f"Mesh file {mesh_file_path} not found.")
                    mesh_file_path = None

        return mesh_file_path

        # mesh_path = urdf_geom.filename
        #     if mesh_path not in self.mesh_dict:
        #         from multiverse_parser.factory import TMP_USD_MESH_PATH
        #
        #         file = os.path.basename(mesh_path)
        #         mesh_name, file_extension = os.path.splitext(file)
        #
        #         if mesh_path.find("package://") != -1:
        #             mesh_path = mesh_path.replace("package://", "")
        #             package_name = mesh_path.split("/", 2)[0]
        #             try:
        #                 package_path = os.path.dirname(self.rospack.get_path(package_name))
        #                 mesh_path_abs = os.path.join(package_path, mesh_path)
        #             except rospkg.common.ResourceNotFound:
        #                 print(f"Package {package_name} not found, searching for {file} in {self.source_file_dir}...")
        #                 file_paths = []
        #                 for root, _, files in os.walk(self.source_file_dir):
        #                     if file in files:
        #                         file_paths.append(os.path.join(root, file))
        #
        #                 if len(file_paths) == 0:
        #                     print(f"Mesh file {file} not found in {self.source_file_dir}.")
        #                     return
        #                 elif len(file_paths) == 1:
        #                     print(f"Found {file_paths[0]}")
        #                 elif len(file_paths) > 1:
        #                     print(
        #                         f"Found {str(len(file_paths))} meshes {file} in {self.source_file_dir}, take the first one {file_paths[0]}.")
        #
        #                 mesh_path_abs = file_paths[0]
        #
        #         elif mesh_path.find("file://") != -1:
        #             mesh_path_abs = mesh_path.replace("file://", "")
        #             if not os.path.isabs(mesh_path_abs):
        #                 mesh_path_abs = os.path.join(os.path.dirname(self.urdf_file_path), mesh_path_abs)
        #                 if not os.path.exists(mesh_path_abs):
        #                     print(f"Mesh file {mesh_path_abs} not found.")
        #                     return
        #
        #         clear_meshes()
        #
        #         if file_extension == ".dae":
        #             import_dae(in_dae=mesh_path_abs)
        #         elif file_extension == ".obj":
        #             import_obj(in_obj=mesh_path_abs)
        #         elif file_extension == ".stl":
        #             import_stl(in_stl=mesh_path_abs)
        #         else:
        #             print(f"File extension {file_extension} not implemented")
        #             return
        #
        #         export_usd(
        #             out_usd=os.path.join(
        #                 TMP_USD_MESH_PATH,
        #                 "visual" if is_visual else "collision",
        #                 modify_name(in_name=mesh_name) + ".usda",
        #             )
        #         )
        #
        #         self.mesh_dict[urdf_geom.filename] = mesh_name
        #     else:
        #         mesh_name = self.mesh_dict[urdf_geom.filename]
        #
        #     mesh_builder = geom_builder.add_mesh(mesh_name=mesh_name)
        #     mesh_builder.save()
        #
        # if not is_visual:
        #     from multiverse_parser.factory import (
        #         COLLISION_MESH_COLOR,
        #         COLLISION_MESH_OPACITY,
        #     )
        #
        #     geom_builder.set_attribute(prefix="primvars", displayColor=COLLISION_MESH_COLOR)
        #     geom_builder.set_attribute(prefix="primvars", displayOpacity=COLLISION_MESH_OPACITY)
        #
        # geom_builder.compute_extent()

    def import_joint(self, joint: urdf.Joint, parent_body_name: str, child_body_name: str) -> JointBuilder | None:
        joint_type = JointType.from_string(joint.type)
        joint_pos, _ = get_joint_pos_and_quat(joint)
        joint_pos = Gf.Vec3d(joint_pos)

        if joint_type != JointType.FIXED and joint_type != JointType.NONE:
            parent_body_builder = self._world_builder.get_body_builder(parent_body_name)
            child_body_builder = self._world_builder.get_body_builder(child_body_name)

            parent_prim = parent_body_builder.xform.GetPrim()
            child_prim = child_body_builder.xform.GetPrim()

            body1_transform = xform_cache.GetLocalToWorldTransform(parent_prim)
            body1_rot = body1_transform.ExtractRotationQuat()

            body2_transform = xform_cache.GetLocalToWorldTransform(child_prim)
            body1_to_body2_transform = body2_transform * body1_transform.GetInverse()
            body1_to_body2_pos = body1_to_body2_transform.ExtractTranslation()

            joint_pos = body1_rot.GetInverse().Transform(joint_pos - body1_to_body2_pos)

            joint_builder = child_body_builder.add_joint(
                joint_name=joint.name,
                parent_prim=parent_prim,
                joint_type=joint_type,
                joint_pos=joint_pos,
                joint_axis=joint.axis,
            )

            if joint_type == JointType.REVOLUTE:
                joint_builder.set_limit(lower=degrees(joint.limit.lower),
                                        upper=degrees(joint.limit.upper))
            elif joint_type == JointType.PRISMATIC:
                joint_builder.set_limit(lower=joint.limit.lower,
                                        upper=joint.limit.upper)

        return None
