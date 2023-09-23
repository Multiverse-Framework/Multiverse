#!/usr/bin/env python3.10

import os
from urdf_parser_py import urdf
import rospkg
import xml.etree.ElementTree as ET
import numpy
import tf
from scipy.spatial.transform import Rotation
from math import degrees

from multiverse_parser import WorldBuilder, GeomType, JointType
from multiverse_parser.factory.body_builder import body_dict
from multiverse_parser.utils import import_dae, import_obj, import_stl
from multiverse_parser.utils import export_usd
from multiverse_parser.utils import diagonalize_inertia, clear_meshes, modify_name
from multiverse_parser.utils import xform_cache
from pxr import Gf


class UrdfImporter:
    def __init__(
        self,
        urdf_file_path: str,
        with_physics: bool,
        with_visual: bool,
        with_collision: bool,
    ) -> None:
        # self.material_dict = {}
        self.mesh_dict = {}
        self.geom_dict = {}
        self.rospack = rospkg.RosPack()

        # for urdf_material in ET.parse(urdf_file_path).getroot().findall("material"):
        #     self.material_dict[urdf_material.get("name")] = tuple(map(float, urdf_material.find("color").get("rgba").split()))

        self.urdf_file_path = urdf_file_path
        self.source_file_dir = os.path.dirname(os.path.dirname(self.urdf_file_path))
        self.with_physics = with_physics
        self.with_visual = with_visual
        self.with_collision = with_collision

        self.robot: urdf.URDF = urdf.URDF.from_xml_file(urdf_file_path)
        self.world_builder = WorldBuilder()

        self.root_link_name = self.robot.name
        body_builder = self.world_builder.add_body(body_name=self.root_link_name)

        if self.robot.name != self.robot.get_root():
            body_builder = self.world_builder.add_body(body_name=self.robot.get_root(), parent_body_name=self.root_link_name)

        urdf_link = self.robot.link_map[self.robot.get_root()]
        if self.with_visual:
            for urdf_visual in urdf_link.visuals:
                geom_name = self.robot.get_root() + "_visual_"
                geom_builder = self.import_geom_and_mesh(
                    body_builder=body_builder,
                    geom_name=geom_name,
                    geom_origin=urdf_visual.origin,
                    urdf_geom=urdf_visual.geometry,
                    is_visual=True,
                )

        if self.with_collision:
            for urdf_collision in urdf_link.collisions:
                geom_name = self.robot.get_root() + "_collision_"
                geom_builder = self.import_geom_and_mesh(
                    body_builder=body_builder,
                    geom_name=geom_name,
                    geom_origin=urdf_collision.origin,
                    urdf_geom=urdf_collision.geometry,
                    is_visual=False,
                )
                if self.with_physics:
                    geom_builder.enable_collision()
                    if hasattr(urdf_collision, "inertial"):
                        inertial = urdf_collision.inertial
                        mass = inertial.mass
                        com = tuple(inertial.origin.xyz)

                        origin_rot = Rotation.from_euler("zyx", urdf_collision.origin.rpy).as_matrix()
                        inertia = numpy.array(
                            [
                                [inertial.ixx, inertial.ixy, inertial.ixz],
                                [inertial.ixy, inertial.iyy, inertial.iyz],
                                [inertial.ixz, inertial.iyz, inertial.izz],
                            ]
                        )
                        inertia = inertia @ origin_rot.T
                        diagonal_inertia, principal_axes = diagonalize_inertia(inertia)

                        geom_builder.set_inertial(
                            mass=mass,
                            com=com,
                            diagonal_inertia=diagonal_inertia,
                            principal_axes=principal_axes,
                        )
                    else:
                        geom_builder.compute_inertial()

        self.import_body_and_joint(urdf_link_name=self.robot.get_root())

    def import_body_and_joint(self, urdf_link_name) -> None:
        if urdf_link_name not in self.robot.child_map:
            return

        for child_joint_name, urdf_child_link_name in self.robot.child_map[urdf_link_name]:
            child_link_name = urdf_child_link_name.replace(" ", "").replace("-", "_")
            urdf_joint = self.robot.joint_map[child_joint_name]

            if hasattr(urdf_joint, "origin") and urdf_joint.origin is not None:
                urdf_joint_pos = tuple(urdf_joint.origin.xyz)
                urdf_joint_rot = tuple(urdf_joint.origin.rpy)
            else:
                urdf_joint_pos = (0.0, 0.0, 0.0)
                urdf_joint_rot = (0.0, 0.0, 0.0)
            urdf_joint_quat = tf.transformations.quaternion_from_euler(urdf_joint_rot[0], urdf_joint_rot[1], urdf_joint_rot[2])
            urdf_joint_quat = (urdf_joint_quat[3], urdf_joint_quat[0], urdf_joint_quat[1], urdf_joint_quat[2])

            if urdf_joint.type != "fixed" and self.with_physics:
                body_builder = self.world_builder.add_body(body_name=child_link_name, parent_body_name=self.root_link_name)
                body_builder.enable_rigid_body()
            else:
                body_builder = self.world_builder.add_body(body_name=child_link_name, parent_body_name=urdf_link_name)

            body_builder.set_transform(pos=urdf_joint_pos, quat=urdf_joint_quat, relative_to=urdf_link_name)

            urdf_link = self.robot.link_map[child_link_name]
            if self.with_visual:
                for urdf_visual in urdf_link.visuals:
                    geom_name = child_link_name + "_visual_"
                    geom_builder = self.import_geom_and_mesh(
                        body_builder=body_builder,
                        geom_name=geom_name,
                        geom_origin=urdf_visual.origin,
                        urdf_geom=urdf_visual.geometry,
                        is_visual=True,
                    )

            if self.with_collision:
                for urdf_collision in urdf_link.collisions:
                    geom_name = child_link_name + "_collision_"
                    geom_builder = self.import_geom_and_mesh(
                        body_builder=body_builder,
                        geom_name=geom_name,
                        geom_origin=urdf_collision.origin,
                        urdf_geom=urdf_collision.geometry,
                        is_visual=False,
                    )
                    if self.with_physics:
                        geom_builder.enable_collision()
                        if hasattr(urdf_collision, "inertial"):
                            inertial = urdf_collision.inertial
                            mass = inertial.mass
                            com = tuple(inertial.origin.xyz)

                            origin_rot = Rotation.from_euler("zyx", urdf_collision.origin.rpy).as_matrix()
                            inertia = numpy.array(
                                [
                                    [inertial.ixx, inertial.ixy, inertial.ixz],
                                    [inertial.ixy, inertial.iyy, inertial.iyz],
                                    [inertial.ixz, inertial.iyz, inertial.izz],
                                ]
                            )
                            inertia = inertia @ origin_rot.T
                            diagonal_inertia, principal_axes = diagonalize_inertia(inertia)

                            geom_builder.set_inertial(
                                mass=mass,
                                com=com,
                                diagonal_inertia=diagonal_inertia,
                                principal_axes=principal_axes,
                            )
                        else:
                            geom_builder.compute_inertial()

            if self.with_physics:
                if urdf_joint.type == "fixed":
                    joint_type = JointType.FIXED
                elif urdf_joint.type == "revolute":
                    joint_type = JointType.REVOLUTE
                elif urdf_joint.type == "continuous":
                    joint_type = JointType.CONTINUOUS
                elif urdf_joint.type == "prismatic":
                    joint_type = JointType.PRISMATIC
                else:
                    joint_type = JointType.NONE
                    print(f"Joint {urdf_joint.name} type {urdf_joint.type} not supported.")

                if joint_type != JointType.FIXED and joint_type != JointType.NONE:
                    if urdf_joint.axis is not None:
                        joint_axis = list(urdf_joint.axis)
                    else:
                        joint_axis = [0, 0, 1]

                    if numpy.array_equal(joint_axis, [1, 0, 0]):
                        joint_axis = "X"
                    elif numpy.array_equal(joint_axis, [0, 1, 0]):
                        joint_axis = "Y"
                    elif numpy.array_equal(joint_axis, [0, 0, 1]):
                        joint_axis = "Z"
                    elif numpy.array_equal(joint_axis, [-1, 0, 0]):
                        joint_axis = "-X"
                    elif numpy.array_equal(joint_axis, [0, -1, 0]):
                        joint_axis = "-Y"
                    elif numpy.array_equal(joint_axis, [0, 0, -1]):
                        joint_axis = "-Z"
                    else:
                        joint_axis = None
                        print(f"Joint {urdf_joint.name} axis {str(joint_axis)} not supported.")

                    if joint_axis is not None:
                        parent_prim = body_dict[urdf_link_name].xform.GetPrim()
                        child_prim = body_dict[child_link_name].xform.GetPrim()

                        body1_transform = xform_cache.GetLocalToWorldTransform(parent_prim)
                        body1_rot = body1_transform.ExtractRotationQuat()

                        body2_transform = xform_cache.GetLocalToWorldTransform(child_prim)
                        body1_to_body2_transform = body2_transform * body1_transform.GetInverse()
                        body1_to_body2_pos = body1_to_body2_transform.ExtractTranslation()

                        joint_pos = body1_rot.GetInverse().Transform(Gf.Vec3d(urdf_joint_pos) - body1_to_body2_pos)

                        joint_builder = body_builder.add_joint(
                            joint_name=urdf_joint.name,
                            parent_name=urdf_link_name,
                            joint_type=joint_type,
                            joint_pos=joint_pos,
                            joint_axis=joint_axis,
                        )

                    if joint_type == JointType.REVOLUTE or joint_type == JointType.PRISMATIC:
                        if joint_type == JointType.REVOLUTE:
                            joint_builder.set_limit(lower=degrees(urdf_joint.limit.lower), upper=degrees(urdf_joint.limit.upper))
                        else:
                            joint_builder.set_limit(lower=urdf_joint.limit.lower, upper=urdf_joint.limit.upper)

            self.import_body_and_joint(urdf_link_name=urdf_child_link_name)

    def import_geom_and_mesh(
        self,
        body_builder,
        geom_name: str,
        geom_origin: urdf.Pose,
        urdf_geom,
        is_visual: bool,
    ) -> None:
        if geom_name in self.geom_dict:
            self.geom_dict[geom_name] += 1
        else:
            self.geom_dict[geom_name] = 0
        geom_name += str(self.geom_dict[geom_name])

        if geom_origin is not None:
            geom_pos = tuple(geom_origin.xyz)
            geom_rot = tuple(geom_origin.rpy)
        else:
            geom_pos = (0.0, 0.0, 0.0)
            geom_rot = (0.0, 0.0, 0.0)
        geom_quat = tf.transformations.quaternion_from_euler(geom_rot[0], geom_rot[1], geom_rot[2])
        geom_quat = (geom_quat[3], geom_quat[0], geom_quat[1], geom_quat[2])

        if type(urdf_geom) == urdf.Box:
            geom_builder = body_builder.add_geom(geom_name=geom_name, geom_type=GeomType.CUBE, is_visual=is_visual)
            geom_builder.set_transform(
                pos=geom_pos,
                quat=geom_quat,
                scale=(
                    urdf_geom.size[0] / 2,
                    urdf_geom.size[1] / 2,
                    urdf_geom.size[2] / 2,
                ),
            )
        elif type(urdf_geom) == urdf.Sphere:
            geom_builder = body_builder.add_geom(geom_name=geom_name, geom_type=GeomType.SPHERE, is_visual=is_visual)
            geom_builder.set_transform(pos=geom_pos, quat=geom_quat)
            geom_builder.set_attribute(radius=urdf_geom.radius)
        elif type(urdf_geom) == urdf.Cylinder:
            geom_builder = body_builder.add_geom(geom_name=geom_name, geom_type=GeomType.CYLINDER, is_visual=is_visual)
            geom_builder.set_transform(pos=geom_pos, quat=geom_quat)
            geom_builder.set_attribute(radius=urdf_geom.radius, height=urdf_geom.length)
        elif type(urdf_geom) == urdf.Mesh:
            geom_builder = body_builder.add_geom(geom_name=geom_name, geom_type=GeomType.MESH, is_visual=is_visual)
            geom_scale = (1.0, 1.0, 1.0) if urdf_geom.scale is None else tuple(urdf_geom.scale)
            geom_builder.set_transform(pos=geom_pos, quat=geom_quat, scale=geom_scale)
            mesh_path = urdf_geom.filename
            if mesh_path not in self.mesh_dict:
                from multiverse_parser.factory import TMP_USD_MESH_PATH

                file = os.path.basename(mesh_path)
                mesh_name, file_extension = os.path.splitext(file)

                if mesh_path.find("package://") != -1:
                    mesh_path = mesh_path.replace("package://", "")
                    package_name = mesh_path.split("/", 2)[0]
                    try:
                        package_path = os.path.dirname(self.rospack.get_path(package_name))
                        mesh_path_abs = os.path.join(package_path, mesh_path)
                    except rospkg.common.ResourceNotFound:
                        print(f"Package {package_name} not found, searching for {file} in {self.source_file_dir}...")
                        file_paths = []
                        for root, _, files in os.walk(self.source_file_dir):
                            if file in files:
                                file_paths.append(os.path.join(root, file))

                        if len(file_paths) == 0:
                            print(f"Mesh file {file} not found in {self.source_file_dir}.")
                            return
                        elif len(file_paths) == 1:
                            print(f"Found {file_paths[0]}")
                        elif len(file_paths) > 1:
                            print(f"Found {str(len(file_paths))} meshes {file} in {self.source_file_dir}, take the first one {file_paths[0]}.")

                        mesh_path_abs = file_paths[0]

                elif mesh_path.find("file://") != -1:
                    mesh_path_abs = mesh_path.replace("file://", "")
                    if not os.path.isabs(mesh_path_abs):
                        mesh_path_abs = os.path.join(self.source_file_dir, mesh_path_abs)
                        if os.path.exists(mesh_path_abs):
                            print(f"Mesh file {mesh_path_abs} not found.")
                            return

                clear_meshes()

                if file_extension == ".dae":
                    import_dae(in_dae=mesh_path_abs)
                elif file_extension == ".obj":
                    import_obj(in_obj=mesh_path_abs)
                elif file_extension == ".stl":
                    import_stl(in_stl=mesh_path_abs)
                else:
                    print(f"File extension {file_extension} not implemented")
                    return

                export_usd(
                    out_usd=os.path.join(
                        TMP_USD_MESH_PATH,
                        "visual" if is_visual else "collision",
                        modify_name(in_name=mesh_name) + ".usda",
                    )
                )

                self.mesh_dict[urdf_geom.filename] = mesh_name
            else:
                mesh_name = self.mesh_dict[urdf_geom.filename]

            mesh_builder = geom_builder.add_mesh(mesh_name=mesh_name)
            mesh_builder.save()

        if not is_visual:
            from multiverse_parser.factory import (
                COLLISION_MESH_COLOR,
                COLLISION_MESH_OPACITY,
            )

            geom_builder.set_attribute(prefix="primvars", displayColor=COLLISION_MESH_COLOR)
            geom_builder.set_attribute(prefix="primvars", displayOpacity=COLLISION_MESH_OPACITY)

        geom_builder.compute_extent()

        return geom_builder
