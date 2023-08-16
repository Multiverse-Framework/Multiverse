#!/usr/bin/env python3.10

from multiverse_parser import WorldBuilder, GeomType, JointType
import os
from urdf_parser_py import urdf
import rospkg
import xml.etree.ElementTree as ET
import numpy
import tf

from multiverse_parser.importer.mesh_importer import import_dae, import_obj, import_stl
from multiverse_parser.exporter.mesh_exporter import export_usd

material_dict = {}

mesh_dict = {}

rospack = rospkg.RosPack()


def build_geom(body_builder, geom_name: str, geometry: urdf.Mesh, collision: bool):
    if type(geometry) == urdf.Mesh:
        geom_builder = body_builder.add_geom(geom_name=geom_name, geom_type=GeomType.MESH)
        mesh_path = geometry.filename
        if mesh_path not in mesh_dict:
            from multiverse_parser.factory import TMP_DIR

            mesh_path = mesh_path.replace("package://", "")
            package_name = mesh_path.split("/", 2)[0]
            package_path = os.path.dirname(rospack.get_path(package_name))
            mesh_path_abs = os.path.join(package_path, mesh_path)

            file_name = os.path.splitext(os.path.basename(mesh_path_abs))[0]
            file_extension = os.path.splitext(os.path.basename(mesh_path_abs))[1]
            if file_extension == ".dae":
                import_dae(mesh_path_abs)
            elif file_extension == ".obj":
                import_obj(mesh_path_abs)
            elif file_extension == ".stl":
                import_stl(mesh_path_abs)
            else:
                print(f"File extension {file_extension} not implemented")
                return

            usd_file_path = os.path.join(body_builder.usd_file_dir, TMP_DIR, file_name + ".usda")
            export_usd(out_usd=usd_file_path)
            mesh_name = "mesh_" + str(len(mesh_dict) + 1)
            mesh_dict[mesh_path] = (mesh_name, usd_file_path)
        else:
            mesh_name, usd_file_path = mesh_dict[mesh_path]

        mesh_builder = geom_builder.add_mesh(mesh_name=mesh_name, usd_file_path=usd_file_path, collision=collision)
        mesh_builder.save()


def import_from_urdf(urdf_file_path: str, with_physics: bool = True) -> WorldBuilder:
    for urdf_material in ET.parse(urdf_file_path).getroot().findall("material"):
        material_dict[urdf_material.get("name")] = tuple(map(float, urdf_material.find("color").get("rgba").split()))

    robot: urdf.Robot = urdf.URDF.from_xml_file(urdf_file_path)

    world_builder = WorldBuilder()

    root_link_name = robot.get_root().replace(" ", "").replace("-", "_")

    world_builder.add_body(body_name=root_link_name)
    urdf_joint: urdf.Joint
    for urdf_joint in robot.joints:
        parent_link_name = urdf_joint.parent.replace(" ", "").replace("-", "_")
        child_link_name = urdf_joint.child.replace(" ", "").replace("-", "_")

        if hasattr(urdf_joint, "origin") and urdf_joint.origin is not None:
            joint_pos = tuple(urdf_joint.origin.xyz)
            joint_rot = tuple(urdf_joint.origin.rpy)
        else:
            joint_pos = (0.0, 0.0, 0.0)
            joint_rot = (0.0, 0.0, 0.0)
        joint_quat = tuple(tf.transformations.quaternion_from_euler(joint_rot[0], joint_rot[1], joint_rot[2]))

        if urdf_joint.type != JointType.FIXED and with_physics:
            body_builder = world_builder.add_body(body_name=child_link_name, parent_body_name=root_link_name)
        else:
            body_builder = world_builder.add_body(body_name=child_link_name, parent_body_name=parent_link_name)

        body_builder.set_transform(pos=joint_pos, quat=joint_quat, relative_to=parent_link_name)

        urdf_link = robot.link_map[child_link_name]
        for visual in urdf_link.visuals:
            geom_name = child_link_name + "_visual"
            build_geom(body_builder, geom_name, visual.geometry, False)

        for collision in urdf_link.collisions:
            geom_name = child_link_name + "_visual"
            build_geom(body_builder, geom_name, collision.geometry, True)

        if with_physics:
            body_builder.enable_collision()

            if urdf_joint.type == "fixed":
                continue

            joint_name = urdf_joint.name.replace(" ", "").replace("-", "_")
            if urdf_joint.type == "revolute":
                joint_type = JointType.REVOLUTE
            elif urdf_joint.type == "continuous":
                joint_type = JointType.CONTINUOUS
            elif urdf_joint.type == "prismatic":
                joint_type = JointType.PRISMATIC
            else:
                print(f"Joint {joint_name} type {urdf_joint.type} not supported.")
                continue

            if hasattr(urdf_joint, "axis") and urdf_joint.axis is not None:
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
                print(f"Joint {joint_name} axis {str(joint_axis)} not supported.")
                return

            joint_builder = body_builder.add_joint(
                joint_name=joint_name,
                parent_name=parent_link_name,
                child_name=child_link_name,
                joint_type=joint_type,
                joint_pos=joint_pos,
                joint_axis=joint_axis,
            )

    return world_builder
