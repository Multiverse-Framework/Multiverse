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

geom_dict = {}

rospack = rospkg.RosPack()


def build_geom(
    source_file_dir: str,
    body_builder,
    geom_name: str,
    geometry,
    origin: urdf.Pose,
    visual: bool,
):
    if geom_name in geom_dict:
        geom_dict[geom_name] += 1
    else:
        geom_dict[geom_name] = 0
    geom_name += str(geom_dict[geom_name])

    if origin is not None:
        geom_pos = tuple(origin.xyz)
        geom_rot = tuple(origin.rpy)
    else:
        geom_pos = (0.0, 0.0, 0.0)
        geom_rot = (0.0, 0.0, 0.0)
    geom_quat = tf.transformations.quaternion_from_euler(geom_rot[0], geom_rot[1], geom_rot[2])
    geom_quat = (geom_quat[3], geom_quat[0], geom_quat[1], geom_quat[2])

    if type(geometry) == urdf.Box:
        geom_builder = body_builder.add_geom(geom_name=geom_name, geom_type=GeomType.CUBE)
        geom_builder.set_transform(
            pos=geom_pos,
            quat=geom_quat,
            scale=(geometry.size[0] / 2, geometry.size[1] / 2, geometry.size[2] / 2),
        )
    elif type(geometry) == urdf.Sphere:
        geom_builder = body_builder.add_geom(geom_name=geom_name, geom_type=GeomType.SPHERE)
        geom_builder.set_transform(pos=geom_pos, quat=geom_quat)
        geom_builder.set_attribute(radius=geometry.radius)
    elif type(geometry) == urdf.Cylinder:
        geom_builder = body_builder.add_geom(geom_name=geom_name, geom_type=GeomType.CYLINDER)
        geom_builder.set_transform(pos=geom_pos, quat=geom_quat)
        geom_builder.set_attribute(radius=geometry.radius, height=geometry.length)
    elif type(geometry) == urdf.Mesh:
        geom_builder = body_builder.add_geom(geom_name=geom_name, geom_type=GeomType.MESH)
        geom_scale = (1.0, 1.0, 1.0) if geometry.scale is None else tuple(geometry.scale)
        geom_builder.set_transform(pos=geom_pos, quat=geom_quat, scale=geom_scale)
        mesh_path = geometry.filename
        if mesh_path not in mesh_dict:
            from multiverse_parser.factory import TMP_USD_MESH_PATH, clear_data

            file = os.path.basename(mesh_path)
            mesh_name, file_extension = os.path.splitext(file)

            if mesh_path.find("package://") != -1:
                mesh_path = mesh_path.replace("package://", "")
                package_name = mesh_path.split("/", 2)[0]
                try:
                    package_path = os.path.dirname(rospack.get_path(package_name))
                    mesh_path_abs = os.path.join(package_path, mesh_path)
                except rospkg.common.ResourceNotFound:
                    print(f"Package {package_name} not found, searching for {file} in {source_file_dir}...")
                    file_paths = []
                    for root, _, files in os.walk(source_file_dir):
                        if file in files:
                            file_paths.append(os.path.join(root, file))

                    if len(file_paths) == 0:
                        print(f"Mesh file {file} not found in {source_file_dir}.")
                        return
                    elif len(file_paths) == 1:
                        print(f"Found {file_paths[0]}")
                    elif len(file_paths) > 1:
                        print(f"Found {str(len(file_paths))} meshes {file} in {source_file_dir}, take the first one {file_paths[0]}.")

                    mesh_path_abs = file_paths[0]

            elif mesh_path.find("file://") != -1:
                mesh_path_abs = mesh_path.replace("file://", "")
                if not os.path.isabs(mesh_path_abs):
                    mesh_path_abs = os.path.join(source_file_dir, mesh_path_abs)
                    if os.path.exists(mesh_path_abs):
                        print(f"Mesh file {mesh_path_abs} not found.")
                        return

            clear_data()
            if file_extension == ".dae":
                import_dae(mesh_path_abs)
            elif file_extension == ".obj":
                import_obj(mesh_path_abs)
            elif file_extension == ".stl":
                import_stl(mesh_path_abs)
            else:
                print(f"File extension {file_extension} not implemented")
                return

            export_usd(
                out_usd=os.path.join(
                    TMP_USD_MESH_PATH,
                    "visual" if visual else "collision",
                    mesh_name + ".usda",
                )
            )
            mesh_dict[geometry.filename] = mesh_name
        else:
            mesh_name = mesh_dict[geometry.filename]

        mesh_builder = geom_builder.add_mesh(mesh_name=mesh_name, visual=visual)
        mesh_builder.save()

    if not visual:
        geom_builder.set_attribute(prefix="primvars", displayColor=[(1, 0, 0)])
        geom_builder.set_attribute(prefix="primvars", displayOpacity=[0.5])

    geom_builder.compute_extent()


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
        joint_quat = tf.transformations.quaternion_from_euler(joint_rot[0], joint_rot[1], joint_rot[2])
        joint_quat = (joint_quat[3], joint_quat[0], joint_quat[1], joint_quat[2])

        if urdf_joint.type != JointType.FIXED and with_physics:
            body_builder = world_builder.add_body(body_name=child_link_name, parent_body_name=root_link_name)
        else:
            body_builder = world_builder.add_body(body_name=child_link_name, parent_body_name=parent_link_name)

        body_builder.set_transform(pos=joint_pos, quat=joint_quat, relative_to=parent_link_name)

        urdf_link = robot.link_map[child_link_name]
        for visual in urdf_link.visuals:
            geom_name = child_link_name + "_visual_"
            build_geom(
                os.path.dirname(os.path.dirname(urdf_file_path)),
                body_builder,
                geom_name,
                visual.geometry,
                visual.origin,
                True,
            )

        for collision in urdf_link.collisions:
            geom_name = child_link_name + "_collision_"
            build_geom(
                os.path.dirname(os.path.dirname(urdf_file_path)),
                body_builder,
                geom_name,
                collision.geometry,
                collision.origin,
                False,
            )

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
