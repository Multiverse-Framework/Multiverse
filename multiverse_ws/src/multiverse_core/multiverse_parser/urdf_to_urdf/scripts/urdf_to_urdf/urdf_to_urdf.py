#!/usr/bin/env python3.10

import os
from urdf_parser_py import urdf
import bpy
from bpy.types import (
    Armature,
    BlendData,
    Camera,
    Image,
    Light,
    Material,
    Mesh,
    Object,
)
import rospkg
import xml.etree.ElementTree as ET

material_dict = {}

rospack = rospkg.RosPack()


def clear_data(data: BlendData) -> None:
    armature: Armature
    for armature in data.armatures:
        data.armatures.remove(armature)
    mesh: Mesh
    for mesh in data.meshes:
        data.meshes.remove(mesh)
    object: Object
    for object in data.objects:
        data.objects.remove(object)
    material: Material
    for material in data.materials:
        data.materials.remove(material)
    camera: Camera
    for camera in data.cameras:
        data.cameras.remove(camera)
    light: Light
    for light in data.lights:
        data.lights.remove(light)
    image: Image
    for image in data.images:
        data.images.remove(image)

    return None


def import_dae(in_dae: str) -> None:
    clear_data(bpy.data)
    bpy.ops.wm.collada_import(filepath=in_dae)


def import_obj(in_obj: str) -> None:
    clear_data(bpy.data)
    bpy.ops.import_scene.obj(filepath=in_obj, axis_forward="Y", axis_up="Z")


def import_stl(in_stl: str) -> None:
    clear_data(bpy.data)
    bpy.ops.import_mesh.stl(filepath=in_stl)


def clean_up_mesh(mesh_name: str) -> None:
    selected_object = bpy.context.object
    camera: Camera
    for camera in bpy.data.cameras:
        bpy.data.cameras.remove(camera)
    light: Light
    for light in bpy.data.lights:
        bpy.data.lights.remove(light)
    bpy.context.view_layer.objects.active = bpy.context.selected_objects[0]
    if len(bpy.context.selected_objects) > 1:
        bpy.ops.object.join()

    selected_object = bpy.context.object
    selected_object.name = mesh_name
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)

def set_material(mesh_name: str, urdf_material: urdf.Material) -> None:
    mesh = bpy.data.objects.get(mesh_name)

    # Get the material or create a new one if the mesh has no materials
    if mesh.material_slots:
        material = mesh.material_slots[0].material
        material.name = "M_" + urdf_material.name
    else:
        material = bpy.data.materials.new(name="M_" + urdf_material.name)
        mesh.data.materials.append(material)

    material.use_nodes = True
    nodes = material.node_tree.nodes

    principled_bsdf = nodes.get('Principled BSDF')

    if principled_bsdf:
        principled_bsdf.inputs['Base Color'].default_value = material_dict.get(urdf_material.name, (0, 0, 0, 1))
    else:
        print(f"No Principled BSDF node found for mesh {mesh_name}")

def export_obj(out_obj: str, mesh_name: str) -> None:
    clean_up_mesh(mesh_name)
    os.makedirs(os.path.dirname(out_obj), exist_ok=True)
    bpy.ops.export_scene.obj(
        filepath=out_obj, use_selection=True, axis_forward="Y", axis_up="Z"
    )


def export_stl(out_stl: str, mesh_name: str) -> None:
    clean_up_mesh(mesh_name)
    selected_object = bpy.context.object
    selected_object.modifiers.new("Weld", "WELD")
    bpy.ops.object.modifier_apply(modifier="Weld")
    os.makedirs(os.path.dirname(out_stl), exist_ok=True)
    bpy.ops.export_mesh.stl(
        filepath=out_stl, use_selection=True, axis_forward="Y", axis_up="Z"
    )


def convert_geometry(geometry: urdf.Mesh, out_urdf:str, out_mesh_format: str, material: urdf.Material) -> None:
    mesh_filename = geometry.filename
    mesh_filename = mesh_filename.replace("package://", "")
    package_name = mesh_filename.split("/", 2)[0]
    package_path = os.path.dirname(rospack.get_path(package_name))
    in_mesh_path_abs = os.path.join(package_path, mesh_filename)

    tmp_out_urdf = out_urdf
    out_mesh_path = os.path.join(
        os.path.splitext(os.path.basename(out_urdf))[0],
        out_mesh_format,
        os.path.splitext(os.path.basename(in_mesh_path_abs))[0] + "." + out_mesh_format,
    )
    while tmp_out_urdf != "/":
        tmp_out_urdf = os.path.dirname(tmp_out_urdf)
        out_mesh_path = os.path.join(os.path.basename(tmp_out_urdf), out_mesh_path)

        if os.path.exists(os.path.join(tmp_out_urdf, "package.xml")):
            break
    else:
        print(f"No ROS package found in {out_urdf}.")
        return

    out_mesh_path_abs = os.path.join(os.path.dirname(tmp_out_urdf), out_mesh_path)
    file_extension = os.path.splitext(os.path.basename(in_mesh_path_abs))[1]
    if file_extension == ".dae":
        import_dae(in_mesh_path_abs)
    elif file_extension == ".obj":
        import_obj(in_mesh_path_abs)
    elif file_extension == ".stl":
        import_stl(in_mesh_path_abs)
    else:
        raise TypeError(f"File extension {file_extension} not implemented")
    
    mesh_name = os.path.splitext(os.path.basename(out_mesh_path_abs))[0]
    if material is not None and material.name is not None:
        set_material(mesh_name=mesh_name, urdf_material=material)

    if out_mesh_format == "obj":
        export_obj(out_obj=out_mesh_path_abs, mesh_name=mesh_name)
    elif out_mesh_format == "stl":
        export_stl(out_stl=out_mesh_path_abs, mesh_name=mesh_name)
    else:
        raise TypeError(f"Mesh format {out_mesh_format} not implemented")

    geometry.filename = "package://" + out_mesh_path


def urdf_to_urdf(in_urdf: str, out_urdf: str) -> None:
    for urdf_material in ET.parse(in_urdf).getroot().findall('material'):
        material_dict[urdf_material.get("name")] = tuple(map(float, urdf_material.find("color").get("rgba").split()))

    in_robot: urdf.Robot = urdf.URDF.from_xml_file(in_urdf)

    out_robot = urdf.URDF(in_robot.name)

    out_robot.add_link(in_robot.link_map[in_robot.get_root()])

    joint: urdf.Joint
    for joint in in_robot.joints:
        out_robot.add_joint(joint)
        link: urdf.Link = in_robot.link_map[joint.child]
        visual: urdf.Visual
        for visual in link.visuals:
            if type(visual.geometry) != urdf.Mesh:
                continue
            convert_geometry(geometry=visual.geometry, out_urdf=out_urdf, out_mesh_format="obj", material=visual.material)

        collision: urdf.Collision
        for collision in link.collisions:
            if type(collision.geometry) != urdf.Mesh:
                continue
            convert_geometry(geometry=collision.geometry, out_urdf=out_urdf, out_mesh_format="stl", material=visual.material)

        out_robot.add_link(link)

    xml_string = out_robot.to_xml_string()

    with open(out_urdf, "w") as file:
        file.write(xml_string)

    tree = ET.parse(out_urdf)
    for parent in tree.getroot().findall('.//'):
        for material in parent.findall('material'):
            parent.remove(material)

    tree.write(out_urdf)
