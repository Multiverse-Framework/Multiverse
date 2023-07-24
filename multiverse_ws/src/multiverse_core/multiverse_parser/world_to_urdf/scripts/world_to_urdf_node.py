#!/usr/bin/env python3

import os
import sys
import xml.etree.ElementTree as ET
from sdf_to_urdf import sdf_to_urdf
from urdf_parser_py import urdf

def modify_urdf(model_name, in_urdf_model):
    try:
        in_urdf_model.get_root()
    except AssertionError:
        urdf_root_link = urdf.Link()
        urdf_root_link.name = model_name
        for link_name, link in in_urdf_model.link_map.items():
            if not any(joint.child == link_name for joint in in_urdf_model.joint_map.values()):
                urdf_root_joint = urdf.Joint()
                urdf_root_joint.name = link_name + "_joint"
                urdf_root_joint.type = "fixed"
                urdf_root_joint.parent = urdf_root_link.name
                urdf_root_joint.child = link_name
                in_urdf_model.add_joint(urdf_root_joint)
        in_urdf_model.add_link(urdf_root_link)

def world_to_urdf(world_file, urdf_file):
    # Parse the XML file
    tree = ET.parse(world_file)
    root = tree.getroot()

    # Iterate over the <item> elements
    world = root.find("world")
    world_name = "world"
    urdf_model = urdf.URDF(world_name)
    urdf_model.add_link(urdf.Link(world_name))

    for include in world.findall("include"):
        
        uri = include.find("uri").text
        name = include.find("name")
        if name is not None:
            name = name.text
        pose = include.find("pose")
        sdf_path = uri[uri.find('//') + 2:]
        if pose is not None:
            pose = [float(x) for x in pose.text.strip().split()]

        for dir_path, dirs, files in os.walk(os.environ.get('MESH_WORKSPACE_PATH'), followlinks=True):
            directories = dir_path.split(os.path.sep)
            last_two_directories = os.path.join(directories[-2], directories[-1])
            if sdf_path == last_two_directories:
                sdf_config = dir_path + '/model.config'
                if not os.path.exists(sdf_config):
                    continue

                sdf_tree = ET.parse(sdf_config)
                sdf_root = sdf_tree.getroot()
                sdf_file = dir_path + '/' + sdf_root.find("sdf").text
                tmp_urdf_file =  dir_path + "/model.urdf"

                if name is None:
                    name = directories[-1]
                tmp_urdf_name = name

                sdf_to_urdf(sdf_file, tmp_urdf_file, tmp_urdf_name)
                tmp_urdf_model = urdf.URDF.from_xml_file(tmp_urdf_file)
                
                modify_urdf(tmp_urdf_name, tmp_urdf_model)

                tmp_root_joint = urdf.Joint()
                if isinstance(pose, list) and len(pose) == 6:
                    urdf_pose = urdf.Pose()
                    urdf_pose.xyz = [pose[0], pose[1], pose[2]]
                    urdf_pose.rpy = [pose[3], pose[4], pose[5]]
                    tmp_root_joint.origin = urdf_pose
                tmp_root_joint.name = tmp_urdf_model.get_root() + "_joint"
                tmp_root_joint.type = "fixed"
                tmp_root_joint.parent = world_name 
                tmp_root_joint.child = tmp_urdf_model.get_root()
                urdf_model.add_joint(tmp_root_joint)

                for joint in tmp_urdf_model.joints:
                    urdf_model.add_joint(joint)

                for link in tmp_urdf_model.links:
                    urdf_model.add_link(link)

                os.remove(tmp_urdf_file)
                
    urdf_xml_string = urdf_model.to_xml_string()
    urdf_xml_string = urdf_xml_string.replace("PATHTOMESHES/kinetic/share/", "")
    urdf_xml_string = urdf_xml_string.replace("PATHTOMESHES/melodic/share/", "")
    urdf_xml_string = urdf_xml_string.replace("PATHTOMESHES/noetic/share/", "")
    with open(urdf_file, 'w') as file:
        file.write(urdf_xml_string)

if __name__ == "__main__":
    if len(sys.argv) >= 3:
        (world_file, urdf_file) = (sys.argv[1], sys.argv[2])
    else:
        print("Usage: in_world.world out_urdf.urdf")
        sys.exit(1)

    world_to_urdf(world_file, urdf_file)
