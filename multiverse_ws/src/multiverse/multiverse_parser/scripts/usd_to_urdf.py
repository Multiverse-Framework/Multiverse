#!/usr/bin/env python3

import sys
import os
from pxr import Usd, UsdGeom, Sdf, Gf, UsdPhysics
from urdf_parser_py.urdf import URDF, Link, Joint, Pose, JointLimit, Inertial, Inertia, Visual, Collision, Box, Cylinder, Sphere, Mesh
import tf
from math import radians
import stl.mesh
import numpy
import rospkg

def usd_quat_to_urdf_rpy(usd_quat):
    w = usd_quat.GetReal()
    [x, y, z] = usd_quat.GetImaginary()
    return tf.transformations.euler_from_quaternion([x, y, z, w])


def usd_to_urdf_handle(usd_file: str, urdf_file: str):
    rospack = rospkg.RosPack()

    urdf_pkg_path = os.path.dirname(os.path.dirname(urdf_file))

    while True:
        urdf_pkg = os.path.basename(urdf_pkg_path)
        if urdf_pkg == '/':
            break
        if urdf_pkg != 'urdf':
            try:
                rospack.get_path(urdf_pkg)
            except rospkg.common.ResourceNotFound:
                urdf_pkg_path = os.path.dirname(urdf_pkg_path)
                continue
        break

    urdf_dir = os.path.dirname(urdf_file)
    robot_name = os.path.basename(urdf_file.replace('.urdf', ''))
    stl_mesh_dir = os.path.join(urdf_dir, robot_name, 'stl')
    os.makedirs(stl_mesh_dir, exist_ok=True)

    stl_mesh_dir_rel = os.path.relpath(stl_mesh_dir, urdf_pkg_path)
    stl_mesh_dir_rel = 'package://' + urdf_pkg + '/' + stl_mesh_dir_rel

    stage = Usd.Stage.Open(usd_file)
    
    robot = URDF(robot_name)

    prim_transform = {}
    urdf_link_transform = {}

    xform_cache = UsdGeom.XformCache()

    for prim in stage.Traverse():
        if UsdPhysics.RevoluteJoint(prim) or UsdPhysics.PrismaticJoint(prim):
            urdf_joint = Joint(name=prim.GetName())

            limit = JointLimit()
            limit.effort = 1000
            limit.velocity = 1000

            if UsdPhysics.RevoluteJoint(prim):
                usd_joint = UsdPhysics.RevoluteJoint(prim)
                urdf_joint.joint_type = 'revolute'
                limit.lower = radians(
                    usd_joint.GetLowerLimitAttr().Get())
                limit.upper = radians(
                    usd_joint.GetUpperLimitAttr().Get())
            elif UsdPhysics.PrismaticJoint(prim):
                usd_joint = UsdPhysics.PrismaticJoint(prim)
                urdf_joint.joint_type = 'prismatic'
                limit.lower = usd_joint.GetLowerLimitAttr().Get()
                limit.upper = usd_joint.GetUpperLimitAttr().Get()

            urdf_joint.limit = limit

            parent_prim = stage.GetPrimAtPath(
                usd_joint.GetBody0Rel().GetTargets()[0])
            urdf_joint.parent = parent_prim.GetName()

            child_prim = stage.GetPrimAtPath(
                usd_joint.GetBody1Rel().GetTargets()[0])
            urdf_joint.child = child_prim.GetName()

            mat = Gf.Matrix4d()
            mat.SetTranslateOnly(Gf.Vec3d(-usd_joint.GetLocalPos1Attr().Get()))
            mat.SetRotateOnly(usd_joint.GetLocalRot1Attr().Get().GetInverse())
            prim_transform[urdf_joint.child] = mat

            joint_pos = xform_cache.GetLocalToWorldTransform(child_prim).ExtractTranslation()
            joint_rot = xform_cache.GetLocalToWorldTransform(parent_prim).ExtractRotation().GetQuat() * usd_joint.GetLocalRot0Attr().Get()

            mat = Gf.Matrix4d()
            mat.SetTranslateOnly(joint_pos)
            mat.SetRotateOnly(joint_rot)
            urdf_link_transform[urdf_joint.child] = mat

            axis = usd_joint.GetAxisAttr().Get()
            if axis == 'X':
                urdf_joint.axis = [1, 0, 0]
            if axis == 'Y':
                urdf_joint.axis = [0, 1, 0]
            if axis == 'Z':
                urdf_joint.axis = [0, 0, 1]

            robot.add_joint(urdf_joint)

    for prim in stage.Traverse():
        if UsdGeom.Xform(prim):
            urdf_link = Link(name=prim.GetName())

            if prim_transform.get(prim.GetName()) is None and prim.GetName() != 'world':
                xformable = UsdGeom.Xformable(prim)

                urdf_link_transform[prim.GetName()] = xform_cache.GetLocalToWorldTransform(prim)
                
                if urdf_link_transform.get(prim.GetParent().GetName()) is not None:
                    transform = (urdf_link_transform[prim.GetParent().GetName()].GetInverse().GetTranspose() * urdf_link_transform[prim.GetName()].GetTranspose()).GetTranspose()
                else:
                    transform = xformable.GetLocalTransformation()
                    
                origin = Pose()
                origin.xyz = transform.ExtractTranslation()
                origin.rpy = usd_quat_to_urdf_rpy(
                    transform.ExtractRotation().GetQuat())

                urdf_joint = Joint(name=prim.GetName() + '_joint')
                urdf_joint.joint_type = 'fixed'
                urdf_joint.origin = origin
                urdf_joint.parent = prim.GetParent().GetName()
                urdf_joint.child = prim.GetName()

                robot.add_joint(urdf_joint)

            if prim.HasAPI(UsdPhysics.MassAPI):
                physics_mass_api = UsdPhysics.MassAPI(prim)
                physics_mass_api.Apply(prim)

                inertial = Inertial()
                inertial.mass = physics_mass_api.GetMassAttr().Get()

                origin = Pose()
                origin.xyz = physics_mass_api.GetCenterOfMassAttr().Get()
                inertial.origin = origin

                inertia = Inertia()
                usd_inertia = physics_mass_api.GetDiagonalInertiaAttr().Get()
                inertia.ixx = usd_inertia[0]
                inertia.iyy = usd_inertia[1]
                inertia.izz = usd_inertia[2]
                inertial.inertia = inertia

                urdf_link.inertial = inertial

            for child_prim in prim.GetChildren():
                if UsdGeom.Gprim(child_prim):
                    geom_prim = UsdGeom.Gprim(child_prim)
                    transform = geom_prim.GetLocalTransformation()
                    scale = numpy.zeros((3), dtype=numpy.float64)
                    for i in range(3):
                        scale[i] = transform.GetRow(i).GetLength()
                        transform.SetRow(i, transform.GetRow(i).GetNormalized())
                    if prim_transform.get(prim.GetName()) is not None:
                        transform *= prim_transform[prim.GetName()]

                    origin = Pose()
                    origin.xyz = transform.ExtractTranslation()
                    origin.rpy = usd_quat_to_urdf_rpy(
                        transform.ExtractRotationQuat())
                    if child_prim.HasAPI(UsdPhysics.CollisionAPI):
                        collision = Collision()
                        collision.origin = origin

                        if UsdGeom.Cube(child_prim):
                            box = Box()
                            box.size = scale * 2

                            collision.geometry = box

                        elif UsdGeom.Sphere(child_prim):
                            sphere = Sphere()
                            sphere.radius = UsdGeom.Sphere(child_prim).GetRadiusAttr().Get()

                            collision.geometry = sphere

                        elif UsdGeom.Cylinder(child_prim):
                            cylinder = Cylinder()
                            cylinder.radius = UsdGeom.Cylinder(child_prim).GetRadiusAttr().Get()
                            cylinder.length = UsdGeom.Cylinder(child_prim).GetHeightAttr().Get()

                            collision.geometry = cylinder

                        elif UsdGeom.Mesh(child_prim):
                            usd_mesh = UsdGeom.Mesh(child_prim)
                            points = numpy.array(usd_mesh.GetPointsAttr().Get())
                            normals = numpy.array(usd_mesh.GetNormalsAttr().Get())
                            face_vertex_indices = usd_mesh.GetFaceVertexIndicesAttr().Get()
                            faces = []
                            index = 0
                            for face_vertex_counts in usd_mesh.GetFaceVertexCountsAttr().Get():
                                face = []
                                for _ in range(face_vertex_counts):   
                                    face.append(face_vertex_indices[index])
                                    index += 1
                                
                                faces.append(face)
                            
                            faces = numpy.array(faces, dtype=numpy.int32)
                            stl_mesh = stl.mesh.Mesh(numpy.zeros(faces.shape[0], dtype=stl.mesh.Mesh.dtype))
                            stl_mesh.vectors = points[faces]
                            stl_mesh.normals = normals
                            stl_mesh_name = child_prim.GetName() + '.stl'
                            stl_mesh.save(os.path.join(stl_mesh_dir, stl_mesh_name))

                            urdf_mesh = Mesh()
                            urdf_mesh.filename = os.path.join(stl_mesh_dir_rel, stl_mesh_name)
                            urdf_mesh.scale = scale

                            collision.geometry = urdf_mesh

                        urdf_link.collision = collision

            robot.add_link(urdf_link)

    for prim in stage.Traverse():
        if UsdPhysics.RevoluteJoint(prim) or UsdPhysics.PrismaticJoint(prim):
            urdf_joint = robot.joint_map[prim.GetName()]
            usd_joint = UsdPhysics.RevoluteJoint(prim)
            
            parent_prim = stage.GetPrimAtPath(
                usd_joint.GetBody0Rel().GetTargets()[0])
            child_prim = stage.GetPrimAtPath(
                usd_joint.GetBody1Rel().GetTargets()[0])
            
            origin = Pose()
            transform = (urdf_link_transform[parent_prim.GetName()].GetInverse().GetTranspose() * urdf_link_transform[child_prim.GetName()].GetTranspose()).GetTranspose()
            origin.xyz = transform.ExtractTranslation()
            origin.rpy = usd_quat_to_urdf_rpy(
                transform.ExtractRotation().GetQuat())
            urdf_joint.origin = origin

    with open(urdf_file, "w") as file:
        file.write(robot.to_xml_string())


if __name__ == '__main__':
    if len(sys.argv) >= 3:
        (usd_file, urdf_file) = (sys.argv[1], sys.argv[2])
    else:
        print('Usage: in_usd.usda out_urdf.urdf')
        sys.exit(1)

    usd_to_urdf_handle(usd_file, urdf_file)
