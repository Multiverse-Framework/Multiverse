#!/usr/bin/env python3

import os
from pxr import Usd, UsdGeom, Sdf, Gf, UsdPhysics
from urdf_parser_py.urdf import URDF, Link, Joint, Pose, JointLimit, Inertial, Inertia, Visual, Collision, Box, Cylinder, Sphere, Mesh
import tf
from math import radians
import stl.mesh
import numpy
import rospkg


def multiply_transform(transform_A: Gf.Matrix4d, transform_B: Gf.Matrix4d):
    return (transform_A.GetTranspose() * transform_B.GetTranspose()).GetTranspose()


def usd_quat_to_urdf_rpy(usd_quat: Gf.Quatf):
    w = usd_quat.GetReal()
    [x, y, z] = usd_quat.GetImaginary()
    return tf.transformations.euler_from_quaternion([x, y, z, w])


def usd_to_urdf_handle(usd_file: str, urdf_file: str):
    rospack = rospkg.RosPack()

    urdf_pkg_path = os.path.dirname(os.path.dirname(urdf_file))

    while True:
        urdf_pkg = os.path.basename(urdf_pkg_path)
        if urdf_pkg == "/":
            break
        if urdf_pkg != "urdf":
            try:
                rospack.get_path(urdf_pkg)
            except rospkg.common.ResourceNotFound:
                urdf_pkg_path = os.path.dirname(urdf_pkg_path)
                continue
        break

    urdf_dir = os.path.dirname(urdf_file)
    robot_name = os.path.basename(urdf_file.replace(".urdf", ""))
    stl_mesh_dir = os.path.join(urdf_dir, robot_name, "stl")
    os.makedirs(stl_mesh_dir, exist_ok=True)

    stl_mesh_dir_rel = os.path.relpath(stl_mesh_dir, urdf_pkg_path)
    stl_mesh_dir_rel = "package://" + urdf_pkg + "/" + stl_mesh_dir_rel

    stage = Usd.Stage.Open(usd_file)

    urdf_robot = URDF(robot_name)

    prim_transform = {}
    urdf_link_transform = {}

    xform_cache = UsdGeom.XformCache()

    for prim in stage.Traverse():
        if UsdPhysics.RevoluteJoint(prim) or UsdPhysics.PrismaticJoint(prim):
            urdf_joint = Joint(name=prim.GetName())

            urdf_limit = JointLimit()
            urdf_limit.effort = 1000
            urdf_limit.velocity = 1000

            if UsdPhysics.RevoluteJoint(prim):
                usd_joint = UsdPhysics.RevoluteJoint(prim)
                urdf_joint.joint_type = "revolute"
                urdf_limit.lower = radians(usd_joint.GetLowerLimitAttr().Get())
                urdf_limit.upper = radians(usd_joint.GetUpperLimitAttr().Get())
            elif UsdPhysics.PrismaticJoint(prim):
                usd_joint = UsdPhysics.PrismaticJoint(prim)
                urdf_joint.joint_type = "prismatic"
                urdf_limit.lower = usd_joint.GetLowerLimitAttr().Get()
                urdf_limit.upper = usd_joint.GetUpperLimitAttr().Get()
            else:
                print(f"Joint {str(prim)} not supported")
                continue

            urdf_joint.limit = urdf_limit

            parent_prim = stage.GetPrimAtPath(usd_joint.GetBody0Rel().GetTargets()[0])
            urdf_joint.parent = parent_prim.GetName()

            child_prim = stage.GetPrimAtPath(usd_joint.GetBody1Rel().GetTargets()[0])
            urdf_joint.child = child_prim.GetName()

            prim_rot = Gf.Rotation(usd_joint.GetLocalRot1Attr().Get().GetInverse())
            prim_pos = Gf.Vec3d(-usd_joint.GetLocalPos1Attr().Get())
            prim_transform[urdf_joint.child] = Gf.Matrix4d(prim_rot, prim_pos)

            urdf_link_pos = xform_cache.GetLocalToWorldTransform(child_prim).ExtractTranslation()
            urdf_link_rot = Gf.Rotation(
                xform_cache.GetLocalToWorldTransform(parent_prim).ExtractRotation().GetQuat() * usd_joint.GetLocalRot0Attr().Get()
            )

            urdf_link_transform[urdf_joint.child] = Gf.Matrix4d(urdf_link_rot, urdf_link_pos)

            usd_joint_axis = usd_joint.GetAxisAttr().Get()
            if usd_joint_axis == "X":
                urdf_joint.axis = [1, 0, 0]
            elif usd_joint_axis == "Y":
                urdf_joint.axis = [0, 1, 0]
            elif usd_joint_axis == "Z":
                urdf_joint.axis = [0, 0, 1]

            urdf_robot.add_joint(urdf_joint)

    for prim in stage.Traverse():
        if UsdGeom.Xform(prim):
            urdf_link = Link(name=prim.GetName())

            if prim_transform.get(prim.GetName()) is None and prim.GetName() != "world":
                xformable = UsdGeom.Xformable(prim)

                urdf_link_transform[prim.GetName()] = xform_cache.GetLocalToWorldTransform(prim)

                if urdf_link_transform.get(prim.GetParent().GetName()) is not None:
                    urdf_joint_transform = multiply_transform(
                        urdf_link_transform[prim.GetParent().GetName()].GetInverse(), urdf_link_transform[prim.GetName()]
                    )
                else:
                    urdf_joint_transform = xformable.GetLocalTransformation()

                urdf_origin = Pose()
                urdf_origin.xyz = urdf_joint_transform.ExtractTranslation()
                urdf_origin.rpy = usd_quat_to_urdf_rpy(urdf_joint_transform.ExtractRotation().GetQuat())

                urdf_joint = Joint(name=prim.GetName() + "_joint")
                urdf_joint.joint_type = "fixed"
                urdf_joint.origin = urdf_origin
                urdf_joint.parent = prim.GetParent().GetName()
                urdf_joint.child = prim.GetName()

                urdf_robot.add_joint(urdf_joint)

            if prim.HasAPI(UsdPhysics.MassAPI):
                physics_mass_api = UsdPhysics.MassAPI(prim)
                physics_mass_api.Apply(prim)

                urdf_inertial = Inertial()
                urdf_inertial.mass = physics_mass_api.GetMassAttr().Get()

                inertial_origin = Pose()
                inertial_origin.xyz = physics_mass_api.GetCenterOfMassAttr().Get()
                urdf_inertial.origin = inertial_origin

                urdf_inertia = Inertia()
                usd_inertia = physics_mass_api.GetDiagonalInertiaAttr().Get()
                urdf_inertia.ixx = usd_inertia[0]
                urdf_inertia.iyy = usd_inertia[1]
                urdf_inertia.izz = usd_inertia[2]
                urdf_inertial.inertia = urdf_inertia

                urdf_link.inertial = urdf_inertial

            for child_prim in prim.GetChildren():
                if UsdGeom.Gprim(child_prim):
                    geom_prim = UsdGeom.Gprim(child_prim)
                    geom_transform = geom_prim.GetLocalTransformation()
                    geom_scale = numpy.array([geom_transform.GetRow(i).GetLength() for i in range(3)])
                    geom_transform = geom_transform.RemoveScaleShear()

                    if prim_transform.get(prim.GetName()) is not None:
                        geom_transform = multiply_transform(prim_transform[prim.GetName()], geom_transform)

                    geom_origin = Pose()
                    geom_origin.xyz = geom_transform.ExtractTranslation()
                    geom_origin.rpy = usd_quat_to_urdf_rpy(geom_transform.ExtractRotationQuat())
                    if child_prim.HasAPI(UsdPhysics.CollisionAPI):
                        urdf_collision = Collision()
                        urdf_collision.origin = geom_origin

                        if UsdGeom.Cube(child_prim):
                            urdf_box = Box()
                            urdf_box.size = geom_scale * 2
                            urdf_collision.geometry = urdf_box

                        elif UsdGeom.Sphere(child_prim):
                            urdf_sphere = Sphere()
                            urdf_sphere.radius = UsdGeom.Sphere(child_prim).GetRadiusAttr().Get()
                            urdf_collision.geometry = urdf_sphere

                        elif UsdGeom.Cylinder(child_prim):
                            urdf_cylinder = Cylinder()
                            urdf_cylinder.radius = UsdGeom.Cylinder(child_prim).GetRadiusAttr().Get()
                            urdf_cylinder.length = UsdGeom.Cylinder(child_prim).GetHeightAttr().Get()

                            urdf_collision.geometry = urdf_cylinder

                        elif UsdGeom.Mesh(child_prim):
                            usd_mesh = UsdGeom.Mesh(child_prim)
                            mesh_points = numpy.array(usd_mesh.GetPointsAttr().Get())
                            mesh_normals = numpy.array(usd_mesh.GetNormalsAttr().Get())
                            mesh_face_vertex_indices = usd_mesh.GetFaceVertexIndicesAttr().Get()
                            mesh_faces = []
                            index = 0
                            for face_vertex_counts in usd_mesh.GetFaceVertexCountsAttr().Get():
                                face = []
                                for _ in range(face_vertex_counts):
                                    face.append(mesh_face_vertex_indices[index])
                                    index += 1
                                mesh_faces.append(face)

                            mesh_faces = numpy.array(mesh_faces, dtype=numpy.int32)
                            stl_mesh = stl.mesh.Mesh(numpy.zeros(mesh_faces.shape[0], dtype=stl.mesh.Mesh.dtype))
                            stl_mesh.vectors = mesh_points[mesh_faces]
                            stl_mesh.normals = mesh_normals
                            stl_mesh_name = child_prim.GetName() + ".stl"
                            stl_mesh.save(os.path.join(stl_mesh_dir, stl_mesh_name))

                            urdf_mesh = Mesh()
                            urdf_mesh.filename = os.path.join(stl_mesh_dir_rel, stl_mesh_name)
                            urdf_mesh.scale = geom_scale

                            urdf_collision.geometry = urdf_mesh

                        urdf_link.collision = urdf_collision

            urdf_robot.add_link(urdf_link)

    for prim in stage.Traverse():
        if UsdPhysics.RevoluteJoint(prim) or UsdPhysics.PrismaticJoint(prim):
            urdf_joint = urdf_robot.joint_map[prim.GetName()]
            usd_joint = UsdPhysics.RevoluteJoint(prim)

            parent_prim = stage.GetPrimAtPath(usd_joint.GetBody0Rel().GetTargets()[0])
            child_prim = stage.GetPrimAtPath(usd_joint.GetBody1Rel().GetTargets()[0])

            urdf_joint_origin = Pose()
            urdf_joint_transform = multiply_transform(
                urdf_link_transform[parent_prim.GetName()].GetInverse(), urdf_link_transform[child_prim.GetName()]
            )
            urdf_joint_origin.xyz = urdf_joint_transform.ExtractTranslation()
            urdf_joint_origin.rpy = usd_quat_to_urdf_rpy(urdf_joint_transform.ExtractRotation().GetQuat())
            urdf_joint.origin = urdf_joint_origin

    with open(urdf_file, "w") as file:
        file.write(urdf_robot.to_xml_string())
