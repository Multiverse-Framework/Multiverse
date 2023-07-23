#!/usr/bin/env python3

import os
import xml.etree.ElementTree as ET
from math import degrees
import numpy
import rospy
from pxr import Usd, UsdGeom, Sdf, Gf, UsdPhysics
import mujoco


def get_sum_mass(body_element: ET.Element) -> float:
    mass = float(body_element.attrib.get("mass", "0"))
    for child_body_element in body_element.findall("body"):
        if child_body_element.find("joint") is None:
            mass += get_sum_mass(child_body_element)
    return mass


def mjcf_to_usd_with_physics_handle(xml_path: str, usd_file: str):
    usd_dir = os.path.dirname(usd_file)

    xml_mesh_dict = {}
    xml_body_gravcomp_dict = {}
    xml_tree = ET.parse(xml_path)
    xml_root = xml_tree.getroot()

    angle = "degree"

    mesh_root_dir = os.path.dirname(xml_path)
    for compiler in xml_root.findall("compiler"):
        if compiler.attrib.get("meshdir") is not None:
            mesh_root_dir = compiler.attrib.get("meshdir")
        if compiler.attrib.get("angle") is not None:
            angle = compiler.attrib.get("angle")

    for xml_asset in xml_root.findall("asset"):
        for xml_mesh in xml_asset.findall("mesh"):
            mesh_name = xml_mesh.attrib.get("name")
            mesh_file = os.path.basename(xml_mesh.attrib.get("file"))
            mesh_file = mesh_file.replace("stl", "usda")
            mesh_file = mesh_file.replace("obj", "usda")
            mesh_dir = os.path.basename(usd_file)
            mesh_dir = mesh_dir.replace(".usda", "")
            xml_mesh_dict[mesh_name] = "./" + os.path.join(mesh_dir, "usd", mesh_file)

    for body_id, xml_body in enumerate(xml_root.findall("body")):
        xml_body_gravcomp_dict[body_id] = float(xml_body.attrib.get("gravcomp", "0"))

    mj_model = mujoco.MjModel.from_xml_path(xml_path)
    mj_data = mujoco.MjData(mj_model)
    mujoco.mj_step1(mj_model, mj_data)

    for mesh_id in range(mj_model.nmesh):
        mj_mesh = mj_model.mesh(mesh_id)

        if os.path.exists(xml_mesh_dict[mj_mesh.name]):
            continue

        stage = Usd.Stage.CreateNew(os.path.join(usd_dir, xml_mesh_dict[mj_mesh.name]))

        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, UsdGeom.LinearUnits.meters)
        usd_mesh = UsdGeom.Mesh.Define(stage, "/Mesh_" + mj_mesh.name.replace("-", "_"))
        stage.SetDefaultPrim(usd_mesh.GetPrim())

        points = numpy.empty(shape=[mj_model.mesh_vertnum[mesh_id], 3], dtype=float)

        normals = numpy.empty(shape=[mj_model.mesh_facenum[mesh_id], 3], dtype=float)

        face_vertex_counts = numpy.empty(shape=mj_model.mesh_facenum[mesh_id], dtype=float)
        face_vertex_counts.fill(3)

        face_vertex_indices = numpy.empty(shape=mj_model.mesh_facenum[mesh_id] * 3, dtype=float)

        vert_adr = mj_model.mesh_vertadr[mesh_id]
        for i in range(mj_model.mesh_vertnum[mesh_id]):
            vert_id = vert_adr + i
            points[i] = mj_model.mesh_vert[vert_id]

        face_adr = mj_model.mesh_faceadr[mesh_id]
        normal_adr = mj_model.mesh_normaladr[mesh_id]
        for i in range(mj_model.mesh_facenum[mesh_id]):
            face_id = face_adr + i
            face_normals = mj_model.mesh_normal[normal_adr + mj_model.mesh_facenormal[face_id]]

            p1 = face_normals[0]
            p2 = face_normals[1]
            p3 = face_normals[2]

            v1 = p2 - p1
            v2 = p3 - p1
            normal = numpy.cross(v1, v2)
            norm = numpy.linalg.norm(normal)
            if norm != 0:
                normal = normal / norm
            normals[i] = normal

            face_vertex_indices[3 * i] = mj_model.mesh_face[face_id][0]
            face_vertex_indices[3 * i + 1] = mj_model.mesh_face[face_id][1]
            face_vertex_indices[3 * i + 2] = mj_model.mesh_face[face_id][2]

        usd_mesh.CreatePointsAttr(points)
        usd_mesh.CreateNormalsAttr(normals)
        usd_mesh.CreateFaceVertexCountsAttr(face_vertex_counts)
        usd_mesh.CreateFaceVertexIndicesAttr(face_vertex_indices)

        stage.Save()

    stage = Usd.Stage.CreateNew(os.path.join(usd_dir, usd_file))

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, UsdGeom.LinearUnits.meters)

    root_path = Sdf.Path("/").AppendPath(mj_model.body(0).name)
    root_prim = UsdGeom.Xform.Define(stage, root_path)

    body_path = root_path
    body_paths = {}
    body_paths[0] = body_path
    for body_id, body_element in enumerate(xml_tree.iter("body")):
        body_id += 1
        body = mj_model.body(body_id)
        if body_id != 0:
            parent_body_id = body.parentid[0]
            parent_body = mj_model.body(parent_body_id)
            parent_body_path = body_paths[parent_body_id]

            if body.jntnum[0] == 0:
                if body.name == "":
                    body_path = parent_body_path.AppendPath("body_" + str(body_id))
                else:
                    body_path = parent_body_path.AppendPath(body.name.replace("-", "_"))
            else:
                if body.name == "":
                    body_path = root_path.AppendPath("body_" + str(body_id))
                else:
                    body_path = root_path.AppendPath(body.name.replace("-", "_"))
            body_paths[body_id] = body_path

            body_prim = UsdGeom.Xform.Define(stage, body_path)
            transform = body_prim.AddTransformOp()
            mat = Gf.Matrix4d()

            if body.jntnum[0] == 0:
                mat.SetTranslateOnly(
                    Gf.Vec3d(
                        body.pos[0],
                        body.pos[1],
                        body.pos[2],
                    )
                )
                mat.SetRotateOnly(
                    Gf.Quatd(
                        body.quat[0],
                        body.quat[1],
                        body.quat[2],
                        body.quat[3],
                    )
                )
            else:
                mat.SetTranslateOnly(
                    Gf.Vec3d(
                        mj_data.xpos[body_id][0],
                        mj_data.xpos[body_id][1],
                        mj_data.xpos[body_id][2],
                    )
                )
                mat.SetRotateOnly(
                    Gf.Quatd(
                        mj_data.xquat[body_id][0],
                        mj_data.xquat[body_id][1],
                        mj_data.xquat[body_id][2],
                        mj_data.xquat[body_id][3],
                    )
                )

            transform.Set(mat)

            if body.geomnum[0] > 0:
                if body.jntnum[0] != 0:
                    physics_rigid_body_api = UsdPhysics.RigidBodyAPI(body_prim)
                    physics_rigid_body_api.CreateRigidBodyEnabledAttr(True)
                    physics_rigid_body_api.Apply(body_prim.GetPrim())

                physics_mass_api = UsdPhysics.MassAPI(body_prim)
                physics_mass_api.CreateCenterOfMassAttr(Gf.Vec3f(body.ipos[0], body.ipos[1], body.ipos[2]))
                if body.jntnum[0] == 0:
                    physics_mass_api.CreateMassAttr(get_sum_mass(body_element))
                else:
                    physics_mass_api.CreateMassAttr(body.mass[0])
                physics_mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(body.inertia[0], body.inertia[1], body.inertia[2]))
                physics_mass_api.Apply(body_prim.GetPrim())

        for i in range(body.geomnum[0]):
            geom_id = body.geomadr[0] + i
            geom = mj_model.geom(geom_id)
            if geom.name == "":
                geom_path = body_path.AppendPath("geom_" + str(geom_id))
            else:
                geom_path = body_path.AppendPath(geom.name.replace("-", "_"))

            mat = Gf.Matrix4d()
            mat.SetTranslateOnly(Gf.Vec3d(geom.pos[0], geom.pos[1], geom.pos[2]))
            mat.SetRotateOnly(Gf.Quatd(geom.quat[0], geom.quat[1], geom.quat[2], geom.quat[3]))

            if geom.type == mujoco.mjtGeom.mjGEOM_BOX:
                geom_prim = UsdGeom.Cube.Define(stage, geom_path)
                mat_scale = Gf.Matrix4d()
                mat_scale.SetScale(Gf.Vec3d(geom.size[0], geom.size[1], geom.size[2]))
                mat = mat_scale * mat

            elif geom.type == mujoco.mjtGeom.mjGEOM_SPHERE:
                geom_prim = UsdGeom.Sphere.Define(stage, geom_path)
                geom_prim.CreateRadiusAttr(geom.size[0])
                geom_prim.CreateExtentAttr(numpy.array([-1, -1, -1, 1, 1, 1]) * geom.size[0])

            elif geom.type == mujoco.mjtGeom.mjGEOM_CYLINDER:
                geom_prim = UsdGeom.Cylinder.Define(stage, geom_path)
                geom_prim.CreateRadiusAttr(geom.size[0])
                geom_prim.CreateHeightAttr(geom.size[1] * 2)
                geom_prim.CreateExtentAttr(
                    numpy.array(
                        [
                            [-geom.size[0], -geom.size[0], -geom.size[1]],
                            [geom.size[0], geom.size[0], geom.size[1]],
                        ]
                    )
                )

            elif geom.type == mujoco.mjtGeom.mjGEOM_MESH:
                mesh_id = geom.dataid[0]
                mesh_name = mj_model.mesh(mesh_id).name
                geom_prim = UsdGeom.Mesh.Define(stage, geom_path)
                geom_prim.GetPrim().GetReferences().AddReference(xml_mesh_dict[mesh_name])

            elif geom.type == mujoco.mjtGeom.mjGEOM_PLANE:
                geom_prim = UsdGeom.Mesh.Define(stage, geom_path)
                geom_prim.CreatePointsAttr([(-0.5, -0.5, 0), (0.5, -0.5, 0), (-0.5, 0.5, 0), (0.5, 0.5, 0)])
                geom_prim.CreateExtentAttr([(-0.5, -0.5, 0), (0.5, 0.5, 0)])
                geom_prim.CreateNormalsAttr([(0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1)])
                geom_prim.CreateFaceVertexCountsAttr([4])
                geom_prim.CreateFaceVertexIndicesAttr([0, 1, 3, 2])
                mat.SetScale(Gf.Vec3d(1, 1, 100))

            else:
                rospy.logwarn("Geom type %d of %s not implemented" % (geom.type, geom_path))

            transform = geom_prim.AddTransformOp()
            transform.Set(mat)

            physics_collision_api = UsdPhysics.CollisionAPI(geom_prim)
            physics_collision_api.CreateCollisionEnabledAttr(True)
            physics_collision_api.Apply(geom_prim.GetPrim())

            if geom.type == mujoco.mjtGeom.mjGEOM_MESH:
                physics_mesh_collision_api = UsdPhysics.MeshCollisionAPI(geom_prim)
                physics_mesh_collision_api.CreateApproximationAttr("convexHull")
                physics_mesh_collision_api.Apply(geom_prim.GetPrim())

            geom_prim.CreateDisplayColorAttr(geom.rgba[:3])
            geom_prim.CreateDisplayOpacityAttr(geom.rgba[3])

        if body_id != 0:
            physics_rigid_body_api = UsdPhysics.RigidBodyAPI(body_prim)
            if body.jntnum[0] > 0:
                for joint_nr in range(body.jntnum[0]):
                    if mj_model.joint(body.jntadr[joint_nr]).type == mujoco.mjtJoint.mjJNT_FREE:
                        continue
                    joint_id = body.jntadr[joint_nr]
                    joint = mj_model.joint(joint_id)

                    joint_pos = Gf.Vec3f(joint.pos[0], joint.pos[1], joint.pos[2])
                    joint_rot = Gf.Quatf(1, 0, 0, 0)

                    if joint.name == "":
                        joint_path = parent_body_path.AppendPath("joint_" + str(joint_id))
                    else:
                        joint_path = parent_body_path.AppendPath(joint.name.replace("-", "_"))

                    if joint.type == mujoco.mjtJoint.mjJNT_HINGE:
                        joint_prim = UsdPhysics.RevoluteJoint.Define(stage, joint_path)
                        if joint.limited[0]:
                            if angle == "radian":
                                joint_prim.CreateLowerLimitAttr(degrees(joint.range[0]))
                                joint_prim.CreateUpperLimitAttr(degrees(joint.range[1]))
                            elif angle == "degree":
                                joint_prim.CreateLowerLimitAttr(joint.range[0])
                                joint_prim.CreateUpperLimitAttr(joint.range[1])
                    elif joint.type == mujoco.mjtJoint.mjJNT_SLIDE:
                        joint_prim = UsdPhysics.PrismaticJoint.Define(stage, joint_path)
                        if joint.limited[0]:
                            joint_prim.CreateLowerLimitAttr(joint.range[0])
                            joint_prim.CreateUpperLimitAttr(joint.range[1])
                    elif joint.type == mujoco.mjtJoint.mjJNT_BALL:
                        joint_prim = UsdPhysics.SphericalJoint.Define(stage, joint_path)

                    joint_prim.CreateAxisAttr("Z")
                    if numpy.array_equal(joint.axis, [1, 0, 0]):
                        joint_rot = Gf.Quatf(0.7071068, 0, 0.7071068, 0)
                    elif numpy.array_equal(joint.axis, [0, 1, 0]):
                        joint_rot = Gf.Quatf(0.7071068, -0.7071068, 0, 0)
                    elif numpy.array_equal(joint.axis, [0, 0, 1]):
                        joint_rot = Gf.Quatf(1, 0, 0, 0)
                    elif numpy.array_equal(joint.axis, [-1, 0, 0]):
                        joint_rot = Gf.Quatf(0.7071068, 0, -0.7071068, 0)
                    elif numpy.array_equal(joint.axis, [0, -1, 0]):
                        joint_rot = Gf.Quatf(0.7071068, 0.7071068, 0, 0)
                    elif numpy.array_equal(joint.axis, [0, 0, -1]):
                        joint_rot = Gf.Quatf(0, 0, 1, 0)

                    joint_prim.CreateCollisionEnabledAttr(False)

                    joint_prim.GetBody0Rel().SetTargets([parent_body_path])
                    joint_prim.GetBody1Rel().SetTargets([body_path])

                    body1_rot = Gf.Quatf(
                        parent_body.quat[0],
                        parent_body.quat[1],
                        parent_body.quat[2],
                        parent_body.quat[3],
                    )

                    body2_pos = Gf.Vec3f(body.pos[0], body.pos[1], body.pos[2])
                    body2_rot = Gf.Quatf(body.quat[0], body.quat[1], body.quat[2], body.quat[3])

                    joint_prim.CreateLocalPos0Attr(body2_pos + body1_rot.Transform(joint_pos))
                    joint_prim.CreateLocalPos1Attr(Gf.Vec3f())

                    joint_prim.CreateLocalRot0Attr(body2_rot * joint_rot)
                    joint_prim.CreateLocalRot1Attr(joint_rot)

    stage.SetDefaultPrim(root_prim.GetPrim())

    stage.Save()