#!/usr/bin/env python3.10

from multiverse_parser import WorldBuilder, GeomType, JointType
import mujoco
import numpy


def modify_name(in_name: str, prefix: str, name_id: str) -> str:
    out_name = in_name.replace(" ", "").replace("-", "_")
    if out_name == "":
        out_name = prefix + str(name_id)
    return out_name


def import_from_mjcf(mjcf_file_path: str, with_physics: bool = True) -> WorldBuilder:
    try:
        mj_model = mujoco.MjModel.from_xml_path(mjcf_file_path)
    except ValueError as error:
        print(f"{error}. Failed to import MJCF.")
        return None

    world_builder = WorldBuilder()

    usd_meshes = {}

    for body_id in range(mj_model.nbody):
        mj_body = mj_model.body(body_id)
        body_name = modify_name(mj_body.name, "body_", body_id)

        if body_id == 0:
            root_body_name = body_name
            body_builder = world_builder.add_body(body_name=root_body_name)
        else:
            mj_body_parent = mj_model.body(mj_body.parentid)
            parent_body_name = modify_name(mj_body_parent.name, "body_", mj_body.parentid)
            if mj_body.jntnum[0] > 0 and with_physics:
                body_builder = world_builder.add_body(body_name=body_name, parent_body_name=root_body_name)
            else:
                body_builder = world_builder.add_body(body_name=body_name, parent_body_name=parent_body_name)
            body_builder.set_transform(
                pos=tuple(mj_body.pos),
                quat=tuple(mj_body.quat),
                relative_to=parent_body_name,
            )

        for geom_id in range(mj_body.geomadr[0], mj_body.geomadr[0] + mj_body.geomnum[0]):
            mj_geom = mj_model.geom(geom_id)
            geom_name = modify_name(mj_geom.name, "geom_", geom_id)

            if mj_geom.type == mujoco.mjtGeom.mjGEOM_PLANE:
                geom_builder = body_builder.add_geom(geom_name=geom_name, geom_type=GeomType.PLANE)
            elif mj_geom.type == mujoco.mjtGeom.mjGEOM_BOX:
                geom_builder = body_builder.add_geom(geom_name=geom_name, geom_type=GeomType.CUBE)
            elif mj_geom.type == mujoco.mjtGeom.mjGEOM_SPHERE:
                geom_builder = body_builder.add_geom(geom_name=geom_name, geom_type=GeomType.SPHERE)
            elif mj_geom.type == mujoco.mjtGeom.mjGEOM_CYLINDER:
                geom_builder = body_builder.add_geom(geom_name=geom_name, geom_type=GeomType.CYLINDER)
            elif mj_geom.type == mujoco.mjtGeom.mjGEOM_MESH:
                geom_builder = body_builder.add_geom(geom_name=geom_name, geom_type=GeomType.MESH)
            else:
                print(f"Geom type {str(mj_geom.type)} not supported.")
                continue

            if mj_geom.type == mujoco.mjtGeom.mjGEOM_BOX:
                geom_builder.set_transform(
                    pos=tuple(mj_geom.pos),
                    quat=tuple(mj_geom.quat),
                    scale=tuple(mj_geom.size),
                )
            elif mj_geom.type == mujoco.mjtGeom.mjGEOM_PLANE:
                geom_builder.set_transform(pos=tuple(mj_geom.pos), quat=tuple(mj_geom.quat), scale=(50, 50, 1))
            else:
                geom_builder.set_transform(pos=tuple(mj_geom.pos), quat=tuple(mj_geom.quat))
                if mj_geom.type == mujoco.mjtGeom.mjGEOM_SPHERE:
                    geom_builder.set_attribute(radius=mj_geom.size[0])
                elif mj_geom.type == mujoco.mjtGeom.mjGEOM_CYLINDER:
                    geom_builder.set_attribute(radius=mj_geom.size[0], height=mj_geom.size[1] * 2)
                elif mj_geom.type == mujoco.mjtGeom.mjGEOM_MESH:
                    mesh_id = mj_geom.dataid[0]
                    mesh_name = modify_name(mj_model.mesh(mesh_id).name, "mesh_", mesh_id)

                    mesh_builder = geom_builder.add_mesh(mesh_name)
                    if mesh_builder not in usd_meshes:
                        vert_adr = mj_model.mesh_vertadr[mesh_id]
                        vert_num = mj_model.mesh_vertnum[mesh_id]

                        face_adr = mj_model.mesh_faceadr[mesh_id]
                        face_num = mj_model.mesh_facenum[mesh_id]
                        points = numpy.empty(shape=[vert_num, 3], dtype=float)

                        normals = numpy.empty(shape=[mj_model.mesh_facenum[mesh_id], 3], dtype=float)

                        face_vertex_counts = numpy.empty(shape=mj_model.mesh_facenum[mesh_id], dtype=float)
                        face_vertex_counts.fill(3)

                        face_vertex_indices = numpy.empty(shape=mj_model.mesh_facenum[mesh_id] * 3, dtype=float)

                        for i in range(vert_num):
                            vert_id = vert_adr + i
                            points[i] = mj_model.mesh_vert[vert_id]

                        face_adr = mj_model.mesh_faceadr[mesh_id]
                        normal_adr = mj_model.mesh_normaladr[mesh_id]
                        for i in range(face_num):
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
                        mesh_builder.build(points, normals, face_vertex_counts, face_vertex_indices)
                        mesh_builder.save()

            geom_builder.set_attribute(prefix="primvars", displayColor=mj_geom.rgba[:3])
            geom_builder.set_attribute(prefix="primvars", displayOpacity=mj_geom.rgba[3])

            geom_builder.compute_extent()

        if with_physics:
            body_builder.enable_collision()
            body_builder.set_inertial(mj_body.mass[0], tuple(mj_body.ipos), tuple(mj_body.inertia))

            for i in range(mj_body.jntnum[0]):
                if mj_model.joint(mj_body.jntadr[i]).type == mujoco.mjtJoint.mjJNT_FREE:
                    continue

                joint_id = mj_body.jntadr[i]
                mj_joint = mj_model.joint(joint_id)
                joint_name = modify_name(mj_joint.name, "joint_", joint_id)
                if mj_joint.type == mujoco.mjtJoint.mjJNT_HINGE:
                    joint_type = JointType.REVOLUTE
                elif mj_joint.type == mujoco.mjtJoint.mjJNT_SLIDE:
                    joint_type = JointType.PRISMATIC
                elif mj_joint.type == mujoco.mjtJoint.mjJNT_BALL:
                    joint_type = JointType.SPHERICAL
                else:
                    print(f"Joint {joint_name} type {str(mj_joint.type)} not supported.")
                    continue

                if numpy.array_equal(mj_joint.axis, [1, 0, 0]):
                    joint_axis = "X"
                elif numpy.array_equal(mj_joint.axis, [0, 1, 0]):
                    joint_axis = "Y"
                elif numpy.array_equal(mj_joint.axis, [0, 0, 1]):
                    joint_axis = "Z"
                elif numpy.array_equal(mj_joint.axis, [-1, 0, 0]):
                    joint_axis = "-X"
                elif numpy.array_equal(mj_joint.axis, [0, -1, 0]):
                    joint_axis = "-Y"
                elif numpy.array_equal(mj_joint.axis, [0, 0, -1]):
                    joint_axis = "-Z"
                else:
                    print(f"Joint {joint_name} axis {str(mj_joint.axis)} not supported.")
                    continue

                joint_builder = body_builder.add_joint(
                    joint_name,
                    body_name,
                    parent_body_name,
                    joint_type,
                    tuple(mj_joint.pos),
                    joint_axis,
                )

                if mj_joint.type == mujoco.mjtJoint.mjJNT_HINGE or mj_joint.type == mujoco.mjtJoint.mjJNT_SLIDE:
                    joint_builder.set_limit(mj_joint.range[0], mj_joint.range[1])

    return world_builder
