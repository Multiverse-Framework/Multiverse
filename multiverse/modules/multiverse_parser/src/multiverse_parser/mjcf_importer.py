#!/usr/bin/env python3.10

from multiverse_parser import WorldBuilder, GeomType
import mujoco
import numpy


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
        body_name = mj_body.name if mj_body.name != "" else "body_" + str(body_id)
        if body_id == 0:
            root_body_name = body_name
            body_builder = world_builder.add_body(body_name=root_body_name)
        else:
            mj_body_parent = mj_model.body(mj_body.parentid)
            parent_body_name = mj_body_parent.name if mj_body_parent.name != "" else "body_" + str(mj_body.parentid)
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
            geom_name = mj_geom.name.replace(" ", "").replace("-", "_")
            if geom_name == "":
                geom_name = "geom_" + str(geom_id)

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
                    mesh_name = mj_model.mesh(mesh_id).name.replace(" ", "").replace("-", "_")
                    if mesh_name == "":
                        mesh_name = "mesh_" + str(mesh_id)
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

    return world_builder
