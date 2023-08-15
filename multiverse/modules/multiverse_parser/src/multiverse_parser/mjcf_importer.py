#!/usr/bin/env python3.10

from multiverse_parser import UsdWorld, GeomType
import mujoco


def import_mjcf(mjcf_file_path: str, with_physics: bool = True) -> UsdWorld:
    try:
        mj_model = mujoco.MjModel.from_xml_path(mjcf_file_path)
    except ValueError as error:
        print(f"{error}. Failed to import MJCF.")
        return None

    usd_world = UsdWorld()

    for body_id in range(mj_model.nbody):
        mj_body = mj_model.body(body_id)
        body_name = mj_body.name if mj_body.name != "" else "body_" + str(body_id)
        if body_id == 0:
            root_body_name = body_name
            usd_body = usd_world.add_body(body_name=root_body_name)
        else:
            mj_body_parent = mj_model.body(mj_body.parentid)
            parent_body_name = (
                mj_body_parent.name
                if mj_body_parent.name != ""
                else "body_" + str(mj_body.parentid)
            )
            if mj_body.jntnum[0] > 0 and with_physics:
                usd_body = usd_world.add_body(
                    body_name=body_name, parent_body_name=root_body_name
                )
            else:
                usd_body = usd_world.add_body(
                    body_name=body_name, parent_body_name=parent_body_name
                )
            usd_body.set_pose(
                pos=tuple(mj_body.pos),
                quat=tuple(mj_body.quat),
                relative_to=parent_body_name,
            )

        for geom_id in range(mj_body.geomadr[0], mj_body.geomadr[0] + mj_body.geomnum[0]):
            mj_geom = mj_model.geom(geom_id)
            geom_name = mj_geom.name.replace(" ", "")
            geom_name = geom_name.replace("-", "_")
            if geom_name == "":
                geom_name = "geom_" + str(geom_id)

            if mj_geom.type == mujoco.mjtGeom.mjGEOM_PLANE:
                usd_geom = usd_body.add_geom(
                    geom_name=geom_name, geom_type=GeomType.PLANE
                )
            elif mj_geom.type == mujoco.mjtGeom.mjGEOM_BOX:
                usd_geom = usd_body.add_geom(
                    geom_name=geom_name, geom_type=GeomType.CUBE
                )
            elif mj_geom.type == mujoco.mjtGeom.mjGEOM_SPHERE:
                usd_geom = usd_body.add_geom(
                    geom_name=geom_name, geom_type=GeomType.SPHERE
                )
            elif mj_geom.type == mujoco.mjtGeom.mjGEOM_CYLINDER:
                usd_geom = usd_body.add_geom(
                    geom_name=geom_name, geom_type=GeomType.CYLINDER
                )
            elif mj_geom.type == mujoco.mjtGeom.mjGEOM_MESH:
                usd_geom = usd_body.add_geom(
                    geom_name=geom_name, geom_type=GeomType.MESH
                )
            else:
                print(f"Geom type {str(mj_geom.type)} not supported.")
                continue

            if mj_geom.type == mujoco.mjtGeom.mjGEOM_BOX:
                usd_geom.set_transform(pos=tuple(mj_geom.pos), quat=tuple(mj_geom.quat), size=tuple(mj_geom.size))
            else:
                usd_geom.set_transform(pos=tuple(mj_geom.pos), quat=tuple(mj_geom.quat))
                if mj_geom.type == mujoco.mjtGeom.mjGEOM_SPHERE:
                    usd_geom.set_attribute(radius=mj_geom.size[0])
                elif mj_geom.type == mujoco.mjtGeom.mjGEOM_CYLINDER:
                    usd_geom.set_attribute(
                        radius=mj_geom.size[0], height=mj_geom.size[1] * 2
                    )
                elif mj_geom.type == mujoco.mjtGeom.mjGEOM_MESH:
                    mesh_id = mj_geom.dataid[0]
                    mesh_name = mj_model.mesh(mesh_id).name
                    usd_world.add_mesh(mesh_name)

    return usd_world
