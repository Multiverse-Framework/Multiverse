#!/usr/bin/env python3

import os
import sys
import xml.etree.ElementTree as ET
import numpy
import rospy
from pxr import Usd, UsdGeom, Sdf, Gf
import mujoco


def mjcf_to_usd_handle(xml_path: str, usd_file: str):
    usd_dir = os.path.dirname(usd_file)

    xml_mesh_dict = {}
    xml_tree = ET.parse(xml_path)
    xml_root = xml_tree.getroot()

    mesh_root_dir = os.path.dirname(xml_path)
    for compiler in xml_root.findall('compiler'):
        if compiler.attrib.get('meshdir') is not None:
            mesh_root_dir = compiler.attrib.get('meshdir')
            break
            
    for xml_asset in xml_root.findall('asset'):
        for xml_mesh in xml_asset.findall('mesh'):
            mesh_name = xml_mesh.attrib.get('name')
            mesh_dir = os.path.join(mesh_root_dir, os.path.dirname(xml_mesh.attrib.get('file')))
            mesh_file = os.path.basename(xml_mesh.attrib.get('file'))
            mesh_file = mesh_file.replace('stl', 'usda')
            mesh_file = mesh_file.replace('obj', 'usda')
            mesh_dir = os.path.basename(usd_file)
            mesh_dir = mesh_dir.replace('.usda', '')
            xml_mesh_dict[mesh_name] = './' + os.path.join(mesh_dir, 'usd', mesh_file)

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
        usd_mesh = UsdGeom.Mesh.Define(
            stage, '/Mesh_' + mj_mesh.name.replace('-', '_'))
        stage.SetDefaultPrim(usd_mesh.GetPrim())

        points = numpy.empty(
            shape=[mj_model.mesh(mesh_id).vertnum[0], 3], dtype=float)
        # normals = numpy.empty(
        #     shape=[mj_model.mesh(mesh_id).vertnum[0], 3], dtype=float)

        face_vertex_counts = numpy.empty(
            shape=mj_model.mesh(mesh_id).facenum[0], dtype=float
        )
        face_vertex_counts.fill(3)
        face_vertex_indices = numpy.empty(
            shape=mj_model.mesh(mesh_id).facenum[0] * 3, dtype=float
        )
        
        for i in range(mj_model.mesh(mesh_id).vertnum[0]):
            vert_id = mj_model.mesh(mesh_id).vertadr[0] + i
            if vert_id < mj_model.nmeshvert:
                points[i] = mj_model.mesh_vert[vert_id]
                
        # for i in range(mj_model.mesh(mesh_id).normalnum[0]):
        #     normal_id = mj_model.mesh(mesh_id).normaladr[0] + i
        #     if normal_id < mj_model.nmeshnormal:
        #         normals[i] = mj_model.mesh_normal[normal_id]

        for i in range(mj_model.mesh(mesh_id).facenum[0]):
            faceid = mj_model.mesh(mesh_id).faceadr[0] + i
            face_vertex_indices[3 * i] = mj_model.mesh_face[faceid][0]
            face_vertex_indices[3 * i + 1] = mj_model.mesh_face[faceid][1]
            face_vertex_indices[3 * i + 2] = mj_model.mesh_face[faceid][2]

        usd_mesh.CreatePointsAttr(points)
        # usd_mesh.CreateNormalsAttr(normals)
        usd_mesh.CreateFaceVertexCountsAttr(face_vertex_counts)
        usd_mesh.CreateFaceVertexIndicesAttr(face_vertex_indices)

        stage.Save()

    stage = Usd.Stage.CreateNew(os.path.join(usd_dir, usd_file))

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, UsdGeom.LinearUnits.meters)

    root_path = Sdf.Path('/').AppendPath(mj_model.body(0).name)
    root_prim = UsdGeom.Xform.Define(stage, root_path)

    body_path = root_path
    body_paths = {}
    body_paths[0] = body_path
    for body_id, _ in enumerate(xml_tree.iter('body')):
        body_id += 1
        body = mj_model.body(body_id)
        if body_id != 0:
            parent_body_id = body.parentid[0]

            if body.name == '':
                body_path = body_paths[parent_body_id].AppendPath(
                    '/body_' + str(body_id))
            else:
                body_path = body_paths[parent_body_id].AppendPath(
                    body.name.replace('-', '_'))

            body_paths[body_id] = body_path

            body_prim = UsdGeom.Xform.Define(stage, body_path)
            transform = body_prim.AddTransformOp()
            mat = Gf.Matrix4d()
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
            transform.Set(mat)

        for i in range(body.geomnum[0]):
            geom_id = body.geomadr[0] + i
            geom = mj_model.geom(geom_id)
            if geom.name == '':
                geom_path = body_path.AppendPath('geom_' + str(geom_id))
            else:
                geom_path = body_path.AppendPath(geom.name.replace('-', '_'))

            mat = Gf.Matrix4d()
            mat.SetTranslateOnly(
                Gf.Vec3d(geom.pos[0], geom.pos[1], geom.pos[2]))
            mat.SetRotateOnly(
                Gf.Quatd(geom.quat[0], geom.quat[1],
                         geom.quat[2], geom.quat[3])
            )

            if geom.type == mujoco.mjtGeom.mjGEOM_BOX:
                geom_prim = UsdGeom.Cube.Define(stage, geom_path)
                mat_scale = Gf.Matrix4d()
                mat_scale.SetScale(
                    Gf.Vec3d(geom.size[0], geom.size[1], geom.size[2]))
                mat = mat_scale * mat

            elif geom.type == mujoco.mjtGeom.mjGEOM_SPHERE:
                geom_prim = UsdGeom.Sphere.Define(stage, geom_path)
                geom_prim.CreateRadiusAttr(geom.size[0])
                geom_prim.CreateExtentAttr(
                    numpy.array([-1, -1, -1, 1, 1, 1]) * geom.size[0]
                )

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
                geom_prim.GetPrim().GetReferences().AddReference(
                    xml_mesh_dict[mesh_name])

            elif geom.type == mujoco.mjtGeom.mjGEOM_PLANE:
                geom_prim = UsdGeom.Mesh.Define(stage, geom_path)
                geom_prim.CreatePointsAttr(
                    [(-0.5, -0.5, 0), (0.5, -0.5, 0),
                     (-0.5, 0.5, 0), (0.5, 0.5, 0)]
                )
                geom_prim.CreateExtentAttr([(-0.5, -0.5, 0), (0.5, 0.5, 0)])
                geom_prim.CreateNormalsAttr(
                    [(0, 0, 1), (0, 0, 1), (0, 0, 1), (0, 0, 1)]
                )
                geom_prim.CreateFaceVertexCountsAttr([4])
                geom_prim.CreateFaceVertexIndicesAttr([0, 1, 3, 2])
                mat.SetScale(Gf.Vec3d(1, 1, 100))

            else:
                rospy.logwarn(
                    'Geom type %d of %s not implemented' % (
                        geom.type, geom_path)
                )

            transform = geom_prim.AddTransformOp()
            transform.Set(mat)

            geom_prim.CreateDisplayColorAttr(geom.rgba[:3])
            geom_prim.CreateDisplayOpacityAttr(geom.rgba[3])

    stage.SetDefaultPrim(root_prim.GetPrim())

    stage.Save()

    return


if __name__ == '__main__':
    if len(sys.argv) >= 3:
        (xml_file, usd_file) = (sys.argv[1], sys.argv[2])
    else:
        print('Usage: in_mjcf.mjcf out_usd.usda')
        sys.exit(1)

    mjcf_to_usd_handle(xml_file, usd_file)
