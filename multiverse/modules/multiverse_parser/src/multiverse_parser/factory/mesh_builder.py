#!/usr/bin/env python3.10

import os

import numpy
from pxr import Usd, UsdGeom

from ..utils import modify_name


class MeshBuilder:
    _stage: Usd.Stage
    _xform: UsdGeom.Xform

    def __init__(self, mesh_file_path: str) -> None:
        if os.path.exists(mesh_file_path):
            self._stage = Usd.Stage.Open(mesh_file_path)
            self._xform = UsdGeom.Xform(self._stage.GetDefaultPrim())
        else:
            mesh_name = os.path.splitext(os.path.basename(mesh_file_path))[0]
            mesh_name = modify_name(mesh_name)
            self._stage = Usd.Stage.CreateNew(mesh_file_path)
            self._xform = UsdGeom.Xform.Define(self._stage, f"/{mesh_name}")
            self._stage.SetDefaultPrim(self._xform.GetPrim())

    def create_mesh(self,
                    mesh_name: str,
                    points: numpy.ndarray = numpy.array([]),
                    normals: numpy.ndarray = numpy.array([]),
                    face_vertex_counts: numpy.ndarray = numpy.array([]),
                    face_vertex_indices: numpy.ndarray = numpy.array([])) -> UsdGeom.Mesh:
        """
        Create a mesh from the given data.
        :param mesh_name: Name of the mesh.
        :param points: List of points.
        :param normals: List of normals.
        :param face_vertex_counts: List of face vertex counts.
        :param face_vertex_indices: List of face vertex indices.
        :return: The created mesh.
        """
        mesh_name = modify_name(mesh_name)
        xform_path = self._xform.GetPath()
        mesh = UsdGeom.Mesh.Define(self._stage, xform_path.AppendChild(f"SM_{mesh_name}"))
        mesh.CreatePointsAttr(points)
        mesh.CreateNormalsAttr(normals)
        mesh.CreateFaceVertexCountsAttr(face_vertex_counts)
        mesh.CreateFaceVertexIndicesAttr(face_vertex_indices)

        self._stage.GetRootLayer().Save()

        return mesh

    @property
    def meshes(self):
        return [UsdGeom.Mesh(prim_child) for prim_child in self._xform.GetPrim().GetChildren() if
                UsdGeom.Mesh(prim_child)]

    @property
    def xform(self):
        return self._xform

    @property
    def stage(self):
        return self._stage

    @property
    def file_path(self):
        return self.stage.GetRootLayer().realPath
