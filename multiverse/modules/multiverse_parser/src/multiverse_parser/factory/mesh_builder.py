#!/usr/bin/env python3.10

from dataclasses import dataclass
import os
from typing import List, Dict

import numpy
from pxr import Usd, UsdGeom

from ..utils import modify_name


@dataclass(init=False)
class MeshProperty:
    name: str
    points: numpy.ndarray
    normals: numpy.ndarray
    face_vertex_counts: numpy.ndarray
    face_vertex_indices: numpy.ndarray

    def __init__(self,
                 name: str,
                 points: numpy.ndarray,
                 normals: numpy.ndarray,
                 face_vertex_counts: numpy.ndarray,
                 face_vertex_indices: numpy.ndarray) -> None:
        self.name = name
        self._points = points
        self._normals = normals
        self._face_vertex_counts = face_vertex_counts
        self._face_vertex_indices = face_vertex_indices
        self.check_validity()

    def check_validity(self):
        assert self.name != ""
        assert self.points.size != 0
        assert self.normals.size == self.face_vertex_counts.size * 3 == self.face_vertex_indices.size

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, name: str):
        self._name = modify_name(name)

    @property
    def points(self):
        return self._points

    @property
    def normals(self):
        return self._normals

    @property
    def face_vertex_counts(self):
        return self._face_vertex_counts

    @property
    def face_vertex_indices(self):
        return self._face_vertex_indices


class MeshBuilder:
    file_path: str
    stage: Usd.Stage
    xform: UsdGeom.Xform
    meshes: List[UsdGeom.Mesh]
    meshes_properties: Dict[str, MeshProperty]

    def __init__(self, mesh_file_path: str) -> None:
        self.meshes_properties = {}
        if os.path.exists(mesh_file_path):
            self._stage = Usd.Stage.Open(mesh_file_path)
            self._xform = UsdGeom.Xform(self._stage.GetDefaultPrim())
            for mesh in self.meshes:
                mesh_name = mesh.GetPrim().GetName()
                self.meshes_properties[mesh_name] = MeshProperty(
                    name=mesh_name,
                    points=mesh.GetPointsAttr().Get(),
                    normals=mesh.GetNormalsAttr().Get(),
                    face_vertex_counts=mesh.GetFaceVertexCountsAttr().Get(),
                    face_vertex_indices=mesh.GetFaceVertexIndicesAttr().Get()
                )
        else:
            mesh_name = os.path.splitext(os.path.basename(mesh_file_path))[0]
            mesh_name = modify_name(mesh_name)
            self._stage = Usd.Stage.CreateNew(mesh_file_path)
            self._xform = UsdGeom.Xform.Define(self._stage, f"/{mesh_name}")
            self._stage.SetDefaultPrim(self.xform.GetPrim())

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
        xform_path = self.xform.GetPath()
        mesh = UsdGeom.Mesh.Define(self.stage, xform_path.AppendChild(f"SM_{mesh_name}"))
        mesh.CreatePointsAttr(points)
        mesh.CreateNormalsAttr(normals)
        mesh.CreateFaceVertexCountsAttr(face_vertex_counts)
        mesh.CreateFaceVertexIndicesAttr(face_vertex_indices)

        self.stage.GetRootLayer().Save()

        return mesh

    @property
    def meshes(self):
        return [UsdGeom.Mesh(prim_child) for prim_child in self.xform.GetPrim().GetChildren() if
                UsdGeom.Mesh(prim_child)]

    @property
    def xform(self):
        return self._xform

    @property
    def stage(self):
        return self.xform.GetPrim().GetStage()

    @property
    def file_path(self):
        return self.stage.GetRootLayer().realPath
