#!/usr/bin/env python3

from dataclasses import dataclass
import os
from typing import List

import numpy

from ..utils import modify_name

from pxr import Usd, UsdGeom


@dataclass(init=False)
class MeshProperty:
    points: numpy.ndarray
    normals: numpy.ndarray
    face_vertex_counts: numpy.ndarray
    face_vertex_indices: numpy.ndarray

    def __init__(self,
                 points: numpy.ndarray,
                 normals: numpy.ndarray,
                 face_vertex_counts: numpy.ndarray,
                 face_vertex_indices: numpy.ndarray) -> None:
        self._points = points
        self._normals = normals
        self._face_vertex_counts = face_vertex_counts
        self._face_vertex_indices = face_vertex_indices
        self.check_validity()

    def check_validity(self):
        assert self.points.size != 0
        assert all(face_vertex_count == 3 for face_vertex_count in self.face_vertex_counts)
        assert self.face_vertex_counts.size * 3 == self.face_vertex_indices.size

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
    mesh: UsdGeom.Mesh
    subsets: List[UsdGeom.Subset]

    def __init__(self, stage, mesh_property: MeshProperty) -> None:
        self._mesh_property = mesh_property
        self._mesh = UsdGeom.Mesh(stage.GetDefaultPrim())

    def build(self) -> UsdGeom.Mesh:
        mesh = self.mesh
        mesh.CreatePointsAttr(self.points)
        mesh.CreateNormalsAttr(self.normals)
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        mesh.CreateFaceVertexCountsAttr(self.face_vertex_counts)
        mesh.CreateFaceVertexIndicesAttr(self.face_vertex_indices)

        self.stage.GetRootLayer().Save()

        return self.mesh

    @property
    def mesh(self):
        return self._mesh

    @property
    def stage(self):
        return self.mesh.GetPrim().GetStage()

    @property
    def file_path(self):
        return self.stage.GetRootLayer().realPath

    @property
    def points(self):
        return self._mesh_property.points

    @property
    def normals(self):
        return self._mesh_property.normals

    @property
    def face_vertex_counts(self):
        return self._mesh_property.face_vertex_counts

    @property
    def face_vertex_indices(self):
        return self._mesh_property.face_vertex_indices
