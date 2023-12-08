#!/usr/bin/env python3.10

from dataclasses import dataclass
import os
from typing import List

import numpy
from pxr import Usd, UsdGeom

from ..utils import modify_name


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
    xform: UsdGeom.Xform
    meshes: List[UsdGeom.Mesh]

    def __init__(self, mesh_file_path: str) -> None:
        self._meshes_properties = {}
        if os.path.exists(mesh_file_path):
            stage = Usd.Stage.Open(mesh_file_path)
            xform = UsdGeom.Xform(stage.GetDefaultPrim())
            for mesh in [UsdGeom.Mesh(prim_child) for prim_child in xform.GetPrim().GetChildren() if
                         UsdGeom.Mesh(prim_child)]:
                mesh_name = mesh.GetPrim().GetName()
                self.set_mesh_property(mesh_name=mesh_name,
                                       mesh_property=MeshProperty(
                                           points=numpy.array(mesh.GetPointsAttr().Get()),
                                           normals=numpy.array(mesh.GetNormalsAttr().Get()),
                                           face_vertex_counts=numpy.array(mesh.GetFaceVertexCountsAttr().Get()),
                                           face_vertex_indices=numpy.array(mesh.GetFaceVertexIndicesAttr().Get())))
        else:
            mesh_name = os.path.splitext(os.path.basename(mesh_file_path))[0]
            mesh_name = modify_name(mesh_name)
            stage = Usd.Stage.CreateNew(mesh_file_path)
            xform = UsdGeom.Xform.Define(stage, f"/{mesh_name}")
            stage.SetDefaultPrim(xform.GetPrim())
        self._stage = stage
        self._xform = xform

    def get_mesh_property(self, mesh_name: str) -> MeshProperty:
        mesh_name = modify_name(in_name=mesh_name)
        return self._meshes_properties[mesh_name]

    def set_mesh_property(self, mesh_name: str, mesh_property: MeshProperty) -> None:
        mesh_name = modify_name(in_name=mesh_name)
        self._meshes_properties[mesh_name] = mesh_property

    def create_mesh(self,
                    mesh_name: str,
                    mesh_property: MeshProperty) -> UsdGeom.Mesh:
        xform_path = self.xform.GetPath()
        mesh = UsdGeom.Mesh.Define(self.stage, xform_path.AppendChild(f"SM_{mesh_name}"))
        mesh.CreatePointsAttr(mesh_property.points)
        mesh.CreateNormalsAttr(mesh_property.normals)
        mesh.CreateFaceVertexCountsAttr(mesh_property.face_vertex_counts)
        mesh.CreateFaceVertexIndicesAttr(mesh_property.face_vertex_indices)

        self.stage.GetRootLayer().Save()

        return mesh

    @property
    def file_path(self):
        return self.stage.GetRootLayer().realPath

    @property
    def stage(self):
        return self.xform.GetPrim().GetStage()

    @property
    def xform(self):
        return self._xform

    @property
    def meshes(self) -> List[UsdGeom.Mesh]:
        return [UsdGeom.Mesh(prim_child) for prim_child in self.xform.GetPrim().GetChildren() if
                UsdGeom.Mesh(prim_child)]
