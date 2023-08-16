#!/usr/bin/env python3.10

import os
from pxr import Usd, UsdGeom

mesh_dict = {}


class MeshBuilder:
    def __init__(self, name: str, usd_file_path: str) -> None:
        mesh_dict[name] = self
        if os.path.exists(usd_file_path):
            self.stage = Usd.Stage.Open(usd_file_path)
            self.prim = self.stage.GetDefaultPrim()
        else:
            self.stage = Usd.Stage.CreateNew(usd_file_path)
            self.prim = UsdGeom.Mesh.Define(self.stage, "/Mesh_" + name)
            self.stage.SetDefaultPrim(self.prim.GetPrim())

        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, UsdGeom.LinearUnits.meters)

    def save(self):
        self.stage.Save()


class VisualMeshBuilder(MeshBuilder):
    def __init__(self, name: str, usd_file_path: str) -> None:
        super().__init__(name, usd_file_path)


class CollisionMeshBuilder(MeshBuilder):
    def __init__(self, name: str, usd_file_path: str) -> None:
        super().__init__(name, usd_file_path)

    def build(self, points, normals, face_vertex_counts, face_vertex_indices):
        self.prim.CreatePointsAttr(points)
        self.prim.CreateNormalsAttr(normals)
        self.prim.CreateFaceVertexCountsAttr(face_vertex_counts)
        self.prim.CreateFaceVertexIndicesAttr(face_vertex_indices)
