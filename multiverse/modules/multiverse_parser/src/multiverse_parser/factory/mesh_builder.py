#!/usr/bin/env python3.10

from pxr import Usd, UsdGeom

mesh_dict = {}


class MeshBuilder:
    def __init__(self, name: str, usd_file_path: str) -> None:
        mesh_dict[name] = self
        self.stage = Usd.Stage.CreateNew(usd_file_path)
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, UsdGeom.LinearUnits.meters)
        self.prim = UsdGeom.Mesh.Define(self.stage, "/Mesh_" + name)
        self.stage.SetDefaultPrim(self.prim.GetPrim())

    def build(self, points, normals, face_vertex_counts, face_vertex_indices):
        self.prim.CreatePointsAttr(points)
        self.prim.CreateNormalsAttr(normals)
        self.prim.CreateFaceVertexCountsAttr(face_vertex_counts)
        self.prim.CreateFaceVertexIndicesAttr(face_vertex_indices)

    def save(self):
        self.stage.Save()
