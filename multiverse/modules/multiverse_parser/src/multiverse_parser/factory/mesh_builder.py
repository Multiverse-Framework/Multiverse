#!/usr/bin/env python3.10

import os
from pxr import Usd, UsdGeom, Sdf

from .material_builder import MaterialBuilder, material_dict

mesh_dict = {}


class MeshBuilder:
    def __init__(self, name: str, usd_file_path: str) -> None:
        mesh_dict[usd_file_path] = self

        self.usd_file_path = usd_file_path
        self.name = name
        root_path = Sdf.Path("/").AppendPath(self.name)
        self.mesh = None
        if os.path.exists(self.usd_file_path):
            self.stage = Usd.Stage.Open(self.usd_file_path)
        else:
            self.stage = Usd.Stage.CreateNew(self.usd_file_path)

        if (
            self.stage.GetDefaultPrim()
            and self.stage.GetDefaultPrim().GetPath() == root_path
            and UsdGeom.Xform(self.stage.GetDefaultPrim())
            and len(self.stage.GetPseudoRoot().GetChildren()) == 1
        ):
            self.xform = UsdGeom.Xform(self.stage.GetDefaultPrim())
        else:
            self.xform = UsdGeom.Xform.Define(self.stage, root_path)
            self.stage.SetDefaultPrim(self.xform.GetPrim())

        for prim_child in self.xform.GetPrim().GetChildren():
            if UsdGeom.Mesh(prim_child):
                self.mesh = UsdGeom.Mesh(prim_child)
                break

        if self.mesh is None:
            if os.path.exists(self.usd_file_path):
                print(f"Mesh prim not found in {self.usd_file_path}, create one.")
            self.mesh = UsdGeom.Mesh.Define(self.stage, root_path.AppendPath(name))

        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, UsdGeom.LinearUnits.meters)

    def save(self):
        self.stage.Save()


class VisualMeshBuilder(MeshBuilder):
    def __init__(self, name: str, usd_file_path: str) -> None:
        super().__init__(name, usd_file_path)

    def add_material(self, material_name) -> MaterialBuilder:
        return material_dict[material_name] if material_name in material_dict else MaterialBuilder(material_name, self.usd_file_path)


class CollisionMeshBuilder(MeshBuilder):
    def __init__(self, name: str, usd_file_path: str) -> None:
        super().__init__(name, usd_file_path)

    def build(self, points, normals, face_vertex_counts, face_vertex_indices) -> None:
        if self.mesh is not None:
            self.mesh.CreatePointsAttr(points)
            self.mesh.CreateNormalsAttr(normals)
            self.mesh.CreateFaceVertexCountsAttr(face_vertex_counts)
            self.mesh.CreateFaceVertexIndicesAttr(face_vertex_indices)
        else:
            print(f"Mesh prim is None.")
