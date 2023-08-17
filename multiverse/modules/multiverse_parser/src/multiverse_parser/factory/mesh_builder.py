#!/usr/bin/env python3.10

import os
from pxr import Usd, UsdGeom, Sdf

from .material_builder import MaterialBuilder, material_dict

mesh_dict = {}


class MeshBuilder:
    def __init__(self, name: str, usd_file_path: str) -> None:
        mesh_dict[name] = self

        root_path = Sdf.Path("/").AppendPath(name)

        self.mesh_prim = None
        if os.path.exists(usd_file_path):
            self.stage = Usd.Stage.Open(usd_file_path)
        else:
            self.stage = Usd.Stage.CreateNew(usd_file_path)

        if self.stage.GetDefaultPrim() and self.stage.GetDefaultPrim().GetPath() == root_path and UsdGeom.Xform(self.stage.GetDefaultPrim()) and len(self.stage.GetPseudoRoot().GetChildren()) == 1:
            self.root_prim = UsdGeom.Xform(self.stage.GetDefaultPrim())
        else:
            self.root_prim = UsdGeom.Xform.Define(self.stage, root_path)
            self.stage.SetDefaultPrim(self.root_prim.GetPrim())

        for prim_child in self.root_prim.GetPrim().GetChildren():
            if UsdGeom.Mesh(prim_child):
                self.mesh_prim = UsdGeom.Mesh(prim_child)
                break

        if self.mesh_prim is None:
            if os.path.exists(usd_file_path):
                print(f"Mesh prim not found in {usd_file_path}, create one.")
            self.mesh_prim = UsdGeom.Mesh.Define(self.stage, root_path.AppendPath("SM_" + name))

        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, UsdGeom.LinearUnits.meters)

    def save(self):
        self.stage.Save()


class VisualMeshBuilder(MeshBuilder):
    def __init__(self, name: str, usd_file_path: str) -> None:
        super().__init__(name, usd_file_path)
        self.usd_file_path = usd_file_path

    def add_material(self, material_name) -> MaterialBuilder:
        if material_name in material_dict:
            material = material_dict[material_name]
        else:
            material = MaterialBuilder(material_name, self.usd_file_path)
        
        return material


class CollisionMeshBuilder(MeshBuilder):
    def __init__(self, name: str, usd_file_path: str) -> None:
        super().__init__(name, usd_file_path)

    def build(self, points, normals, face_vertex_counts, face_vertex_indices):
        if self.mesh_prim is not None:
            self.mesh_prim.CreatePointsAttr(points)
            self.mesh_prim.CreateNormalsAttr(normals)
            self.mesh_prim.CreateFaceVertexCountsAttr(face_vertex_counts)
            self.mesh_prim.CreateFaceVertexIndicesAttr(face_vertex_indices)
        else:
            print(f"Mesh prim is None.")
