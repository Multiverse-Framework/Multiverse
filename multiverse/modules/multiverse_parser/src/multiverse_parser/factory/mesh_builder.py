#!/usr/bin/env python3.10

import os
from typing import List

from pxr import Usd, UsdGeom, Sdf


#
# from .material_builder import MaterialBuilder, material_dict
#
# mesh_dict = {}
#
#


class MeshBuilder:
    file_path: str
    name: str
    xform: UsdGeom.Xform
    _stage: Usd.Stage

    def __init__(self, name: str, mesh_file_path: str) -> None:
        self.file_path = mesh_file_path
        self._stage = Usd.Stage.Open(self.file_path) if os.path.exists(self.file_path) \
            else Usd.Stage.CreateNew(self.file_path)
        self.name = name
        root_path = Sdf.Path("/").AppendPath(self.name)
        if (
                self._stage.GetDefaultPrim()
                and UsdGeom.Xform(self._stage.GetDefaultPrim())
                and len(self._stage.GetPseudoRoot().GetChildren()) == 1
        ):
            self.xform = UsdGeom.Xform(self._stage.GetDefaultPrim())
        else:
            self.xform = UsdGeom.Xform.Define(self._stage, root_path)
            self._stage.SetDefaultPrim(self.xform.GetPrim())

    def build(self) -> List[UsdGeom.Mesh]:
        meshes = []

        for prim_child in self.xform.GetPrim().GetChildren():
            mesh = UsdGeom.Mesh(prim_child)
            if mesh:
                meshes.append(mesh)

        # if len(meshes) == 0:
        #     print(f"Mesh prim not found in {self.file_path}, creating one...")
        #     mesh = UsdGeom.Mesh.Define(self._stage, self.xform.GetPath().AppendPath("Mesh_0"))
        #     meshes.append(mesh)

        UsdGeom.SetStageUpAxis(self._stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self._stage, UsdGeom.LinearUnits.meters)

        return meshes


    def save(self):
        self._stage.Save()


# class VisualMeshBuilder(MeshBuilder):
#     def __init__(self, name: str, usd_file_path: str) -> None:
#         super().__init__(name, usd_file_path)
#
#     def add_material(self, material_name) -> MaterialBuilder:
#         return material_dict[material_name] if material_name in material_dict else MaterialBuilder(material_name, self.usd_file_path)
#
#
# class CollisionMeshBuilder(MeshBuilder):
#     def __init__(self, name: str, usd_file_path: str) -> None:
#         super().__init__(name, usd_file_path)
#
#     def build(self, points, normals, face_vertex_counts, face_vertex_indices) -> None:
#         if self.mesh is not None:
#             self.mesh.CreatePointsAttr(points)
#             self.mesh.CreateNormalsAttr(normals)
#             self.mesh.CreateFaceVertexCountsAttr(face_vertex_counts)
#             self.mesh.CreateFaceVertexIndicesAttr(face_vertex_indices)
#         else:
#             print(f"Mesh prim is None.")
