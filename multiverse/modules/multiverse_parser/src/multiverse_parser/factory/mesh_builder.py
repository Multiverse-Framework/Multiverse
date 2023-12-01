#!/usr/bin/env python3.10

import os
from typing import List

from pxr import Usd, UsdGeom, Sdf, UsdShade


class MeshBuilder:
    _stage: Usd.Stage
    _xform: UsdGeom.Xform

    def __init__(self, mesh_file_path: str) -> None:
        self._stage = Usd.Stage.Open(mesh_file_path) if os.path.exists(mesh_file_path) \
            else Usd.Stage.CreateNew(mesh_file_path)
        self._xform = UsdGeom.Xform(self._stage.GetDefaultPrim())

    def build(self) -> List[UsdGeom.Mesh]:
        meshes = []

        for prim_child in self._xform.GetPrim().GetChildren():
            mesh = UsdGeom.Mesh(prim_child)
            if mesh:
                meshes.append(mesh)

        UsdGeom.SetStageUpAxis(self._stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self._stage, UsdGeom.LinearUnits.meters)
        self._stage.Save()

        return meshes

    @property
    def xform(self):
        return self._xform

    @property
    def stage(self):
        return self._stage

    @property
    def file_path(self):
        return self.stage.GetRootLayer().realPath
