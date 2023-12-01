#!/usr/bin/env python3.10

import os

from pxr import Usd, Sdf


class MaterialBuilder:
    _stage: Usd.Stage

    def __init__(self, file_path: str) -> None:
        self._stage = Usd.Stage.Open(file_path) \
            if os.path.exists(file_path) else Usd.Stage.CreateNew(file_path)

    @property
    def root_prim(self):
        material_root_paths = [Sdf.Path("/").AppendPath("Materials"), Sdf.Path("/").AppendPath("_materials")]
        for material_root_path in material_root_paths:
            if self._stage.GetPrimAtPath(material_root_path).IsValid():
                return self._stage.GetPrimAtPath(material_root_path)
        return self._stage.DefinePrim(material_root_paths[0])
