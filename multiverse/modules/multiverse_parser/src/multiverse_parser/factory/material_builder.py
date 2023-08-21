#!/usr/bin/env python3.10

import os
from pxr import Usd, UsdShade, Sdf

material_dict = {}


class MaterialBuilder:
    def __init__(self, name: str, usd_file_path: str) -> None:
        material_dict[name] = self
        material_root_paths = [Sdf.Path("/").AppendPath("Materials"), Sdf.Path("/").AppendPath("_materials")]

        self.root_prim = None
        self.materials = []
        if os.path.exists(usd_file_path):
            self.stage = Usd.Stage.Open(usd_file_path)
        else:
            self.stage = self.stage.CreateNew(usd_file_path)

        for material_root_path in material_root_paths:
            self.root_prim = self.stage.GetPrimAtPath(material_root_path)
            if self.root_prim.IsValid():
                break

        if self.root_prim is None or not self.root_prim.IsValid():
            self.root_prim = self.stage.DefinePrim(material_root_paths[0])

        for prim in self.root_prim.GetChildren():
            if UsdShade.Material(prim):
                self.materials.append(UsdShade.Material(prim))

        if len(self.materials) == 0:
            if os.path.exists(usd_file_path):
                print(f"Material prim not found in {usd_file_path}, create one.")
            self.materials = [UsdShade.Material.Define(self.stage, self.root_prim.GetPath().AppendPath(name))]

    def save(self) -> None:
        self.stage.Save()
