#!/usr/bin/env python3.10

from dataclasses import dataclass
import os
from typing import Optional, List

import numpy
from pxr import Usd, Sdf, UsdShade, Gf


@dataclass(init=False)
class MaterialProperty:
    diffuse_color: Optional[numpy.ndarray]
    emissive_color: Optional[numpy.ndarray]
    specular_color: Optional[numpy.ndarray]

    def __init__(self,
                 diffuse_color: Optional[numpy.ndarray] = None,
                 emissive_color: Optional[numpy.ndarray] = None,
                 specular_color: Optional[numpy.ndarray] = None) -> None:
        self._diffuse_color = diffuse_color
        self._emissive_color = emissive_color
        self._specular_color = specular_color

    @property
    def diffuse_color(self):
        return self._diffuse_color

    @property
    def emissive_color(self):
        return self._emissive_color

    @property
    def specular_color(self):
        return self._specular_color


class MaterialBuilder:
    stage: Usd.Stage
    root_prim: Usd.Prim
    materials: List[UsdShade.Material]

    def __init__(self, file_path: str) -> None:
        self._stage = Usd.Stage.Open(file_path) \
            if os.path.exists(file_path) else Usd.Stage.CreateNew(file_path)

    def apply_material(self, material_property: MaterialProperty, mesh_prim: Optional[Usd.Prim] = None):
        if mesh_prim is None:
            for _mesh_prim in self._stage.GetDefaultPrim().GetChildren():
                self.apply_material(material_property, _mesh_prim)
        else:
            mesh_prim.ApplyAPI(UsdShade.MaterialBindingAPI)
            material = UsdShade.Material.Define(self._stage,
                                                Sdf.Path("/Materials").AppendChild(f"M_{mesh_prim.GetName()}"))
            material_binding_api = UsdShade.MaterialBindingAPI(mesh_prim)
            material_binding_api.Bind(material)

            material_path = material.GetPath()
            shader = UsdShade.Shader.Define(self._stage, material_path.AppendChild("Shader"))
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput("useSpecularWorkflow", Sdf.ValueTypeNames.Int).Set(1)
            if material_property.diffuse_color is not None:
                shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
                    Gf.Vec3f(*material_property.diffuse_color))
            if material_property.emissive_color is not None:
                shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
                    Gf.Vec3f(*material_property.emissive_color))
            if material_property.specular_color is not None:
                shader.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(
                    Gf.Vec3f(*material_property.specular_color))

            material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    @property
    def stage(self):
        return self._stage

    @property
    def root_prim(self):
        material_root_paths = [Sdf.Path("/").AppendPath("_materials"), Sdf.Path("/").AppendPath("Materials")]
        for material_root_path in material_root_paths:
            if self.stage.GetPrimAtPath(material_root_path).IsValid():
                return self.stage.GetPrimAtPath(material_root_path)
        return self.stage.DefinePrim(material_root_paths[0])

    @property
    def materials(self):
        return [UsdShade.Material(prim_child) for prim_child in self.root_prim.GetChildren() if
                UsdShade.Material(prim_child)]
