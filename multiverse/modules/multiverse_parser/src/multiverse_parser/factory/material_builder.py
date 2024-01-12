#!/usr/bin/env python3

from dataclasses import dataclass
import os
from typing import Optional, List

import numpy

from .texture_builder import TextureBuilder

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

    def apply_material(self, material_property: MaterialProperty, mesh_prim: Optional[Usd.Prim] = None) -> List[UsdShade.Material]:
        materials = []
        if mesh_prim is None:
            for _mesh_prim in self._stage.GetDefaultPrim().GetChildren():
                materials += self.apply_material(material_property, _mesh_prim)
        else:
            mesh_prim.ApplyAPI(UsdShade.MaterialBindingAPI)
            material_name = mesh_prim.GetName()
            if material_name[:3] == "SM_":
                material_name = material_name[3:]
            if material_name[:2] != "M_":
                material_name = f"M_{material_name}"
            material = UsdShade.Material.Define(self.stage,
                                                Sdf.Path("/Materials").AppendChild(material_name))
            material_binding_api = UsdShade.MaterialBindingAPI(mesh_prim)
            material_binding_api.Bind(material)

            material_path = material.GetPath()
            pbr_shader = UsdShade.Shader.Define(self.stage,
                                                material_path.AppendChild("PBRShader"))
            pbr_shader.CreateIdAttr("UsdPreviewSurface")
            pbr_shader.CreateInput("useSpecularWorkflow", Sdf.ValueTypeNames.Int).Set(1)
            if material_property.diffuse_color is not None:
                pbr_shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
                    Gf.Vec3f(*material_property.diffuse_color))
            if material_property.emissive_color is not None:
                pbr_shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
                    Gf.Vec3f(*material_property.emissive_color))
            if material_property.specular_color is not None:
                pbr_shader.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(
                    Gf.Vec3f(*material_property.specular_color))

            material.CreateSurfaceOutput().ConnectToSource(pbr_shader.ConnectableAPI(), "surface")
            materials.append(material)

        self.stage.GetRootLayer().Save()

        return materials

    def add_texture(self, file_path: str, rgb: numpy.ndarray, material: UsdShade.Material) -> TextureBuilder:
        if not os.path.isabs(file_path):
            file_abspath = os.path.join(os.path.dirname(self.stage.GetRootLayer().realPath), file_path)
        else:
            file_abspath = file_path
        texture_builder = TextureBuilder(file_path=file_abspath)
        texture_builder.rgb = rgb

        material_path = material.GetPath()
        uv_map_path = material_path.AppendChild("uvmap")
        uv_map = UsdShade.Shader.Define(self.stage, uv_map_path)
        uv_map.CreateIdAttr("UsdPrimvarReader_float2")

        diffuse_texture_path = material_path.AppendChild("diffuse_texture")
        diffuse_texture = UsdShade.Shader.Define(self.stage, diffuse_texture_path)
        diffuse_texture.CreateIdAttr('UsdUVTexture')
        diffuse_texture.CreateInput('file', Sdf.ValueTypeNames.Asset).Set(file_path)
        diffuse_texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(uv_map.ConnectableAPI(), 'result')
        diffuse_texture.CreateOutput('rgb', Sdf.ValueTypeNames.Float3)

        pbr_shader = UsdShade.Shader(self.stage.GetPrimAtPath(material_path.AppendChild("PBRShader")))
        if pbr_shader is None:
            raise ValueError(f"Material {material_path} does not have a PBRShader.")

        pbr_shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
            diffuse_texture.ConnectableAPI(), 'rgb')

        uv_input = material.CreateInput('frame:stPrimvarName', Sdf.ValueTypeNames.Token)
        uv_input.Set('uv')

        uv_map.CreateInput('varname', Sdf.ValueTypeNames.Token).ConnectToSource(uv_input)

        self.stage.GetRootLayer().Save()

        return texture_builder

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
