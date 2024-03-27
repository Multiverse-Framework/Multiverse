#!/usr/bin/env python3

from dataclasses import dataclass
import os
from typing import Optional, Union, Any

import numpy

from .texture_builder import TextureBuilder

from pxr import Usd, Sdf, UsdShade, Gf


def get_input(shader: UsdShade.Shader, shader_input: str) -> Any:
    shader_inputs = [shader_input.GetBaseName() for shader_input in shader.GetInputs()]
    if shader_input not in shader_inputs:
        return None
    shader_input = shader.GetInput(shader_input)
    if shader_input.HasConnectedSource():
        source = shader_input.GetConnectedSource()[0]
        if len(source.GetOutputs()) != 1:
            raise NotImplementedError("Multiple outputs are not supported yet.")
        output = source.GetOutputs()[0]
        output_prim = output.GetPrim()
        if not output_prim.IsA(UsdShade.Shader):
            raise NotImplementedError("Only shader output is supported.")
        output_shader = UsdShade.Shader(output_prim)
        if output_shader.GetIdAttr().Get() != "UsdUVTexture":
            raise NotImplementedError("Only texture shader is supported.")
        file_input = output_shader.GetInput("file").Get()
        if file_input is None:
            raise NotImplementedError("Only texture file input is supported.")
        file_path = file_input.path
        if os.path.relpath(file_path):
            file_path = os.path.join(os.path.dirname(shader.GetPrim().GetStage().GetRootLayer().realPath),
                                     file_path)
        return os.path.normpath(file_path)
    else:
        return shader_input.Get()


@dataclass(init=False)
class MaterialProperty:
    diffuse_color: Optional[Union[numpy.ndarray, str]]
    opacity: Optional[float]
    emissive_color: Optional[numpy.ndarray]
    specular_color: Optional[numpy.ndarray]
    wrap_s = "repeat"
    wrap_t = "repeat"

    def __init__(self,
                 diffuse_color: Any = None,
                 opacity: Any = None,
                 emissive_color: Any = None,
                 specular_color: Any = None,
                 wrap_s: str = "repeat",
                 wrap_t: str = "repeat") -> None:
        self._diffuse_color = diffuse_color if isinstance(diffuse_color, str) \
            else numpy.array(diffuse_color) if diffuse_color is not None \
            else None
        self._opacity = opacity if isinstance(opacity, float) else 1.0
        self._emissive_color = numpy.array(emissive_color) if emissive_color is not None else None
        self._specular_color = numpy.array(specular_color) if specular_color is not None else None
        self._wrap_s = wrap_s
        self._wrap_t = wrap_t

    @classmethod
    def from_material_file_path(cls, material_file_path: str, material_path: Sdf.Path) -> "MaterialProperty":
        material_stage = Usd.Stage.Open(material_file_path)
        material_prim = material_stage.GetPrimAtPath(material_path)
        if not material_prim.IsA(UsdShade.Material):
            raise TypeError(f"{material_path} is not a material")
        return cls.from_prim(material_prim=material_prim)

    @classmethod
    def from_prim(cls, material_prim: Usd.Prim) -> "MaterialProperty":
        for shader in [UsdShade.Shader(child_prim) for child_prim in material_prim.GetChildren()]:
            if shader.GetIdAttr().Get() == "UsdPreviewSurface":
                return cls.from_pbr_shader(pbr_shader=shader)

        raise NotImplementedError(
            f"Material {material_prim.GetName()} does not have a shader with UsdPreviewSurface id.")

    @classmethod
    def from_pbr_shader(cls, pbr_shader: UsdShade.Shader) -> "MaterialProperty":
        wrap_s = "repeat"
        wrap_t = "repeat"
        # TODO: Implement wrap_s and wrap_t

        diffuse_color = get_input(shader=pbr_shader, shader_input="diffuseColor")
        if isinstance(diffuse_color, Gf.Vec3f):
            diffuse_color = numpy.array(diffuse_color)

        opacity = get_input(shader=pbr_shader, shader_input="opacity")
        if opacity is not None:
            if isinstance(opacity, str):
                print(f"Opacity {opacity} not supported yet, using 1.0 instead.")
                opacity = 1.0
            else:
                opacity = float(opacity)

        emissive_color = get_input(shader=pbr_shader, shader_input="emissiveColor")
        if isinstance(emissive_color, Gf.Vec3f):
            emissive_color = numpy.array(emissive_color)

        specular_color = get_input(shader=pbr_shader, shader_input="specularColor")
        if isinstance(specular_color, Gf.Vec3f):
            specular_color = numpy.array(specular_color)

        return cls(diffuse_color=diffuse_color,
                   opacity=opacity,
                   emissive_color=emissive_color,
                   specular_color=specular_color,
                   wrap_s=wrap_s,
                   wrap_t=wrap_t)

    @property
    def diffuse_color(self):
        return self._diffuse_color

    @property
    def opacity(self):
        return self._opacity

    @property
    def emissive_color(self):
        return self._emissive_color

    @property
    def specular_color(self):
        return self._specular_color

    @property
    def wrap_s(self):
        return self._wrap_s

    @property
    def wrap_t(self):
        return self._wrap_t


class MaterialBuilder:
    stage: Usd.Stage
    material: UsdShade.Material

    def __init__(self, stage, material_property: MaterialProperty) -> None:
        self._material_property = material_property
        self._material = UsdShade.Material(stage.GetDefaultPrim())

    def build(self) -> UsdShade.Material:
        material = self.material
        pbr_shader = UsdShade.Shader.Define(self.stage,
                                            material.GetPath().AppendChild("PBRShader"))
        pbr_shader.CreateIdAttr("UsdPreviewSurface")
        pbr_shader.CreateInput("useSpecularWorkflow", Sdf.ValueTypeNames.Int).Set(1)
        diffuse_color = self.diffuse_color
        if diffuse_color is not None:
            if isinstance(diffuse_color, numpy.ndarray):
                diffuse_color_input = Gf.Vec3f(*diffuse_color.tolist())
                pbr_shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(diffuse_color_input)
            elif isinstance(diffuse_color, str):
                texture_file_path = diffuse_color
                if not os.path.isabs(texture_file_path):
                    texture_file_path = os.path.join(os.path.dirname(self.stage.GetRootLayer().realPath),
                                                     texture_file_path)
                texture_builder = TextureBuilder(file_path=texture_file_path)
                rgb = texture_builder.rgb
                texture_name = os.path.splitext(os.path.basename(texture_file_path))[0]
                new_texture_file_path = os.path.join(os.path.dirname(self.stage.GetRootLayer().realPath),
                                                     "..",
                                                     "..",
                                                     "textures",
                                                     f"{texture_name}.png")
                self.add_texture(file_path=new_texture_file_path, rgb=rgb, wrap_s=self.wrap_s, wrap_t=self.wrap_t)
            else:
                raise ValueError(f"Unsupported diffuse color type: {type(diffuse_color)}")

        opacity = self.opacity
        if isinstance(opacity, float):
            pbr_shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)

        emissive_color = self.emissive_color
        if isinstance(emissive_color, numpy.ndarray):
            emissive_color_input = Gf.Vec3f(*emissive_color.tolist())
            pbr_shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(emissive_color_input)

        specular_color = self.specular_color
        if isinstance(specular_color, numpy.ndarray):
            specular_color_input = Gf.Vec3f(*specular_color.tolist())
            pbr_shader.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(specular_color_input)

        material.CreateSurfaceOutput().ConnectToSource(pbr_shader.ConnectableAPI(), "surface")

        self.stage.GetRootLayer().Save()

        return material

    def add_texture(self, file_path: str, rgb: numpy.ndarray, wrap_s: str = "repeat", wrap_t: str = "repeat") -> TextureBuilder:
        if not os.path.isabs(file_path):
            file_abspath = os.path.join(os.path.dirname(self.stage.GetRootLayer().realPath), file_path)
        else:
            file_abspath = file_path
        file_relpath = os.path.relpath(file_abspath, os.path.dirname(self.stage.GetRootLayer().realPath))
        texture_builder = TextureBuilder(file_path=file_abspath)
        texture_builder.rgb = rgb

        st_input = self.material.CreateInput('frame:stPrimvarName', Sdf.ValueTypeNames.Token)
        st_input.Set('st')

        material_path = self.material.GetPath()
        prim_var_reader_path = material_path.AppendChild("PrimvarReader")
        prim_var_reader = UsdShade.Shader.Define(self.stage, prim_var_reader_path)
        prim_var_reader.CreateIdAttr("UsdPrimvarReader_float2")

        diffuse_texture_path = material_path.AppendChild("DiffuseTexture")
        diffuse_texture = UsdShade.Shader.Define(self.stage, diffuse_texture_path)
        diffuse_texture.CreateIdAttr('UsdUVTexture')
        diffuse_texture_file_input = diffuse_texture.CreateInput('file', Sdf.ValueTypeNames.Asset)
        diffuse_texture_file_input.Set(file_relpath)
        diffuse_texture_st_input = diffuse_texture.CreateInput("st", Sdf.ValueTypeNames.Float2)
        diffuse_texture_st_input.ConnectToSource(prim_var_reader.ConnectableAPI(), 'result')
        diffuse_texture_wrap_s_input = diffuse_texture.CreateInput("wrapS", Sdf.ValueTypeNames.Token)
        diffuse_texture_wrap_s_input.Set(wrap_s)
        diffuse_texture_wrap_t_input = diffuse_texture.CreateInput("wrapT", Sdf.ValueTypeNames.Token)
        diffuse_texture_wrap_t_input.Set(wrap_t)
        diffuse_texture.CreateOutput('rgb', Sdf.ValueTypeNames.Float3)

        pbr_shader = UsdShade.Shader(self.stage.GetPrimAtPath(material_path.AppendChild("PBRShader")))
        if pbr_shader is None:
            raise ValueError(f"Material {material_path} does not have a PBRShader.")

        diffuse_color_input = pbr_shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f)
        diffuse_color_input.ConnectToSource(diffuse_texture.ConnectableAPI(), 'rgb')

        var_name_input = prim_var_reader.CreateInput('varname', Sdf.ValueTypeNames.Token)
        var_name_input.ConnectToSource(st_input)

        self.stage.GetRootLayer().Save()

        return texture_builder

    @property
    def stage(self):
        return self.material.GetPrim().GetStage()

    @property
    def material(self):
        return self._material

    @property
    def diffuse_color(self):
        return self._material_property.diffuse_color

    @property
    def opacity(self):
        return self._material_property.opacity

    @property
    def emissive_color(self):
        return self._material_property.emissive_color

    @property
    def specular_color(self):
        return self._material_property.specular_color

    @property
    def wrap_s(self):
        return self._material_property.wrap_s

    @property
    def wrap_t(self):
        return self._material_property.wrap_t
