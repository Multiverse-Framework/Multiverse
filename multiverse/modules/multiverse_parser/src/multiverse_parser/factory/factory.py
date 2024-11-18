#!/usr/bin/env python3

import atexit
import os
from dataclasses import dataclass, field
import random
import shutil
import string
import subprocess
from typing import Optional, Dict, Tuple, List

import numpy
from pxr import Usd, UsdShade, Sdf

from .world_builder import WorldBuilder
from .body_builder import InertiaSource
from ..utils import (import_obj, import_stl, import_dae, import_usd,
                     export_obj, export_stl, export_dae, export_usd)


@dataclass
class Configuration:
    """
    Configuration class for the Multiverse Parser.
    """
    model_name: str = ""
    fixed_base: bool = True
    with_physics: bool = True
    with_visual: bool = True
    with_collision: bool = True
    inertia_source: InertiaSource = InertiaSource.FROM_SRC
    default_rgba: numpy.ndarray = field(default_factory=lambda: numpy.array([0.9, 0.9, 0.9, 1.0]))


def merge_folders(source_folder: str,
                  destination_folder: str,
                  excludes: Optional[List[str]] = None) -> None:
    os.makedirs(name=destination_folder, exist_ok=True)

    # Iterate through all files and folders in the source folder
    for item in os.listdir(source_folder):
        source_item = os.path.join(source_folder, item)
        destination_item = os.path.join(destination_folder, item)

        # If item is a folder, call the function recursively
        if os.path.isdir(source_item):
            if excludes is None:
                shutil.copytree(source_item, destination_item,
                                dirs_exist_ok=True)
            else:
                shutil.copytree(source_item, destination_item,
                                dirs_exist_ok=True,
                                ignore=shutil.ignore_patterns(*excludes))
        # If item is a file, simply copy it
        else:
            _, extension = os.path.splitext(source_item)
            if excludes is None or extension not in excludes:
                shutil.copy2(source_item, destination_item)


def fix_texture_path(usd_mesh_file_path: str):
    stage = Usd.Stage.Open(usd_mesh_file_path)
    material_prims = []

    for prim in stage.TraverseAll():
        if prim.HasAPI(UsdShade.MaterialBindingAPI):
            material_binding_api = UsdShade.MaterialBindingAPI(prim)
            material_path = material_binding_api.GetDirectBindingRel().GetTargets()[0]
            material_prim = stage.GetPrimAtPath(material_path)
            material_prims.append(material_prim)

    for shader in [UsdShade.Shader(child_prim)
                   for material_prim in material_prims
                   for child_prim in material_prim.GetChildren()]:
        if shader.GetIdAttr().Get() == "UsdPreviewSurface":
            diffuse_color_input = shader.GetInput("diffuseColor")
            if diffuse_color_input.HasConnectedSource():
                source = diffuse_color_input.GetConnectedSource()[0]
                if len(source.GetOutputs()) != 1:
                    for output in source.GetOutputs():
                        if output.GetBaseName() == "rgb":
                            output_prim = output.GetPrim()
                            break
                    else:
                        raise NotImplementedError("Multiple outputs are not supported yet.")
                else:
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
                texture_file_path = file_input.path
                if os.path.relpath(texture_file_path):
                    texture_file_path = os.path.join(os.path.dirname(usd_mesh_file_path), texture_file_path)
                new_texture_file_path = os.path.join(os.path.dirname(usd_mesh_file_path), "..", "..", "..", "textures",
                                                     os.path.basename(texture_file_path))
                os.makedirs(name=os.path.dirname(new_texture_file_path), exist_ok=True)
                if not os.path.exists(new_texture_file_path):
                    shutil.move(texture_file_path, new_texture_file_path)
                if texture_file_path != new_texture_file_path and os.path.exists(texture_file_path):
                    os.remove(texture_file_path)
                    if len(os.listdir(os.path.dirname(texture_file_path))) == 0:
                        os.rmdir(os.path.dirname(texture_file_path))

                new_texture_file_relpath = os.path.relpath(new_texture_file_path,
                                                           os.path.dirname(usd_mesh_file_path))
                output_shader.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(new_texture_file_relpath)
                stage.GetRootLayer().Save()


class Factory:
    world_builder: WorldBuilder
    source_file_path: str
    config: Configuration
    tmp_usd_file_path: str
    tmp_mesh_dir_path: str
    tmp_texture_dir_path: str
    tmp_material_dir_path: str
    _mesh_file_path_dict: Dict[str, Tuple[str, str]] = {}

    def __init__(self, file_path: str, config: Configuration = Configuration()):
        self._world_builder = None
        self._source_file_path = file_path
        (self._tmp_usd_file_path,
         self._tmp_mesh_dir_path,
         self._tmp_material_dir_path,
         self._tmp_texture_dir_path) = self._create_tmp_paths()
        self._config = config
        self._cmds = []
        atexit.register(self.clean_up)

    def _create_tmp_paths(self) -> Tuple[str, str, str, str]:
        """
        Create temporary paths for the USD file and the mesh directory.
        :return: Tuple of the temporary USD file path and the mesh dir, material dir and texture dir paths.
        """
        tmp_dir_path = os.path.join(f"/{self.tmp_file_name}",
                                    "cache",
                                    "".join(random.choices(string.ascii_letters + string.digits, k=10)))
        tmp_usd_file_path = os.path.join(tmp_dir_path, f"{self.tmp_file_name}.usda")
        tmp_mesh_dir_path = os.path.join(tmp_dir_path, self.tmp_file_name, "meshes")
        tmp_material_dir_path = os.path.join(tmp_dir_path, self.tmp_file_name, "materials")
        tmp_texture_dir_path = os.path.join(tmp_dir_path, self.tmp_file_name, "textures")
        os.makedirs(name=tmp_dir_path, exist_ok=True)
        os.makedirs(name=tmp_mesh_dir_path, exist_ok=True)
        os.makedirs(name=tmp_material_dir_path, exist_ok=True)
        os.makedirs(name=tmp_texture_dir_path, exist_ok=True)
        print(f"Created {tmp_dir_path}, {tmp_mesh_dir_path}, {tmp_material_dir_path} and {tmp_texture_dir_path}.")
        return tmp_usd_file_path, tmp_mesh_dir_path, tmp_material_dir_path, tmp_texture_dir_path

    def import_model(self, save_file_path: Optional[str] = None) -> str:
        """
        Import the model from the source file path to the temporary file path.
        :param save_file_path: Optional path to save the USD file to.
        :return: If save_file_path is None, return the temporary file path. Otherwise, return the save_file_path.
        """
        raise NotImplementedError

    def import_mesh(self,
                    mesh_file_path: str,
                    mesh_scale: numpy.ndarray = numpy.array([1.0, 1.0, 1.0]),
                    merge_mesh: bool = False,
                    execute_later: bool = False) -> Tuple[str, str]:
        """
        Import the mesh from the mesh file path to the temporary mesh directory path.
        :param mesh_file_path: Path to the mesh file.
        :param mesh_scale: Scale of the mesh.
        :param merge_mesh: Merge the mesh.
        :param execute_later: Execute the command later (for batch processing).
        :return: Tuple of the temporary USD mesh file path and the temporary mesh file path.
        """
        if mesh_file_path in self.mesh_file_path_dict:
            return self.mesh_file_path_dict[mesh_file_path]

        mesh_file_name = os.path.basename(mesh_file_path).split(".")[0]
        mesh_file_extension = os.path.splitext(mesh_file_path)[1]
        tmp_mesh_file_path = os.path.join(self.tmp_mesh_dir_path,
                                          mesh_file_extension[1:].replace("usda", "usd").replace("usdz", "usd").replace("usd", "usda"),
                                          f"{mesh_file_name}{mesh_file_extension}")
        tmp_usd_mesh_file_path = os.path.join(self.tmp_mesh_dir_path,
                                              "usd",
                                              f"from_{mesh_file_extension[1:]}",
                                              f"{mesh_file_name}.usda")

        self.mesh_file_path_dict[mesh_file_path] = tmp_usd_mesh_file_path, tmp_mesh_file_path

        print("Importing mesh from", mesh_file_path, "to", tmp_usd_mesh_file_path, "and", tmp_mesh_file_path, ".")
        mesh_file_path_clone = os.path.join(os.path.dirname(mesh_file_path),
                                            f"clone_{os.path.basename(mesh_file_path)}")
        if mesh_file_extension in [".usd", ".usda", ".usdz"]:
            os.makedirs(name=os.path.dirname(tmp_mesh_file_path), exist_ok=True)
            shutil.copyfile(mesh_file_path, tmp_mesh_file_path)
            if mesh_file_path != self.source_file_path:
                cmd = import_usd([mesh_file_path], mesh_scale) + export_usd(tmp_usd_mesh_file_path, merge_mesh)
            else:
                shutil.copyfile(mesh_file_path, mesh_file_path_clone)
                cmd = import_usd([mesh_file_path_clone], mesh_scale) + export_usd(tmp_usd_mesh_file_path, merge_mesh)
        elif mesh_file_extension == ".obj":
            cmd = import_obj([mesh_file_path], mesh_scale) + export_obj(tmp_mesh_file_path) + export_usd(
                tmp_usd_mesh_file_path, merge_mesh)
        elif mesh_file_extension == ".stl":
            cmd = import_stl([mesh_file_path], mesh_scale) + export_stl(tmp_mesh_file_path) + export_usd(
                tmp_usd_mesh_file_path, merge_mesh)
        elif mesh_file_extension == ".dae":
            cmd = import_dae([mesh_file_path], mesh_scale) + export_dae(tmp_mesh_file_path) + export_usd(
                tmp_usd_mesh_file_path, merge_mesh)
        else:
            raise ValueError(f"Unsupported file extension {mesh_file_extension}.")

        if not execute_later:
            cmd = ["blender",
                   "--background",
                   "--python-expr",
                   f"import bpy"
                   f"{cmd}"]

            process = subprocess.Popen(cmd)
            process.wait()

            if mesh_file_path == self.source_file_path and os.path.exists(mesh_file_path_clone):
                os.remove(mesh_file_path_clone)

            fix_texture_path(usd_mesh_file_path=tmp_usd_mesh_file_path)
        else:
            self._cmds.append(cmd)

        return tmp_usd_mesh_file_path, tmp_mesh_file_path

    def export_mesh(self,
                    in_mesh_file_path: str,
                    out_mesh_file_path: str,
                    mesh_scale: numpy.ndarray = numpy.array([1.0, 1.0, 1.0]),
                    execute_later: bool = False) -> None:
        in_mesh_file_extension = os.path.splitext(in_mesh_file_path)[1]
        out_mesh_file_extension = os.path.splitext(out_mesh_file_path)[1]
        if in_mesh_file_path in self.mesh_file_path_dict:
            tmp_usd_mesh_file_path, tmp_origin_mesh_file_path = self.mesh_file_path_dict[in_mesh_file_path]
            if out_mesh_file_extension == os.path.splitext(tmp_origin_mesh_file_path)[1]:
                shutil.copyfile(tmp_origin_mesh_file_path, out_mesh_file_path)
                return

        if in_mesh_file_extension in [".usd", ".usda", ".usdz"]:
            cmd = import_usd([in_mesh_file_path], mesh_scale)
        elif in_mesh_file_extension == ".obj":
            cmd = import_obj([in_mesh_file_path], mesh_scale)
        elif in_mesh_file_extension == ".stl":
            cmd = import_stl([in_mesh_file_path], mesh_scale)
        elif in_mesh_file_extension == ".dae":
            cmd = import_dae([in_mesh_file_path], mesh_scale)
        else:
            raise ValueError(f"Unsupported file extension {in_mesh_file_extension}.")

        if out_mesh_file_extension in [".usd", ".usda", ".usdz"]:
            cmd += export_usd(out_mesh_file_path)
        elif out_mesh_file_extension == ".obj":
            cmd += export_obj(out_mesh_file_path)
        elif out_mesh_file_extension == ".stl":
            cmd += export_stl(out_mesh_file_path)
        elif out_mesh_file_extension == ".dae":
            cmd += export_dae(out_mesh_file_path)
        else:
            raise ValueError(f"Unsupported file extension {out_mesh_file_extension}.")

        if not execute_later:
            cmd = ["blender",
                   "--background",
                   "--python-expr",
                   f"import bpy"
                   f"{cmd}"]

            process = subprocess.Popen(cmd)
            process.wait()

            if ".usd" in out_mesh_file_extension:
                fix_texture_path(usd_mesh_file_path=out_mesh_file_path)
        else:
            self._cmds.append(cmd)

    def execute_cmds(self) -> None:
        if len(self.cmds) == 0:
            return
        cmd = ["blender", "--background", "--python-expr", "import bpy"]
        for sub_cmd in self.cmds:
            cmd[3] += sub_cmd

        process = subprocess.Popen(cmd)
        process.wait()

        self._cmds = []

    def save_tmp_model(self,
                       usd_file_path: str,
                       excludes: Optional[List[str]] = None) -> None:
        self.world_builder.stage.GetRootLayer().Save()

        usd_file_name = os.path.basename(usd_file_path).split(".")[0]
        usd_dir_path = os.path.dirname(usd_file_path)
        tmp_usd_dir_path = os.path.dirname(self.tmp_usd_file_path)

        merge_folders(source_folder=tmp_usd_dir_path,
                      destination_folder=usd_dir_path,
                      excludes=excludes)

        new_usd_file_path = os.path.join(usd_dir_path, os.path.basename(self.tmp_usd_file_path))
        if os.path.exists(new_usd_file_path):
            shutil.move(new_usd_file_path, usd_file_path)

        new_mesh_dir_path = os.path.join(usd_dir_path, usd_file_name)
        tmp_mesh_dir_path = os.path.join(usd_dir_path, self.tmp_file_name)
        if os.path.exists(tmp_mesh_dir_path):
            merge_folders(source_folder=tmp_mesh_dir_path,
                          destination_folder=new_mesh_dir_path)
            shutil.rmtree(tmp_mesh_dir_path)

            if os.path.exists(usd_file_path):
                with open(usd_file_path, encoding="utf-8") as file:
                    file_contents = file.read()

                new_usd_mesh_dir_path = usd_file_name
                file_contents = file_contents.replace(os.path.dirname(self.tmp_mesh_dir_path), new_usd_mesh_dir_path)

                tmp_mesh_dir_relpath = os.path.relpath(os.path.dirname(self.tmp_mesh_dir_path),
                                                       os.path.dirname(self.tmp_usd_file_path))
                file_contents = file_contents.replace(tmp_mesh_dir_relpath, new_usd_mesh_dir_path)

                with open(usd_file_path, "w", encoding="utf-8") as file:
                    file.write(file_contents)

    def clean_up(self) -> None:
        """
        Remove the temporary directory.
        :return: None
        """
        tmp_dir_path = os.path.dirname(self.tmp_usd_file_path)
        if os.path.exists(tmp_dir_path):
            print(f"Remove {tmp_dir_path}.")
            shutil.rmtree(tmp_dir_path)

    @property
    def world_builder(self) -> WorldBuilder:
        return self._world_builder

    @property
    def tmp_file_name(self) -> str:
        return "tmp"

    @property
    def tmp_usd_file_path(self) -> str:
        return self._tmp_usd_file_path

    @property
    def tmp_mesh_dir_path(self) -> str:
        return self._tmp_mesh_dir_path

    @property
    def tmp_material_dir_path(self) -> str:
        return self._tmp_material_dir_path

    @property
    def tmp_texture_dir_path(self) -> str:
        return self._tmp_texture_dir_path

    @property
    def mesh_file_path_dict(self) -> Dict[str, Tuple[str, str]]:
        return self._mesh_file_path_dict

    @property
    def source_file_path(self) -> str:
        return self._source_file_path

    @source_file_path.setter
    def source_file_path(self, file_path: str) -> None:
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"File {file_path} not found.")
        self._source_file_path = file_path

    @property
    def config(self) -> Configuration:
        return self._config

    @config.setter
    def config(self, config: Configuration) -> None:
        if not isinstance(config, Configuration):
            raise TypeError(f"Expected {Configuration}, got {type(config)}")
        self._config = config

    @property
    def cmds(self) -> List[str]:
        return self._cmds
