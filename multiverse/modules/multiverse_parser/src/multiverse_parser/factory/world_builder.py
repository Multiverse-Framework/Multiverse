#!/usr/bin/env python3.10
#
import os
import atexit
import shutil
from typing import Dict

from pxr import Usd, UsdGeom

from .body_builder import BodyBuilder
from .config import Cache, randomize_path
from ..utils import modify_name


# from .body_builder import BodyBuilder, body_dict
#
#
def copy_and_overwrite(source_folder: str, destination_folder: str) -> None:
    os.makedirs(name=destination_folder, exist_ok=True)

    # Iterate through all files and folders in the source folder
    for item in os.listdir(source_folder):
        source_item = os.path.join(source_folder, item)
        destination_item = os.path.join(destination_folder, item)

        # If item is a folder, call the function recursively
        if os.path.isdir(source_item):
            if os.path.exists(destination_item):
                shutil.rmtree(destination_item)
            shutil.copytree(source_item, destination_item)
        # If item is a file, simply copy it
        else:
            shutil.copy2(source_item, destination_item)


def export_to_destination(usd_file_path: str) -> None:
    usd_file_name = os.path.basename(usd_file_path).split(".")[0]

    usd_file_dir = os.path.dirname(usd_file_path)
    copy_and_overwrite(source_folder=Cache.TMP_USD_FILE_DIR, destination_folder=usd_file_dir)

    os.rename(
        os.path.join(usd_file_dir, os.path.basename(Cache.TMP_USD_FILE_PATH)),
        usd_file_path,
    )

    tmp_mesh_dir = os.path.join(usd_file_dir, Cache.TMP)
    new_mesh_dir = os.path.join(usd_file_dir, usd_file_name)
    if os.path.exists(new_mesh_dir):
        shutil.rmtree(new_mesh_dir)

    if os.path.exists(tmp_mesh_dir):
        os.rename(tmp_mesh_dir, new_mesh_dir)

        with open(usd_file_path, "r", encoding="utf-8") as file:
            file_contents = file.read()

        tmp_path = "@./" + Cache.TMP
        new_path = "@./" + usd_file_name
        file_contents = file_contents.replace(tmp_path, new_path)

        with open(usd_file_path, "w", encoding="utf-8") as file:
            file.write(file_contents)


def setup() -> Usd.Stage:
    randomize_path()
    print(f"Create {Cache.TMP_USD_FILE_DIR}.")
    stage = Usd.Stage.CreateNew(Cache.TMP_USD_FILE_PATH)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, UsdGeom.LinearUnits.meters)
    return stage


def clean_up() -> None:
    if os.path.exists(Cache.TMP_USD_FILE_DIR):
        print(f"Remove {Cache.TMP_USD_FILE_DIR}.")
        shutil.rmtree(Cache.TMP_USD_FILE_DIR)


class WorldBuilder:
    stage: Usd.Stage
    body_builders: Dict[str, BodyBuilder] = {}

    def __init__(self) -> None:
        self.stage = setup()
        atexit.register(clean_up)

    def add_body(self, body_name: str, parent_body_name: str = None) -> BodyBuilder:
        body_name = modify_name(in_name=body_name)

        if body_name in self.body_builders:
            print(f"Body {body_name} already exists.")
        else:
            if parent_body_name is None:
                body_builder = BodyBuilder(stage=self.stage, name=body_name)
                self.stage.SetDefaultPrim(body_builder.xform.GetPrim())
            else:
                parent_body_name = modify_name(in_name=parent_body_name)
                if parent_body_name not in self.body_builders:
                    raise ValueError(f"Parent body {parent_body_name} not found.")
                parent_body_builder = self.body_builders[parent_body_name]
                parent_xform = parent_body_builder.xform
                body_builder = BodyBuilder(stage=self.stage, name=body_name, parent_xform=parent_xform)
            self.body_builders[body_name] = body_builder
        return self.body_builders[body_name]

    def export(self, usd_file_path: str = None) -> str:
        self.stage.Save()

        if usd_file_path is not None:
            export_to_destination(usd_file_path=usd_file_path)
            return usd_file_path
        else:
            return Cache.TMP_USD_FILE_PATH
