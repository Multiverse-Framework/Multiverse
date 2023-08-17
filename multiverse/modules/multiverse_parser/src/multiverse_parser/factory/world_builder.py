#!/usr/bin/env python3.10

import os, shutil
from pxr import Usd, UsdGeom

from multiverse_parser.factory import TMP, TMP_USD_FILE_DIR, TMP_USD_FILE_PATH
from .body_builder import BodyBuilder, body_dict


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


class WorldBuilder:
    def __init__(self) -> None:
        print(f"Create {TMP_USD_FILE_PATH}")
        os.makedirs(TMP_USD_FILE_DIR)
        self.stage = Usd.Stage.CreateNew(TMP_USD_FILE_PATH)
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, UsdGeom.LinearUnits.meters)

    def add_body(self, body_name: str, parent_body_name: str = None) -> BodyBuilder:
        if body_name in body_dict:
            print(f"Body {body_name} already exists.")
            return body_dict[body_name]

        if parent_body_name is None:
            self.root_body = BodyBuilder(self.stage, body_name)
            self.stage.SetDefaultPrim(self.root_body.prim.GetPrim())
            return self.root_body
        else:
            return BodyBuilder(self.stage, body_name, parent_body_name)

    def export(self, usd_file_path: str = None) -> None:
        self.stage.Save()

        if usd_file_path is not None:
            usd_file = os.path.basename(usd_file_path)
            usd_file_name, usd_file_extension = os.path.splitext(usd_file)
            print(usd_file_path, usd_file_extension)

            if usd_file_extension == ".usda":
                usd_file_dir = os.path.dirname(usd_file_path)
                copy_and_overwrite(TMP_USD_FILE_DIR, usd_file_dir)

                os.rename(
                    os.path.join(usd_file_dir, os.path.basename(TMP_USD_FILE_PATH)),
                    usd_file_path,
                )

                tmp_mesh_dir = os.path.join(usd_file_dir, TMP)
                new_mesh_dir = os.path.join(usd_file_dir, usd_file_name)
                if os.path.exists(new_mesh_dir):
                    shutil.rmtree(new_mesh_dir)

                if os.path.exists(tmp_mesh_dir):
                    os.rename(tmp_mesh_dir, new_mesh_dir)

                    with open(usd_file_path, "r", encoding="utf-8") as file:
                        file_contents = file.read()

                    tmp_path = "prepend references = @./" + TMP
                    new_path = "prepend references = @./" + usd_file_name
                    file_contents = file_contents.replace(tmp_path, new_path)

                    with open(usd_file_path, "w", encoding="utf-8") as file:
                        file.write(file_contents)
            else:
                self.stage.GetRootLayer().Export(usd_file_path)


    def clean_up(self) -> None:
        print(f"Remove {TMP_USD_FILE_DIR}")
        shutil.rmtree(TMP_USD_FILE_DIR)
