#!/usr/bin/env python3.10

import importlib.util
import os, shutil
import random, string
from pxr import Usd, UsdGeom

from .body_builder import BodyBuilder, body_dict

multiverse_parser_path = os.path.dirname(importlib.util.find_spec("multiverse_parser").origin)


TMP = "tmp"
TMP_DIR = "tmp/usd"


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
        random_string = "".join(random.choices(string.ascii_letters + string.digits, k=10))
        self.usd_file_path = os.path.join(multiverse_parser_path, ".cache", random_string, TMP + ".usda")
        print(f"Create {self.usd_file_path}")
        os.makedirs(os.path.dirname(self.usd_file_path))
        self.stage = Usd.Stage.CreateNew(self.usd_file_path)
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
            usd_file_dir = os.path.dirname(usd_file_path)
            usd_file_name = os.path.splitext(os.path.basename(usd_file_path))[0]

            copy_and_overwrite(os.path.dirname(self.usd_file_path), usd_file_dir)

            tmp_usd_file_path = os.path.join(usd_file_dir, os.path.basename(self.usd_file_path))
            os.rename(tmp_usd_file_path, usd_file_path)

            tmp_mesh_dir = os.path.join(usd_file_dir, TMP)
            new_mesh_dir = os.path.join(usd_file_dir, usd_file_name)
            if os.path.exists(new_mesh_dir):
                shutil.rmtree(new_mesh_dir)
            os.rename(tmp_mesh_dir, new_mesh_dir)

            with open(usd_file_path, "r", encoding="utf-8") as file:
                file_contents = file.read()

            tmp_path = "prepend references = @./" + TMP + "/usd/"
            new_path = "prepend references = @./" + usd_file_name + "/usd/"
            file_contents = file_contents.replace(tmp_path, new_path)

            with open(usd_file_path, "w", encoding="utf-8") as file:
                file.write(file_contents)

    def clean_up(self) -> None:
        print(f"Remove {os.path.dirname(self.usd_file_path)}")
        shutil.rmtree(os.path.dirname(self.usd_file_path))
