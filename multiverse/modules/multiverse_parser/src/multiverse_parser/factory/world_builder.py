#!/usr/bin/env python3.10

from typing import Dict

from pxr import Usd, UsdGeom

from .body_builder import BodyBuilder
from ..utils import modify_name


def setup_stage(file_path: str) -> Usd.Stage:
    stage = Usd.Stage.CreateNew(file_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, UsdGeom.LinearUnits.meters)
    return stage


class WorldBuilder:
    _stage: Usd.Stage
    _body_builders: Dict[str, BodyBuilder]

    def __init__(self, file_path: str) -> None:
        self._stage = setup_stage(file_path)
        self._body_builders = {}

    def add_body(self, body_name: str, parent_body_name: str = None) -> BodyBuilder:
        body_name = modify_name(in_name=body_name)

        if body_name in self._body_builders:
            print(f"Body {body_name} already exists.")
        else:
            if parent_body_name is None:
                body_builder = BodyBuilder(stage=self._stage, name=body_name)
                self._stage.SetDefaultPrim(body_builder.xform.GetPrim())
            else:
                parent_body_builder = self.get_body_builder(body_name=parent_body_name)
                parent_xform = parent_body_builder.xform
                body_builder = BodyBuilder(stage=self._stage, name=body_name, parent_xform=parent_xform)
            self._body_builders[body_name] = body_builder
        return self._body_builders[body_name]

    def get_body_builder(self, body_name: str) -> BodyBuilder:
        body_name = modify_name(in_name=body_name)
        if body_name not in self._body_builders:
            raise ValueError(f"Body {body_name} not found in {self.__class__.__name__}.")
        return self._body_builders[body_name]

    def export(self) -> None:
        self._stage.Save()
