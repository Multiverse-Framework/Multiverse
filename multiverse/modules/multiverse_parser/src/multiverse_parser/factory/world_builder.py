#!/usr/bin/env python3.10

from typing import Optional

from .body_builder import BodyBuilder
from ..utils import modify_name

from pxr import Usd, UsdGeom


def setup_stage(file_path: str) -> Usd.Stage:
    stage = Usd.Stage.CreateNew(file_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, UsdGeom.LinearUnits.meters)
    return stage


class WorldBuilder:
    stage: Usd.Stage

    def __init__(self, file_path: str) -> None:
        self._stage = setup_stage(file_path)
        self._body_builders = {}

    def add_body(self, body_name: str, parent_body_name: str = None, body_id: Optional[int] = None) -> BodyBuilder:
        body_name = modify_name(in_name=body_name, replacement=f"Body_{body_id}" if body_id is not None else None)

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
                parent_body_builder.add_child_body_builder(child_body_builder=body_builder)
            self._body_builders[body_name] = body_builder
        return self._body_builders[body_name]

    def get_body_builder(self, body_name: str) -> BodyBuilder:
        body_name = modify_name(in_name=body_name)
        if body_name not in self._body_builders:
            raise ValueError(f"Body {body_name} not found in {self.__class__.__name__}.")
        return self._body_builders[body_name]

    def export(self) -> None:
        self.stage.Save()

    @property
    def stage(self):
        return self._stage
