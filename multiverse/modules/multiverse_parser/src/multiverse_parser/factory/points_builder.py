#!/usr/bin/env python3

from dataclasses import dataclass
from typing import List, Optional

import numpy

from pxr import Usd, UsdGeom, Sdf


@dataclass(init=False)
class PointProperty:
    id: int
    pos: numpy.ndarray
    width: float

    def __init__(self, point_id: int, point_pos: numpy.ndarray, point_width: float) -> None:
        self._id = point_id
        self._pos = point_pos
        self._width = point_width

    @property
    def id(self) -> int:
        return self._id

    @property
    def pos(self) -> numpy.ndarray:
        return self._pos

    @property
    def width(self) -> float:
        return self._width


class PointsBuilder:
    stage: Usd.Stage
    points: UsdGeom.Points

    def __init__(self,
                 stage: Usd.Stage,
                 points_path: Sdf.Path,
                 points_property: List[PointProperty],
                 points_rgba: Optional[numpy.ndarray] = None) -> None:
        self._points_properties = points_property
        self._points = UsdGeom.Points.Define(stage, points_path)
        if points_rgba is not None:
            self.points.CreateDisplayColorAttr(points_rgba[:3])
            self.points.CreateDisplayOpacityAttr(points_rgba[3])

    def add_point(self, point_property: PointProperty) -> None:
        self.points_properties.append(point_property)

    def build(self):
        ids = [point_property.id for point_property in self.points_properties]
        self.points.GetIdsAttr().Set(ids)

        positions = [point_property.pos for point_property in self.points_properties]
        self.points.GetPointsAttr().Set(positions)

        widths = [point_property.width for point_property in self.points_properties]
        self.points.GetWidthsAttr().Set(widths)

    @property
    def stage(self) -> Usd.Stage:
        return self.points.GetPrim().GetStage()

    @property
    def points(self) -> UsdGeom.Points:
        return self._points

    @property
    def points_properties(self) -> List[PointProperty]:
        return self._points_properties
