#!/usr/bin/env python3

from dataclasses import dataclass

import numpy

from pxr import Usd, UsdGeom, Sdf


@dataclass(init=False)
class PointProperty:
    id: int
    pos: numpy.ndarray
    size: float
    rgba: numpy.ndarray

    def __init__(self, point_id: int, point_pos: numpy.ndarray, point_size: float, point_rgba: numpy.ndarray) -> None:
        self._id = point_id
        self._pos = point_pos
        self._size = point_size
        self._rgba = point_rgba

    @property
    def id(self) -> int:
        return self._id

    @property
    def pos(self) -> numpy.ndarray:
        return self._pos

    @property
    def size(self) -> float:
        return self._size

    @property
    def rgba(self) -> numpy.ndarray:
        return self._rgba


class PointBuilder:
    stage: Usd.Stage
    points: UsdGeom.Points

    def __init__(self,
                 stage: Usd.Stage,
                 points_path: Sdf.Path,
                 point_property: PointProperty) -> None:
        points_prim = stage.GetPrimAtPath(points_path)
        self._point_property = point_property
        if points_prim.IsValid() and points_prim.IsA(UsdGeom.Points):
            self._points = UsdGeom.Points(points_prim)
        else:
            self._points = UsdGeom.Points.Define(stage, points_path)
            self.points.CreateDisplayColorAttr(self.rgba[:3])
            self.points.CreateDisplayOpacityAttr(self.rgba[3])

    def build(self):
        ids = self.points.GetIdsAttr().Get()
        if ids is None:
            ids = [self.id]
        else:
            ids = [*ids, self.id]
        self.points.GetIdsAttr().Set(ids)

        positions = self.points.GetPointsAttr().Get()
        if positions is None:
            positions = [self.pos]
        else:
            positions = [*positions, self.pos]
        self.points.GetPointsAttr().Set(positions)

        widths = self.points.GetWidthsAttr().Get()
        if widths is None:
            widths = [self.size]
        else:
            widths = [*widths, self.size]
        self.points.GetWidthsAttr().Set(widths)

    @property
    def stage(self) -> Usd.Stage:
        return self.points.GetPrim().GetStage()

    @property
    def points(self) -> UsdGeom.Points:
        return self._points

    @property
    def id(self) -> int:
        return self._point_property.id

    @property
    def pos(self) -> numpy.ndarray:
        return self._point_property.pos

    @property
    def size(self) -> float:
        return self._point_property.size

    @property
    def rgba(self) -> numpy.ndarray:
        return self._point_property.rgba
