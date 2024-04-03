#!/usr/bin/env python3

from dataclasses import dataclass
from typing import Optional, Dict

import numpy
import os

from pxr import Usd, UsdGeom, Sdf, UsdShade

cache_mesh_stages = {}


@dataclass(init=False)
class MeshProperty:
    points: numpy.ndarray
    normals: numpy.ndarray
    face_vertex_counts: numpy.ndarray
    face_vertex_indices: numpy.ndarray
    texture_coordinates: Optional[numpy.ndarray]
    geom_subsets: Optional[Dict[str, numpy.ndarray]]
    mesh_file_name: Optional[str]

    def __init__(self,
                 points: numpy.ndarray,
                 normals: numpy.ndarray,
                 face_vertex_counts: numpy.ndarray,
                 face_vertex_indices: numpy.ndarray,
                 texture_coordinates: Optional[numpy.ndarray] = None,
                 geom_subsets: Optional[Dict[str, numpy.ndarray]] = None,
                 mesh_file_name: Optional[str] = None) -> None:
        self._points = points
        self._normals = normals
        self._face_vertex_counts = face_vertex_counts
        self._face_vertex_indices = face_vertex_indices
        self._texture_coordinates = texture_coordinates
        self._geom_subsets = geom_subsets
        self._mesh_file_name = mesh_file_name
        self.check_validity()

    def check_validity(self):
        assert self.points.size != 0
        if not all(face_vertex_count == 3 for face_vertex_count in self.face_vertex_counts):
            raise ValueError("Only triangular meshes are supported.")
        assert self.face_vertex_counts.size * 3 == self.face_vertex_indices.size
        if self.texture_coordinates is not None:
            assert self.texture_coordinates.size == self.face_vertex_indices.size * 2

    @classmethod
    def from_mesh_file_path(cls,
                            mesh_file_path: str,
                            mesh_path: Sdf.Path,
                            texture_coordinate_name: str = "st") -> "MeshProperty":
        if mesh_file_path in cache_mesh_stages:
            mesh_stage = cache_mesh_stages[mesh_file_path]
        else:
            mesh_stage = Usd.Stage.Open(mesh_file_path)
            cache_mesh_stages[mesh_file_path] = mesh_stage
        mesh_prim = mesh_stage.GetPrimAtPath(mesh_path) if not mesh_path.isEmpty else mesh_stage.GetDefaultPrim()
        if not mesh_prim.IsA(UsdGeom.Mesh):
            mesh_path = mesh_prim.GetPath().AppendChild(mesh_prim.GetName())
            if mesh_stage.GetPrimAtPath(mesh_path).IsValid():
                mesh_prim = mesh_stage.GetPrimAtPath(mesh_path)
            else:
                print(f"Prim {mesh_prim} from {mesh_file_path} is not a mesh, try to get its child.")
                mesh_prims = mesh_prim.GetChildren()
                if len(mesh_prims) == 0:
                    raise ValueError(f"{mesh_prim} has no child.")
                mesh_prim = mesh_prims[0]
                if not mesh_prim.IsA(UsdGeom.Mesh):
                    raise TypeError(f"Prim {mesh_prim} is not a mesh")

        mesh_file_name = os.path.splitext(os.path.basename(mesh_file_path))[0]
        return cls.from_prim(mesh_prim, texture_coordinate_name, mesh_file_name)

    @classmethod
    def from_prim(cls, mesh_prim: Usd.Prim, texture_coordinate_name: str = "st",
                  mesh_file_name: Optional[str] = None) -> "MeshProperty":
        mesh = UsdGeom.Mesh(mesh_prim)
        prim_vars_api = UsdGeom.PrimvarsAPI(mesh_prim)
        if prim_vars_api.HasPrimvar(texture_coordinate_name):
            texture_coordinates = prim_vars_api.GetPrimvar(texture_coordinate_name).Get()
            texture_coordinates = numpy.array(texture_coordinates, dtype=numpy.float32)
        else:
            texture_coordinates = None

        geom_subsets = {}
        for geom_subset_prim in [prim for prim in mesh_prim.GetChildren() if
                                 prim.IsA(UsdGeom.Subset)]:
            if not geom_subset_prim.HasAPI(UsdShade.MaterialBindingAPI):
                continue
            geom_subset_name = geom_subset_prim.GetName()
            material_binding_api = UsdShade.MaterialBindingAPI(geom_subset_prim)
            material_paths = material_binding_api.GetDirectBindingRel().GetTargets()
            if len(material_paths) > 1:
                raise NotImplementedError(f"GeomSubset {geom_subset_name} has more than one material.")
            material_name = material_paths[0].name
            geom_subset = UsdGeom.Subset(geom_subset_prim)
            if geom_subset.GetElementTypeAttr().Get() != UsdGeom.Tokens.face:
                raise ValueError(
                    f"Only face subset is supported, but {geom_subset} is {geom_subset.GetElementTypeAttr().Get()}")

            geom_subset_indices = numpy.array(geom_subset.GetIndicesAttr().Get())
            geom_subsets[material_name] = geom_subset_indices

        return cls(points=numpy.array(mesh.GetPointsAttr().Get()),
                   normals=numpy.array(mesh.GetNormalsAttr().Get()),
                   face_vertex_counts=numpy.array(mesh.GetFaceVertexCountsAttr().Get()),
                   face_vertex_indices=numpy.array(mesh.GetFaceVertexIndicesAttr().Get()),
                   texture_coordinates=texture_coordinates,
                   geom_subsets=geom_subsets,
                   mesh_file_name=mesh_file_name)

    @property
    def points(self):
        return self._points

    @property
    def normals(self):
        return self._normals

    @property
    def face_vertex_counts(self):
        return self._face_vertex_counts

    @property
    def face_vertex_indices(self):
        return self._face_vertex_indices

    @property
    def texture_coordinates(self):
        return self._texture_coordinates

    @property
    def geom_subsets(self):
        return self._geom_subsets

    @property
    def mesh_file_name(self):
        return self._mesh_file_name


class MeshBuilder:
    file_path: str
    stage: Usd.Stage
    mesh: UsdGeom.Mesh

    def __init__(self, stage, mesh_property: MeshProperty) -> None:
        self._mesh_property = mesh_property
        self._mesh = UsdGeom.Mesh(stage.GetDefaultPrim())

    def build(self) -> UsdGeom.Mesh:
        mesh = self.mesh
        mesh.CreatePointsAttr(self.points)
        mesh.CreateNormalsAttr(self.normals)
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        mesh.CreateFaceVertexCountsAttr(self.face_vertex_counts)
        mesh.CreateFaceVertexIndicesAttr(self.face_vertex_indices)
        if self.texture_coordinates is not None:
            prim_vars_api = UsdGeom.PrimvarsAPI(mesh)
            texture_coordinates = prim_vars_api.CreatePrimvar("st",
                                                              Sdf.ValueTypeNames.TexCoord2fArray,
                                                              UsdGeom.Tokens.faceVarying)
            texture_coordinates.Set(self.texture_coordinates)

        # for material_name, geom_subset_indices in self.geom_subsets.items():
        #     geom_subset = UsdGeom.Subset.Define(self.stage, mesh.GetPath().AppendChild(material_name))
        #     geom_subset.CreateElementTypeAttr(UsdGeom.Tokens.face)
        #     geom_subset.CreateIndicesAttr().Set(geom_subset_indices)
        #     UsdShade.MaterialBindingAPI.Apply(geom_subset.GetPrim())

        self.stage.GetRootLayer().Save()

        return self.mesh

    @property
    def mesh(self):
        return self._mesh

    @property
    def stage(self):
        return self.mesh.GetPrim().GetStage()

    @property
    def file_path(self):
        return self.stage.GetRootLayer().realPath

    @property
    def points(self):
        return self._mesh_property.points

    @property
    def normals(self):
        return self._mesh_property.normals

    @property
    def face_vertex_counts(self):
        return self._mesh_property.face_vertex_counts

    @property
    def face_vertex_indices(self):
        return self._mesh_property.face_vertex_indices

    @property
    def texture_coordinates(self):
        return self._mesh_property.texture_coordinates

    @property
    def geom_subsets(self):
        return self._mesh_property.geom_subsets
