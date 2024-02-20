#!/usr/bin/env python3.10

# import os
# import shutil
# from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf
# from multiverse_parser import WorldBuilder, GeomType, JointType
# from multiverse_parser.factory.body_builder import body_dict
# from multiverse_parser.utils import xform_cache
# from multiverse_parser.utils import import_usd, export_usd
# from multiverse_parser.utils import clear_meshes, copy_prim

import os
from math import degrees
from typing import Optional, Union, Dict

import numpy
from scipy.spatial.transform import Rotation
from urdf_parser_py import urdf

from ..factory import Factory, Configuration, InertiaSource
from ..factory import (WorldBuilder,
                       BodyBuilder,
                       JointBuilder, JointAxis, JointType, JointProperty,
                       GeomType, GeomProperty,
                       MeshProperty,
                       MaterialProperty)
from ..utils import xform_cache, shift_inertia_tensor, diagonalize_inertia

from pxr import UsdUrdf, Gf, UsdPhysics, Usd, UsdGeom, UsdShade, Sdf


def get_usd_file_path(gprim_prim: Usd.Prim) -> (str, Sdf.Path):
    prepended_items = gprim_prim.GetPrimStack()[0].referenceList.prependedItems
    if len(prepended_items) == 0:
        return gprim_prim.GetStage().GetRootLayer().realPath, gprim_prim.GetPath()
    elif len(prepended_items) == 1:
        prepended_item = prepended_items[0]
        file_abspath = prepended_item.assetPath
        prim_path = prepended_item.primPath
        if not os.path.isabs(file_abspath):
            usd_file_path = gprim_prim.GetStage().GetRootLayer().realPath
            file_abspath = os.path.join(os.path.dirname(usd_file_path), file_abspath)
        return file_abspath, prim_path
    else:
        raise ValueError(f"Multiple prepended items found for {gprim_prim}.")


class UsdImporter(Factory):
    stage: Usd.Stage
    parent_map: Dict[Usd.Prim, Usd.Prim]
    usd_mesh_path_dict: Dict[str, str]

    def __init__(
            self,
            usd_file_path: str,
            with_physics: bool,
            with_visual: bool,
            with_collision: bool,
            inertia_source: InertiaSource = InertiaSource.FROM_SRC,
            default_rgba: Optional[numpy.ndarray] = None,
    ) -> None:
        self._stage = Usd.Stage.Open(usd_file_path)
        xform_root_prims = [prim for prim in self.stage.GetPseudoRoot().GetChildren() if prim.IsA(UsdGeom.Xform)]
        if len(xform_root_prims) > 1:
            print("Multiple root prim found, add a default root prim")
            world_prim = UsdGeom.Xform.Define(self.stage, "/world").GetPrim()
            self.stage.SetDefaultPrim(world_prim)
        default_prim = self.stage.GetDefaultPrim()
        model_name = default_prim.GetName()

        super().__init__(file_path=usd_file_path, config=Configuration(
            model_name=model_name,
            with_physics=with_physics,
            with_visual=with_visual,
            with_collision=with_collision,
            default_rgba=default_rgba if default_rgba is not None else numpy.array([1.0, 0.0, 0.0, 0.0]),
            inertia_source=inertia_source
        ))

        self.parent_map = {}
        self.usd_mesh_path_dict = {}

    def import_model(self, save_file_path: Optional[str] = None) -> str:
        self._world_builder = WorldBuilder(self.tmp_usd_file_path)

        root_prim = self.stage.GetDefaultPrim()
        self.world_builder.add_body(root_prim.GetName())

        for joint_prim in [joint_prim for joint_prim in self.stage.Traverse() if joint_prim.IsA(UsdPhysics.Joint)]:
            parent_prim = self.stage.GetPrimAtPath(UsdPhysics.Joint(joint_prim).GetBody0Rel().GetTargets()[0])
            child_prim = self.stage.GetPrimAtPath(UsdPhysics.Joint(joint_prim).GetBody1Rel().GetTargets()[0])
            self.parent_map[child_prim] = parent_prim

        for xform_prim in [xform_prim for xform_prim in self.stage.Traverse() if xform_prim.IsA(UsdGeom.Xform)]:
            self._import_body(xform_prim=xform_prim)

        self.world_builder.export()

        return self.tmp_usd_file_path if save_file_path is None else self.save_tmp_model(usd_file_path=save_file_path)

        # if self.config.with_physics:
        #     self._import_joint(root_prim)

    def _import_body(self, xform_prim: Usd.Prim) -> None:
        if self.config.with_physics or self.parent_map.get(xform_prim) is None:
            body_builder = self.world_builder.add_body(body_name=xform_prim.GetName(),
                                                       parent_body_name=self.config.model_name)
            body_builder.enable_rigid_body()
            xform_local_transformation, _ = xform_cache.ComputeRelativeTransform(xform_prim,
                                                                                 self.stage.GetDefaultPrim())
            body_builder.xform.ClearXformOpOrder()
            body_builder.xform.AddTransformOp().Set(xform_local_transformation)

        elif self.parent_map.get(xform_prim) is not None:
            body_builder = self.world_builder.add_body(
                body_name=xform_prim.GetName(),
                parent_body_name=xform_prim.GetName(),
            )
            xform_local_transformation, _ = xform_cache.ComputeRelativeTransform(xform_prim,
                                                                                 self.parent_map[xform_prim])
            body_builder.xform.AddTransformOp().Set(xform_local_transformation)

        else:
            return

        for gprim_prim in [gprim_prim for gprim_prim in xform_prim.GetChildren() if gprim_prim.IsA(UsdGeom.Gprim)]:
            self._import_geom(gprim_prim=gprim_prim, body_builder=body_builder)

        for child_xform_prim in [child_xform_prim for child_xform_prim in xform_prim.GetChildren() if
                                 child_xform_prim.IsA(UsdGeom.Xform)]:
            self._import_body(xform_prim=child_xform_prim)

    def _import_geom(self, gprim_prim: UsdGeom.Gprim, body_builder: BodyBuilder) -> None:
        geom_is_visible = not gprim_prim.HasAPI(UsdPhysics.CollisionAPI)
        geom_is_collidable = not geom_is_visible

        if (geom_is_visible and self.config.with_visual) or (not geom_is_visible and self.config.with_collision):
            if gprim_prim.IsA(UsdGeom.Cube):
                geom_type = GeomType.CUBE
            elif gprim_prim.IsA(UsdGeom.Sphere):
                geom_type = GeomType.SPHERE
            elif gprim_prim.IsA(UsdGeom.Cylinder):
                geom_type = GeomType.CYLINDER
            elif gprim_prim.IsA(UsdGeom.Mesh):
                geom_type = GeomType.MESH
            else:
                raise ValueError(f"Geom type {gprim_prim} not supported.")

            geom_rgba = self.config.default_rgba
            geom_density = 1000.0
            geom_property = GeomProperty(geom_type=geom_type,
                                         is_visible=geom_is_visible,
                                         is_collidable=geom_is_collidable,
                                         rgba=geom_rgba,
                                         density=geom_density)

            geom_name = gprim_prim.GetName()
            gprim = UsdGeom.Gprim(gprim_prim)
            transformation = gprim.GetLocalTransformation().RemoveScaleShear()
            geom_pos = transformation.ExtractTranslation()
            geom_pos = numpy.array([*geom_pos])
            geom_quat = transformation.ExtractRotationQuat()
            geom_quat = numpy.array([*geom_quat.GetImaginary(), geom_quat.GetReal()])
            geom_scale = numpy.array([transformation.GetRow(i).GetLength() for i in range(3)])

            if not gprim_prim.IsA(UsdGeom.Mesh):
                geom_builder = body_builder.add_geom(geom_name=geom_name, geom_property=geom_property)
                geom_builder.set_transform(pos=geom_pos, quat=geom_quat, scale=geom_scale)
                if gprim_prim.IsA(UsdGeom.Cube):
                    pass
                elif gprim_prim.IsA(UsdGeom.SPHERE):
                    sphere = UsdGeom.Sphere(gprim_prim)
                    radius = sphere.GetRadiusAttr().Get()
                    geom_builder.set_attribute(radius=radius)
                elif gprim_prim.IsA(UsdGeom.CYLINDER):
                    cylinder = UsdGeom.Cylinder(gprim_prim)
                    radius = cylinder.GetRadiusAttr().Get()
                    height = cylinder.GetHeightAttr().Get()
                    geom_builder.set_attribute(radius=radius, height=height)
                else:
                    raise ValueError(f"Geom type {gprim_prim} not implemented.")

                geom_builder.build()

            else:
                usd_file_path, mesh_path = get_usd_file_path(gprim_prim=gprim_prim)
                tmp_usd_mesh_file_path, tmp_origin_mesh_file_path = self.import_mesh(
                    mesh_file_path=usd_file_path, merge_mesh=False)

                mesh_name = gprim_prim.GetName()
                mesh_property = MeshProperty.from_mesh_file_path(mesh_file_path=tmp_usd_mesh_file_path,
                                                                 mesh_path=mesh_path,
                                                                 texture_coordinate_name="UVMap")
                if mesh_property.face_vertex_counts.size == 0 or mesh_property.face_vertex_indices.size == 0:
                    # TODO: Fix empty mesh
                    return

                geom_builder = body_builder.add_geom(geom_name=f"{geom_name}_{mesh_name}",
                                                     geom_property=geom_property)
                geom_builder.add_mesh(mesh_name=mesh_name,
                                      mesh_property=mesh_property)
                geom_builder.set_transform(pos=geom_pos, quat=geom_quat, scale=geom_scale)
                geom_builder.build()

    @property
    def stage(self) -> Usd.Stage:
        return self._stage
