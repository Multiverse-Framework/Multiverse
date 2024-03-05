#!/usr/bin/env python3.10

import os
from typing import Optional, Dict

import numpy

from ..factory import Factory, Configuration, InertiaSource
from ..factory import (WorldBuilder,
                       BodyBuilder,
                       JointBuilder, JointAxis, JointType, JointProperty,
                       GeomType, GeomProperty,
                       MeshProperty,
                       MaterialProperty)
from ..utils import xform_cache

from pxr import UsdPhysics, Usd, UsdGeom, UsdShade, Sdf


def get_usd_mesh_file_path(gprim_prim: Usd.Prim) -> (str, Sdf.Path):
    prepended_items = gprim_prim.GetPrimStack()[0].referenceList.prependedItems
    if len(prepended_items) == 0:
        # raise NotImplementedError(f"No prepended item found for {gprim_prim}.")
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
            file_path: str,
            with_physics: bool,
            with_visual: bool,
            with_collision: bool,
            inertia_source: InertiaSource = InertiaSource.FROM_SRC,
            default_rgba: Optional[numpy.ndarray] = None,
            add_xform_for_each_geom: bool = False
    ) -> None:
        self._stage = Usd.Stage.Open(file_path)
        self._add_xform_for_each_geom = add_xform_for_each_geom
        xform_root_prims = [prim for prim in self.stage.GetPseudoRoot().GetChildren() if prim.IsA(UsdGeom.Xform)]
        if len(xform_root_prims) > 1:
            print("Multiple root prim found, add a default root prim")
            world_prim = UsdGeom.Xform.Define(self.stage, "/world").GetPrim()
            self.stage.SetDefaultPrim(world_prim)
        default_prim = self.stage.GetDefaultPrim()
        model_name = default_prim.GetName()

        super().__init__(file_path=file_path, config=Configuration(
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

        self._import_body(body_prim=root_prim)

        self.world_builder.export()

        return self.tmp_usd_file_path if save_file_path is None else self.save_tmp_model(usd_file_path=save_file_path)

        # if self.config.with_physics:
        #     self._import_joint(root_prim)

    def _import_body(self, body_prim: Usd.Prim) -> None:
        if self.config.with_physics or self.parent_map.get(body_prim) is None:
            parent_xform_prim = self.stage.GetDefaultPrim()
        elif self.parent_map.get(body_prim) is not None:
            parent_xform_prim = self.parent_map[body_prim]
        else:
            raise ValueError(f"Parent of {body_prim} not found.")

        parent_xform_name = parent_xform_prim.GetName()
        body_builder = self.world_builder.add_body(body_name=body_prim.GetName(),
                                                   parent_body_name=parent_xform_name)
        if body_prim.IsA(UsdGeom.Xform) and body_prim != self.stage.GetDefaultPrim():
            xform_local_transformation, _ = xform_cache.ComputeRelativeTransform(body_prim,
                                                                                 parent_xform_prim)
            body_builder.xform.ClearXformOpOrder()
            body_builder.xform.AddTransformOp().Set(xform_local_transformation)

        for gprim_prim in [gprim_prim for gprim_prim in body_prim.GetChildren() if gprim_prim.IsA(UsdGeom.Gprim)]:
            if self.add_xform_for_each_geom:
                xform_local_transformation, _ = xform_cache.ComputeRelativeTransform(gprim_prim, body_prim)
                body_builder = self.world_builder.add_body(body_name=gprim_prim.GetName(),
                                                           parent_body_name=body_prim.GetName())
                body_builder.xform.AddTransformOp().Set(xform_local_transformation)

            self._import_geom(gprim_prim=gprim_prim,
                              body_builder=body_builder,
                              zero_origin=self.add_xform_for_each_geom)

            if any([child_body_prim.IsA(UsdGeom.Gprim) for child_body_prim in gprim_prim.GetChildren()]):
                self._import_body(body_prim=gprim_prim)

        body_builder.compute_and_set_inertial(inertia_source=self.config.inertia_source)

        for child_body_prim in [child_body_prim for child_body_prim in body_prim.GetChildren()
                                if child_body_prim.IsA(UsdGeom.Xform) or child_body_prim.GetTypeName() == ""]:
            self._import_body(body_prim=child_body_prim)

    def _import_geom(self, gprim_prim: UsdGeom.Gprim, body_builder: BodyBuilder, zero_origin: bool = False) -> None:
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
            transformation = gprim.GetLocalTransformation()
            if zero_origin:
                geom_pos = numpy.zeros(3)
                geom_quat = numpy.array([0.0, 0.0, 0.0, 1.0])
            else:
                geom_pos = transformation.ExtractTranslation()
                geom_pos = numpy.array([*geom_pos])
                geom_quat = transformation.ExtractRotationQuat()
                geom_quat = numpy.array([*geom_quat.GetImaginary(), geom_quat.GetReal()])
            geom_scale = numpy.array([transformation.GetRow(i).GetLength() for i in range(3)])

            if not gprim_prim.IsA(UsdGeom.Mesh):
                geom_builder = body_builder.add_geom(geom_name=geom_name, geom_property=geom_property)
                geom_builder.build()
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

            else:
                mesh_file_path, mesh_path = get_usd_mesh_file_path(gprim_prim=gprim_prim)
                tmp_mesh_file_path, tmp_origin_mesh_file_path = self.import_mesh(mesh_file_path=mesh_file_path,
                                                                                 merge_mesh=False)

                mesh_name = gprim_prim.GetName()
                mesh_property = MeshProperty.from_mesh_file_path(mesh_file_path=tmp_mesh_file_path,
                                                                 mesh_path=mesh_path,
                                                                 texture_coordinate_name="UVMap")
                if mesh_property.face_vertex_counts.size == 0 or mesh_property.face_vertex_indices.size == 0:
                    # TODO: Fix empty mesh
                    return

                geom_builder = body_builder.add_geom(geom_name=f"{geom_name}_{mesh_name}",
                                                     geom_property=geom_property)
                geom_builder.build()
                geom_builder.add_mesh(mesh_name=mesh_name,
                                      mesh_property=mesh_property)
                geom_builder.set_transform(pos=geom_pos, quat=geom_quat, scale=geom_scale)

                if geom_is_visible:
                    if gprim_prim.HasAPI(UsdShade.MaterialBindingAPI):
                        material_binding_api = UsdShade.MaterialBindingAPI(gprim_prim)
                        material_paths = material_binding_api.GetDirectBindingRel().GetTargets()
                        if len(material_paths) > 1:
                            raise NotImplementedError(f"Mesh {geom_name} has more than one material.")
                        if len(material_paths) == 0:
                            raise ValueError(f"Mesh {geom_name} has no material.")
                        material_prim = self.stage.GetPrimAtPath(material_paths[0])
                        if len(material_prim.GetPrimStack()) >= 2:
                            material_prim_stack = material_prim.GetPrimStack()[1]
                            material_file_path = material_prim_stack.layer.realPath
                            material_path = material_prim_stack.path
                        else:
                            material_file_path = material_prim.GetStage().GetRootLayer().realPath
                            material_path = material_prim.GetPath()
                        material_property = MaterialProperty.from_material_file_path(
                            material_file_path=material_file_path,
                            material_path=material_path)
                        if material_property.opacity == 0.0:
                            print(f"Opacity of {material_path} is 0.0. Set to 1.0.")
                            material_property._opacity = 1.0
                        geom_builder.add_material(material_name=material_path.name,
                                                  material_property=material_property)
                    for subset_prim in [subset_prim for subset_prim in gprim_prim.GetChildren() if
                                        subset_prim.IsA(UsdGeom.Subset)]:
                        subset_name = subset_prim.GetName()
                        material_binding_api = UsdShade.MaterialBindingAPI(subset_prim)
                        material_paths = material_binding_api.GetDirectBindingRel().GetTargets()
                        if len(material_paths) > 1:
                            raise NotImplementedError(f"Subset {subset_name} has more than one material.")
                        if len(material_paths) == 0:
                            raise ValueError(f"Subset {subset_name} has no material.")
                        material_prim = self.stage.GetPrimAtPath(material_paths[0])
                        if len(material_prim.GetPrimStack()) >= 2:
                            material_prim_stack = material_prim.GetPrimStack()[1]
                            material_file_path = material_prim_stack.layer.realPath
                            material_path = material_prim_stack.path
                        else:
                            material_file_path = material_prim.GetStage().GetRootLayer().realPath
                            material_path = material_prim.GetPath()
                        material_property = MaterialProperty.from_material_file_path(
                            material_file_path=material_file_path,
                            material_path=material_path)
                        if material_property.opacity == 0.0:
                            print(f"Opacity of {material_path} is 0.0. Set to 1.0.")
                            material_property._opacity = 1.0
                        geom_builder.add_material(material_name=material_path.name,
                                                  material_property=material_property,
                                                  subset=UsdGeom.Subset(subset_prim))

    @property
    def stage(self) -> Usd.Stage:
        return self._stage

    @property
    def add_xform_for_each_geom(self) -> bool:
        return self._add_xform_for_each_geom
