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

from pxr import UsdPhysics, Usd, UsdGeom, UsdShade, Sdf, Gf


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
            fixed_base: bool,
            with_physics: bool,
            with_visual: bool,
            with_collision: bool,
            inertia_source: InertiaSource = InertiaSource.FROM_SRC,
            default_rgba: Optional[numpy.ndarray] = None,
            add_xform_for_each_geom: Optional[bool] = None
    ) -> None:
        self._stage = Usd.Stage.Open(file_path)
        xform_root_prims = [prim for prim in self.stage.GetPseudoRoot().GetChildren() if prim.IsA(UsdGeom.Xform)]
        if len(xform_root_prims) > 1:
            print("Multiple root prim found, add a default root prim")
            world_prim = UsdGeom.Xform.Define(self.stage, "/world").GetPrim()
            self.stage.SetDefaultPrim(world_prim)
        default_prim = self.stage.GetDefaultPrim()
        model_name = default_prim.GetName()

        super().__init__(file_path=file_path, config=Configuration(
            model_name=model_name,
            fixed_base=fixed_base,
            with_physics=with_physics,
            with_visual=with_visual,
            with_collision=with_collision,
            default_rgba=default_rgba if default_rgba is not None else numpy.array([0.9, 0.9, 0.9, 1.0]),
            inertia_source=inertia_source
        ))

        self._add_xform_for_each_geom = add_xform_for_each_geom
        if self._add_xform_for_each_geom is None:
            self._add_xform_for_each_geom = len([prim for prim in self.stage.Traverse() if prim.IsA(UsdGeom.Xform)]) == 1
            for prim in [prim for prim in self.stage.Traverse() if prim.IsA(UsdGeom.Gprim)]:
                if any([child_prim.IsA(UsdGeom.Gprim) for child_prim in prim.GetChildren()]):
                    self._add_xform_for_each_geom = True
                    break
            else:
                self._add_xform_for_each_geom = False

        self.parent_map = {}
        self.usd_mesh_path_dict = {}

    def import_model(self, save_file_path: Optional[str] = None) -> str:
        self._world_builder = WorldBuilder(self.tmp_usd_file_path)

        root_prim = self.stage.GetDefaultPrim()
        self.world_builder.add_body(root_prim.GetName())

        if not self.config.with_physics:
            for joint_prim in [joint_prim for joint_prim in self.stage.Traverse() if joint_prim.IsA(UsdPhysics.Joint)]:
                parent_prim = self.stage.GetPrimAtPath(UsdPhysics.Joint(joint_prim).GetBody0Rel().GetTargets()[0])
                child_prim = self.stage.GetPrimAtPath(UsdPhysics.Joint(joint_prim).GetBody1Rel().GetTargets()[0])
                self.parent_map[child_prim] = parent_prim

        self._import_body(body_prim=root_prim)

        if self._config.with_physics:
            self._import_joints()

        self.world_builder.export()

        return self.tmp_usd_file_path if save_file_path is None else self.save_tmp_model(usd_file_path=save_file_path)

    def _import_body(self, body_prim: Usd.Prim) -> None:
        if self.parent_map.get(body_prim) is not None:
            parent_xform_prim = self.parent_map[body_prim]
        elif self.config.with_physics:
            parent_xform_prim = self.stage.GetDefaultPrim()
        else:
            parent_xform_prim = body_prim.GetParent()

        parent_xform_name = parent_xform_prim.GetName()
        body_builder = self.world_builder.add_body(body_name=body_prim.GetName(),
                                                   parent_body_name=parent_xform_name)
        if body_prim.IsA(UsdGeom.Gprim) and body_prim != self.stage.GetDefaultPrim():
            xform_local_transformation, _ = xform_cache.ComputeRelativeTransform(body_prim,
                                                                                 parent_xform_prim)
            body_builder.xform.ClearXformOpOrder()
            body_builder.xform.AddTransformOp().Set(xform_local_transformation)

        for gprim_prim in [gprim_prim for gprim_prim in body_prim.GetChildren()
                           if gprim_prim.IsA(UsdGeom.Gprim)]:
            if self.add_xform_for_each_geom:
                self.parent_map[gprim_prim] = body_prim
                self._import_body(body_prim=gprim_prim)
            else:
                self._import_geom(gprim_prim=gprim_prim,
                                  body_builder=body_builder,
                                  zero_origin=self.add_xform_for_each_geom)

        if body_prim.IsA(UsdGeom.Gprim):
            self._import_geom(gprim_prim=body_prim,
                              body_builder=body_builder,
                              zero_origin=self.add_xform_for_each_geom)

        for child_body_prim in [child_body_prim for child_body_prim in body_prim.GetChildren()
                                if child_body_prim.IsA(UsdGeom.Xform)]:
            self._import_body(body_prim=child_body_prim)

        self._import_inertial(body_prim=body_prim, body_builder=body_builder)

    def _import_geom(self, gprim_prim: Usd.Prim, body_builder: BodyBuilder, zero_origin: bool = False) -> None:
        gprim = UsdGeom.Gprim(gprim_prim)
        geom_is_visible = gprim.GetVisibilityAttr().Get() != UsdGeom.Tokens.invisible
        geom_is_collidable = gprim_prim.HasAPI(UsdPhysics.CollisionAPI)

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
                elif gprim_prim.IsA(UsdGeom.Sphere):
                    sphere = UsdGeom.Sphere(gprim_prim)
                    radius = sphere.GetRadiusAttr().Get()
                    geom_builder.set_attribute(radius=radius)
                elif gprim_prim.IsA(UsdGeom.Cylinder):
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
                                                                 mesh_path=mesh_path)
                if mesh_property.face_vertex_counts.size == 0 or mesh_property.face_vertex_indices.size == 0:
                    # TODO: Fix empty mesh
                    return

                geom_builder = body_builder.add_geom(geom_name=f"{mesh_name}",
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

    def _import_inertial(self, body_prim: Usd.Prim, body_builder: BodyBuilder) -> None:
        if self._config.with_physics and not (self._config.fixed_base and body_prim == self.stage.GetDefaultPrim()):
            if self._config.inertia_source == InertiaSource.FROM_SRC:
                body_prim_api = UsdPhysics.MassAPI(body_prim)
                body_mass = body_prim_api.GetMassAttr().Get()
                body_center_of_mass = body_prim_api.GetCenterOfMassAttr().Get()
                body_center_of_mass = numpy.array([*body_center_of_mass]) \
                    if body_center_of_mass is not None else numpy.zeros(3)
                body_diagonal_inertia = body_prim_api.GetDiagonalInertiaAttr().Get()
                body_diagonal_inertia = numpy.array([*body_diagonal_inertia]) \
                    if body_diagonal_inertia is not None else numpy.zeros(3)
                body_principal_axes = body_prim_api.GetPrincipalAxesAttr().Get()
                body_principal_axes = numpy.array([*body_principal_axes.GetImaginary(), body_principal_axes.GetReal()]) \
                    if body_principal_axes is not None else numpy.array([0.0, 0.0, 0.0, 1.0])
                body_builder.set_inertial(mass=body_mass,
                                          center_of_mass=body_center_of_mass,
                                          diagonal_inertia=body_diagonal_inertia,
                                          principal_axes=body_principal_axes)
            else:
                _, physics_mass_api = body_builder.compute_and_set_inertial(inertia_source=self._config.inertia_source)

    def _import_joints(self) -> None:
        for joint_prim in [joint_prim for joint_prim in self.stage.Traverse() if joint_prim.IsA(UsdPhysics.Joint)]:
            joint = UsdPhysics.Joint(joint_prim)
            joint_name = joint.GetPrim().GetName()
            parent_prim_name = joint.GetBody0Rel().GetTargets()[0].name
            parent_body_builder = self.world_builder.get_body_builder(body_name=parent_prim_name)
            parent_prim = parent_body_builder.xform.GetPrim()
            child_prim_name = joint.GetBody1Rel().GetTargets()[0].name
            child_body_builder = self.world_builder.get_body_builder(body_name=child_prim_name)
            child_prim = child_body_builder.xform.GetPrim()

            joint_axis = "Z"
            if joint_prim.IsA(UsdPhysics.FixedJoint):
                joint_type = JointType.FIXED
            elif joint_prim.IsA(UsdPhysics.RevoluteJoint):
                joint_type = JointType.REVOLUTE
                joint = UsdPhysics.RevoluteJoint(joint)
                joint_axis = joint.GetAxisAttr().Get()
            elif joint_prim.IsA(UsdPhysics.PrismaticJoint):
                joint_type = JointType.PRISMATIC
                joint = UsdPhysics.PrismaticJoint(joint)
                joint_axis = joint.GetAxisAttr().Get()
            elif joint_prim.IsA(UsdPhysics.SphericalJoint):
                joint_type = JointType.SPHERICAL
            else:
                raise ValueError(f"Joint type {joint_prim} not supported.")

            body1_transform = xform_cache.GetLocalToWorldTransform(parent_prim)
            body1_rot = body1_transform.ExtractRotationQuat()

            body2_transform = xform_cache.GetLocalToWorldTransform(child_prim)
            body1_to_body2_transform = body2_transform * body1_transform.GetInverse()
            body1_to_body2_pos = body1_to_body2_transform.ExtractTranslation()

            joint_quat = joint.GetLocalRot1Attr().Get()
            joint_quat = numpy.array([*joint_quat.GetImaginary(), joint_quat.GetReal()])
            joint_pos = body1_rot.GetInverse().Transform(
                Gf.Vec3d(joint.GetLocalPos0Attr().Get()) - body1_to_body2_pos)
            joint_pos = numpy.array([*joint_pos])

            joint_property = JointProperty(joint_parent_prim=parent_prim,
                                           joint_child_prim=child_prim,
                                           joint_pos=joint_pos,
                                           joint_quat=joint_quat,
                                           joint_axis=JointAxis.from_string(joint_axis),
                                           joint_type=joint_type)
            joint_builder = child_body_builder.add_joint(joint_name=joint_name,
                                                         joint_property=joint_property)
            if joint_prim.IsA(UsdPhysics.RevoluteJoint) or joint_prim.IsA(UsdPhysics.PrismaticJoint):
                joint_builder.joint.CreateUpperLimitAttr(joint.GetUpperLimitAttr().Get())
                joint_builder.joint.CreateLowerLimitAttr(joint.GetLowerLimitAttr().Get())

    @property
    def stage(self) -> Usd.Stage:
        return self._stage

    @property
    def add_xform_for_each_geom(self) -> bool:
        return self._add_xform_for_each_geom
