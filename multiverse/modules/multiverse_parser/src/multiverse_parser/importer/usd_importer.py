#!/usr/bin/env python3.10

import os
from pxr import Usd, UsdGeom, UsdPhysics
from multiverse_parser import WorldBuilder, GeomType, JointType
from multiverse_parser.factory.body_builder import body_dict
from multiverse_parser.utils import xform_cache
from multiverse_parser.utils import import_usd, export_usd
from multiverse_parser.utils import diagonalize_inertia, clear_meshes, modify_name


class UsdImporter:
    def __init__(
        self,
        usd_file_path: str,
        with_physics: bool,
        with_visual: bool,
        with_collision: bool,
    ) -> None:
        self.usd_file_path = usd_file_path
        self.with_physics = with_physics

        self.world_builder = WorldBuilder()

        self.stage = Usd.Stage.Open(self.usd_file_path)
        root_prim = self.stage.GetDefaultPrim()
        self.world_builder.add_body(root_prim.GetName())

        self.parent_map = {}

        for joint_prim in [joint_prim for joint_prim in self.stage.Traverse() if joint_prim.IsA(UsdPhysics.Joint)]:
            parent_prim = self.stage.GetPrimAtPath(UsdPhysics.Joint(joint_prim).GetBody0Rel().GetTargets()[0])
            child_prim = self.stage.GetPrimAtPath(UsdPhysics.Joint(joint_prim).GetBody1Rel().GetTargets()[0])
            self.parent_map[child_prim.GetName()] = parent_prim.GetName()

        self.build_body(root_prim)

    def build_body(self, parent_prim):
        for xform_prim in [xform_prim for xform_prim in parent_prim.GetChildren() if xform_prim.IsA(UsdGeom.Xform)]:
            child_geom_prims = [geom_prim for geom_prim in xform_prim.GetChildren() if geom_prim.IsA(UsdGeom.Gprim)]
            if len(child_geom_prims) == 0:
                if self.with_physics or self.parent_map.get(xform_prim.GetName()) is None:
                    body_builder = self.world_builder.add_body(
                        body_name=xform_prim.GetName(),
                        parent_body_name=parent_prim.GetName(),
                    )
                    xform_local_transformation, _ = xform_cache.ComputeRelativeTransform(xform_prim, parent_prim)
                    body_builder.xform.AddTransformOp().Set(xform_local_transformation)

                elif self.parent_map.get(xform_prim.GetName()) is not None:
                    body_builder = self.world_builder.add_body(
                        body_name=xform_prim.GetName(),
                        parent_body_name=self.parent_map[xform_prim.GetName()],
                    )
                    xform_local_transformation = UsdGeom.Xform(xform_prim).GetLocalTransformation()
                    body_builder.xform.AddTransformOp().Set(xform_local_transformation)

                body_builder = self.build_body(xform_prim)

            else:
                if len(child_geom_prims) > 1:
                    print(f"More than one child geom for {xform_prim.GetName()} exist, take the first one.")
                geom_prim = child_geom_prims[0]

                if geom_prim.IsA(UsdGeom.Cube):
                    geom_type = GeomType.CUBE
                elif geom_prim.IsA(UsdGeom.Sphere):
                    geom_type = GeomType.SPHERE
                elif geom_prim.IsA(UsdGeom.Cylinder):
                    geom_type = GeomType.CYLINDER
                elif geom_prim.IsA(UsdGeom.Mesh):
                    geom_type = GeomType.MESH
                else:
                    print(f"Geom type {geom_prim} not supported.")
                    continue

                is_visual = not geom_prim.HasAPI(UsdPhysics.CollisionAPI)

                body_builder = body_dict[parent_prim.GetName()]
                body_has_mass = body_builder.xform.GetPrim().HasAPI(UsdPhysics.MassAPI)
            
                if body_has_mass and self.with_physics:
                    physics_mass_api = UsdPhysics.MassAPI(body_builder.xform)
                    mass = physics_mass_api.GetMassAttr().Get()
                    com = physics_mass_api.GetCenterOfMassAttr().Get()
                    diagonal_inertia = physics_mass_api.GetDiagonalInertiaAttr().Get()

                    body_builder.set_inertial(mass=mass, com=com, diagonal_inertia=diagonal_inertia)

                geom_builder = body_builder.add_geom(
                    geom_name=xform_prim.GetName(),
                    geom_type=geom_type,
                    is_visual=is_visual,
                )
                geom_local_transformation = UsdGeom.Xform(xform_prim).GetLocalTransformation()
                geom_builder.xform.AddTransformOp().Set(geom_local_transformation)

                if geom_type == GeomType.MESH:
                    from multiverse_parser.factory import TMP_USD_MESH_PATH

                    for prim_spec in geom_prim.GetPrimStack():
                        print(prim_spec)
                        print(dir(prim_spec))
                        print(prim_spec.GetInfo())
                    break

                    if len(geom_prim.GetPrim().GetPrimStack()[0].referenceList.prependedItems) > 0:
                        usd_file_path = geom_prim.GetPrim().GetPrimStack()[0].referenceList.prependedItems[0].assetPath
                        if not os.path.abspath(usd_file_path):
                            usd_file_path = os.path.join(os.path.dirname(self.usd_file_path), usd_file_path)

                        clear_meshes()

                        import_usd(in_usd=usd_file_path)

                        export_usd(
                            out_usd=os.path.join(
                                TMP_USD_MESH_PATH,
                                "visual" if is_visual else "collision",
                                geom_prim.GetName() + ".usda",
                            )
                        )

                        mesh_builder = geom_builder.add_mesh(mesh_name=geom_prim.GetName())
                        mesh_builder.save()

                # if self.with_physics:
                #     geom_builder.enable_collision()
                #     if not body_has_mass:
                #         if geom_builder.xform.GetPrim().HasAPI(UsdPhysics.MassAPI):
                #             physics_mass_api = UsdPhysics.MassAPI(geom_builder.xform)
                #             mass = physics_mass_api.GetMassAttr().Get()
                #             com = physics_mass_api.GetCenterOfMassAttr().Get()
                #             diagonal_inertia = physics_mass_api.GetDiagonalInertiaAttr().Get()
                #             principal_axes = physics_mass_api.GetPrincipalAxesAttr().Get()

                #             geom_builder.set_inertial(
                #                 mass=mass,
                #                 com=com,
                #                 diagonal_inertia=diagonal_inertia,
                #                 principal_axes=principal_axes,
                #             )
                #         else:
                #             geom_builder.compute_inertial()

                geom_builder.compute_extent()