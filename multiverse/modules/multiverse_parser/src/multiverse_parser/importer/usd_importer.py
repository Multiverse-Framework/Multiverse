#!/usr/bin/env python3.10

from pxr import Usd, UsdGeom
from multiverse_parser import WorldBuilder, GeomType, JointType
from multiverse_parser.factory.body_builder import body_dict


class UsdImporter:
    def __init__(
        self,
        usd_file_path: str,
        with_physics: bool,
        with_visual: bool,
        with_collision: bool,
    ) -> None:
        self.usd_file_path = usd_file_path
        self.world_builder = WorldBuilder()

        self.stage = Usd.Stage.Open(self.usd_file_path)
        root_prim = self.stage.GetDefaultPrim()
        self.world_builder.add_body(root_prim.GetName())

        self.build_body(root_prim)
        # for prim in [xform_prim for xform_prim in self.stage.Traverse() if xform_prim.IsA(UsdGeom.Xform)]:
        #     if prim.GetParent().GetName() == "/":
        #         body_builder = self.world_builder.add_body(prim.GetName())
        #     elif not any([geom_prim.IsA(UsdGeom.Gprim) for geom_prim in prim.GetChildren()]):
        #         print(prim.GetName(), prim.GetParent().GetName())
        #         body_builder = self.world_builder.add_body(prim.GetName(), prim.GetParent().GetName())
        #     body_builder = body_dict[prim.GetParent().GetName()]
        # for child_prim in [geom_prim for geom_prim in prim.GetChildren() if geom_prim.IsA(UsdGeom.Gprim)]:
        #     if child_prim.IsA(UsdGeom.Cube):
        #         geom_type = GeomType.CUBE
        #     elif child_prim.IsA(UsdGeom.Sphere):
        #         geom_type = GeomType.SPHERE
        #     elif child_prim.IsA(UsdGeom.Cylinder):
        #         geom_type = GeomType.CYLINDER
        #     elif child_prim.IsA(UsdGeom.Mesh):
        #         geom_type = GeomType.MESH
        #     else:
        #         print(f"Geom type of {child_prim} not supported.")
        #         continue

        #     body_builder.add_geom(geom_name=child_prim.GetName(), geom_type=geom_type)

    def build_body(self, parent_prim):
        for xform_prim in [xform_prim for xform_prim in parent_prim.GetChildren() if xform_prim.IsA(UsdGeom.Xform)]:
            body_builder = self.world_builder.add_body(xform_prim.GetName(), parent_prim.GetName())
            self.build_body(xform_prim)
        for geom_prim in [geom_prim for geom_prim in parent_prim.GetChildren() if geom_prim.IsA(UsdGeom.Gprim)]:
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

            body_builder = body_dict[parent_prim.GetName()]
            geom_builder = body_builder.add_geom(geom_name=geom_prim.GetName(), geom_type=geom_type)

            if geom_type == GeomType.MESH:
                for mesh_prim in [mesh_prim for mesh_prim in geom_prim.GetChildren() if mesh_prim.IsA(UsdGeom.Mesh)]:
                    geom_builder.add_mesh(mesh_name=mesh_prim.GetName(), is_visual=True)
