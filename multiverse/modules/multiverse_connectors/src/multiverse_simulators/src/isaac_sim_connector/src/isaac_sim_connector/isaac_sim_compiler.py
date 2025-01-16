#!/usr/bin/env python3

import os
from typing import Dict

import numpy
import shutil

from multiverse_simulator import MultiverseSimulatorCompiler, Robot, Object, multiverse_simulator_compiler_main


class IsaacSimCompiler(MultiverseSimulatorCompiler):
    name: str = "isaac_sim"
    ext: str = "usda"
    world_stage: "Usd.Stage"

    def __init__(self, args):
        super().__init__(args)

    def build_world(self,
                    robots: Dict[str, Robot],
                    objects: Dict[str, Object],
                    references: Dict[str, Dict[str, Any]] = None,
                    multiverse_params: Dict[str, Dict] = None):
        from pxr import Usd, UsdGeom, UsdPhysics, Gf  # Ask NVIDIA for this shitty importing style

        for entity in list(robots.values()) + list(objects.values()):
            file_ext = os.path.splitext(entity.path)[1]
            entity_usd_dir = os.path.dirname(entity.path)
            entity_usd_path = os.path.join(self.save_dir_path, entity.name + f"{file_ext}")
            shutil.copy(entity.path, entity_usd_path)
            entity.path = entity_usd_path
            entity_stage = Usd.Stage.Open(entity_usd_path)

            if "body" in entity.apply:
                body_apply = entity.apply["body"]
                for xform_prim in [prim for prim in entity_stage.Traverse() if
                                   prim.IsA(UsdGeom.Xform) and prim.GetName() in body_apply]:
                    if xform_prim.GetName() == entity.name and not xform_prim.GetParent().IsPseudoRoot():
                        continue
                    xform = UsdGeom.Xform(xform_prim)
                    xform.ClearXformOpOrder()
                    pos = body_apply[xform_prim.GetName()].get("pos", [0, 0, 0])
                    quat = body_apply[xform_prim.GetName()].get("quat", [1, 0, 0, 0])

                    pos = numpy.asarray(pos)
                    quat = numpy.asarray(quat)
                    mat = Gf.Matrix4d()
                    mat.SetTranslateOnly(Gf.Vec3d(*pos))
                    mat.SetRotateOnly(Gf.Quatd(quat[0], Gf.Vec3d(*quat[1:])))

                    xform.AddTransformOp().Set(mat)

            for joint_name, joint_value in entity.joint_state.items():
                for joint_prim in [prim for prim in entity_stage.TraverseAll() if
                                   prim.IsA(UsdPhysics.Joint) and prim.GetName() == joint_name]:
                    if joint_prim.IsA(UsdPhysics.RevoluteJoint) and joint_prim.HasAPI(UsdPhysics.DriveAPI):
                        drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
                        drive_api.GetTargetPositionAttr().Set(numpy.rad2deg(joint_value))

                    elif joint_prim.IsA(UsdPhysics.PrismaticJoint) and joint_prim.HasAPI(UsdPhysics.DriveAPI):
                        drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "linear")
                        drive_api.GetTargetPositionAttr().Set(joint_value)
                    else:
                        print(f"Joint {joint_name} does not have DriveAPI")

            entity_stage.GetRootLayer().Save()

            if file_ext == ".usda":
                with open(entity_usd_path, "r") as f:
                    data = f.read()
                data = data.replace("@./", f"@{entity_usd_dir}/")
                with open(entity_usd_path, "w") as f:
                    f.write(data)

        robots_path = os.path.join(self.save_dir_path, os.path.basename(self.save_file_path).split(".")[0] + "_robots.usda")
        robots_stage = Usd.Stage.CreateNew(robots_path)
        if len(robots) == 1:
            sublayer_paths = [robot.path for robot in robots.values()]
            robots_stage.GetRootLayer().subLayerPaths = sublayer_paths
            robots_stage.SetDefaultPrim(robots_stage.GetPrimAtPath(Usd.Stage.Open(sublayer_paths[0]).GetDefaultPrim().GetPath()))
        robots_stage.Flatten()
        robots_stage.Export(robots_path)

        print(f"Robots: {robots_path}")

        file_ext = os.path.splitext(self.world_path)[1]
        if file_ext == ".usda":
            with open(self.save_file_path, "r") as f:
                data = f.read()
            world_usd_dir = os.path.dirname(self.world_path)
            data = data.replace("@./", f"@{world_usd_dir}/")
            with open(self.save_file_path, "w") as f:
                f.write(data)


if __name__ == "__main__":
    multiverse_simulator_compiler_main(IsaacSimCompiler)
