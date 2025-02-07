#!/usr/bin/env python3

import os
from typing import Dict, Any

import numpy
import shutil
import json

from multiverse_simulator import MultiverseSimulatorCompiler, Robot, Object, multiverse_simulator_compiler_main


class IsaacSimCompiler(MultiverseSimulatorCompiler):
    name: str = "isaac_sim"
    ext: str = "usda"

    def __init__(self, args):
        super().__init__(args)

    def build_world(self,
                    robots: Dict[str, Robot],
                    objects: Dict[str, Object],
                    references: Dict[str, Dict[str, Any]] = None,
                    multiverse_params: Dict[str, Dict] = None):
        from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf  # Ask NVIDIA for this shitty importing style

        for entity in list(robots.values()) + list(objects.values()):
            file_ext = os.path.splitext(entity.path)[1]
            entity_usd_dir = os.path.dirname(entity.path)
            entity_usd_path = os.path.join(self.save_dir_path, entity.name + f"{file_ext}")
            shutil.copy(entity.path, entity_usd_path)
            entity.path = entity_usd_path

            if file_ext == ".usda":
                with open(entity_usd_path, "r") as f:
                    data = f.read()
                data = data.replace("@./", f"@{entity_usd_dir}/")
                with open(entity_usd_path, "w") as f:
                    f.write(data)
            entity_stage = Usd.Stage.Open(entity_usd_path)

            if "body" in entity.apply:
                body_apply = entity.apply["body"]
                for xform_prim in [prim for prim in entity_stage.Traverse() if
                                   prim.IsA(UsdGeom.Xform) and prim.GetName() in body_apply]:
                    if xform_prim.GetName() == entity.name and not xform_prim.GetParent().IsPseudoRoot():
                        continue
                    xform = UsdGeom.Xform(xform_prim)
                    pose = xform.GetLocalTransformation()
                    pos = pose.ExtractTranslation()
                    pos = [*pos]
                    quat = pose.ExtractRotationQuat()
                    quat = [*quat.GetImaginary(), quat.GetReal()]
                    xform.ClearXformOpOrder()
                    pos = body_apply[xform_prim.GetName()].get("pos", pos)
                    quat = body_apply[xform_prim.GetName()].get("quat", quat)

                    pos = numpy.asfarray(pos)
                    quat = numpy.asfarray(quat)
                    mat = Gf.Matrix4d()
                    mat.SetTranslateOnly(Gf.Vec3d(*pos))
                    mat.SetRotateOnly(Gf.Quatd(quat[0], Gf.Vec3d(*quat[1:])))

                    xform.AddTransformOp().Set(mat)

            for joint_name, joint_value in entity.joint_state.items():
                for joint_prim in [prim for prim in entity_stage.TraverseAll() if
                                   prim.IsA(UsdPhysics.Joint) and prim.GetName() == joint_name]:
                    if joint_prim.HasAPI(UsdPhysics.DriveAPI):
                        drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "angular" if joint_prim.IsA(UsdPhysics.RevoluteJoint) else "linear")
                        drive_api.GetTargetPositionAttr().Set(numpy.rad2deg(joint_value) if joint_prim.IsA(UsdPhysics.RevoluteJoint) else joint_value)
                    else:
                        print(f"Joint {joint_name} does not have DriveAPI")

            # entity_editor = Usd.NamespaceEditor(entity_stage)
            # entity_prim = entity_stage.GetDefaultPrim()
            # entity_editor.RenamePrim(entity_prim, entity.name)
            # entity_editor.ApplyEdits()

            entity_stage.GetRootLayer().Save()

        robots_path = os.path.join(self.save_dir_path, os.path.basename(self.save_file_path).split(".")[0] + "_robots.usda")
        robots_stage = Usd.Stage.CreateNew(robots_path)
        if len(robots) == 1:
            sublayer_paths = [robot.path for robot in robots.values()]
            robots_stage.GetRootLayer().subLayerPaths = sublayer_paths
            robots_stage.SetDefaultPrim(robots_stage.GetPrimAtPath(Usd.Stage.Open(sublayer_paths[0]).GetDefaultPrim().GetPath()))
        robots_stage.Flatten()
        robots_stage.Export(robots_path)

        print(f"Robots: {robots_path}")

        objects_path = os.path.join(self.save_dir_path, os.path.basename(self.save_file_path).split(".")[0] + "_objects.usda")
        objects_stage = Usd.Stage.CreateNew(objects_path)
        objects_prim_path = Sdf.Path("/Objects")
        objects_xform = UsdGeom.Xform.Define(objects_stage, objects_prim_path)
        objects_prim = objects_xform.GetPrim()
        objects_stage.SetDefaultPrim(objects_prim)

        for object in objects.values():
            object_prim_path = objects_prim_path.AppendChild(object.name)
            object_xform = UsdGeom.Xform.Define(objects_stage, object_prim_path)
            object_prim = object_xform.GetPrim()
            object_stage = Usd.Stage.Open(object.path)
            object_prim.GetReferences().AddReference(object_stage.GetRootLayer().identifier, object_stage.GetDefaultPrim().GetPath())

        objects_stage.Flatten()
        objects_stage.Export(objects_path)

        print(f"Objects: {objects_path}")

        file_ext = os.path.splitext(self.world_path)[1]
        if file_ext == ".usda":
            with open(self.save_file_path, "r") as f:
                data = f.read()
            world_usd_dir = os.path.dirname(self.world_path)
            data = data.replace("@./", f"@{world_usd_dir}/")
            with open(self.save_file_path, "w") as f:
                f.write(data)

        if multiverse_params is not None and multiverse_params != {}:
            world_stage = Usd.Stage.Open(self.save_file_path)
            customLayerData = world_stage.GetRootLayer().customLayerData
            customLayerData["multiverse_connector"] = {}
            customLayerData["multiverse_connector"]["host"] = multiverse_params.get("host", "tcp://127.0.0.1")
            customLayerData["multiverse_connector"]["server_port"] = multiverse_params.get("server_port", "7000")
            customLayerData["multiverse_connector"]["client_port"] = multiverse_params.get("client_port", "8000")
            customLayerData["multiverse_connector"]["world_name"] = multiverse_params.get("world_name", "world")
            customLayerData["multiverse_connector"]["simulation_name"] = multiverse_params.get("simulation_name", "isaac_sim")
            for send_receive in ["send", "receive"]:
                customLayerData["multiverse_connector"][send_receive] = multiverse_params.get(send_receive, {})
                for key, values in customLayerData["multiverse_connector"][send_receive].items():
                    customLayerData["multiverse_connector"][send_receive][key] = json.dumps(values)
            world_stage.GetRootLayer().customLayerData = customLayerData
            world_stage.GetRootLayer().Save()

            tmp_path = "/tmp/multiverse_isaacsim_connector.yaml"
            with open(tmp_path, "w") as f:
                f.write(str(customLayerData["multiverse_connector"]))

if __name__ == "__main__":
    multiverse_simulator_compiler_main(IsaacSimCompiler)
