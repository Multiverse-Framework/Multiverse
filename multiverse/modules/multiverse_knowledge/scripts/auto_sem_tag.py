#!/usr/bin/env python3

import argparse
import shutil
import os
from pxr import Usd, UsdOntology


sem_labels = {"box": ["_class_Box"], "cat": ["_class_Cat"], 
              "kitchen": ["_class_Kitchen"], 
              "cabinet1": ["_class_KitchenCabinet"],
              "cabinet2": ["_class_KitchenCabinet"],
              "cabinet3": ["_class_KitchenCabinet"],
              "cabinet4": ["_class_KitchenCabinet"],
              "cabinet5": ["_class_KitchenCabinet"],
              "cabinet6": ["_class_KitchenCabinet"],
              "cabinet8": ["_class_KitchenCabinet"],
              "cabinet9": ["_class_KitchenCabinet"],
              "cabinet10": ["_class_KitchenCabinet"],
              "cabinet11": ["_class_KitchenCabinet"],
              "cabinet1_drawer1": ["_class_Drawer"],
              "cabinet1_drawer2": ["_class_Drawer"],
              "cabinet2_drawer1": ["_class_Drawer"],
              "cabinet2_drawer2": ["_class_Drawer"],
              "cabinet2_drawer3": ["_class_Drawer"],
              "cabinet2_drawer4": ["_class_Drawer"],
              "cabinet2_drawer5": ["_class_Drawer"],
              "cabinet2_drawer6": ["_class_Drawer"],
              "cabinet5_drawer1": ["_class_Drawer"],
              "cabinet5_drawer2": ["_class_Drawer"],
              "cabinet5_drawer3": ["_class_Drawer"],
              "cabinet6_drawer1": ["_class_Drawer"],
              "cabinet6_drawer2": ["_class_Drawer"],
              "cabinet6_drawer3": ["_class_Drawer"],
              "cabinet9_drawer1": ["_class_Drawer"],
              "cabinet9_drawer2": ["_class_Drawer"],
              "cabinet9_drawer3": ["_class_Drawer"],
              "cabinet10_drawer1": ["_class_Drawer"],
              "cabinet10_drawer2": ["_class_Drawer"],
              "cabinet10_drawer3": ["_class_Drawer"],
              "cabinet11_drawer1": ["_class_Drawer"],
              "cabinet11_drawer2": ["_class_Drawer"],
              "cabinet11_drawer3": ["_class_Drawer"]}

sem_TBox = {}


def auto_sem_tag(in_ABox_usd_file: str, in_TBox_Usd_file: str, out_ABox_usd_file: str) -> None:
    in_mesh_dir = in_ABox_usd_file.replace(".usda", "").replace(".usdc", "")
    out_mesh_dir = os.path.join(os.path.dirname(out_ABox_usd_file), os.path.basename(in_ABox_usd_file.replace(".usda", "").replace(".usdc", "")))
    shutil.copytree(in_mesh_dir, out_mesh_dir, dirs_exist_ok=True)
    shutil.copy(in_ABox_usd_file, out_ABox_usd_file)

    stage_TBox = Usd.Stage.Open(in_TBox_Usd_file)
    for prim in stage_TBox.Traverse():
        for prim_class in prim.GetAllChildren():
            sem_TBox[prim_class.GetName()] = prim_class.GetPrimPath()

    stage_ABox = Usd.Stage.Open(out_ABox_usd_file)
    stage_ABox.GetRootLayer().subLayerPaths = ["./" + os.path.relpath(in_TBox_Usd_file, os.path.dirname(out_ABox_usd_file))]

    for prim in stage_ABox.Traverse():
        if prim.GetName() in sem_labels:
            semanticTagAPI = UsdOntology.SemanticTagAPI.Apply(prim)
            for sem_class in sem_labels[prim.GetName()]:
                if sem_class in sem_TBox:
                    semanticTagAPI.CreateSemanticLabelRel().AddTarget(sem_TBox[sem_class])

    print(f"Save usd stage to {out_ABox_usd_file} that has semantic labels from {in_TBox_Usd_file}")
    stage_ABox.GetRootLayer().Save()

    return None


def main():
    parser = argparse.ArgumentParser(description="Auto semantic tagging based on object names")
    parser.add_argument("--in_usd", type=str, required=True, help="Input USD")
    parser.add_argument("--in_TBox_usd", type=str, required=True, help="Input TBox USD")
    parser.add_argument("--out_ABox_usd", type=str, required=True, help="Output ABox USD")
    args = parser.parse_args()
    auto_sem_tag(args.in_usd, args.in_TBox_usd, args.out_ABox_usd)


if __name__ == "__main__":
    main()
