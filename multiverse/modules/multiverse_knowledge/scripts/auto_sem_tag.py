#!/usr/bin/env python3

import argparse
import shutil
import os
from pxr import Usd, UsdGeom, UsdOntology


sem_labels = {
    "box": ["_class_Box"],
    "cat": ["_class_Cat"],
    "spoon": ["_class_Spoon"],
    "milk_box": ["_class_MilkBottle"],
    "kitchen": ["_class_Kitchen"],
    "cabinet1": ["_class_KitchenCabinet"],
    "cabinet2": ["_class_KitchenCabinet"],
    "cabinet3": ["_class_Refrigerator"],
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
    "cabinet11_drawer3": ["_class_Drawer"],
}

sem_TBox = {}


def auto_sem_tag(in_ABox_usd_file: str, in_TBox_Usd_file: str, out_ABox_usd_file: str) -> None:
    tmp_out_ABox_usd_file = os.path.join(os.path.dirname(in_ABox_usd_file), "tmp.usda")
    shutil.copy(src=in_ABox_usd_file, dst=tmp_out_ABox_usd_file)

    stage_TBox = Usd.Stage.Open(in_TBox_Usd_file)
    for prim in stage_TBox.Traverse():
        for prim_class in prim.GetAllChildren():
            sem_TBox[prim_class.GetName()] = prim_class.GetPrimPath()

    stage_ABox = Usd.Stage.Open(tmp_out_ABox_usd_file)
    stage_ABox.GetRootLayer().subLayerPaths = [in_TBox_Usd_file]

    for prim in stage_ABox.Traverse():
        prepended_items = prim.GetPrim().GetPrimStack()[0].referenceList.prependedItems
        if len(prepended_items) == 1:
            prim.GetPrim().GetReferences().ClearReferences()
            mesh_dir_abs_path = prepended_items[0].assetPath
            prim_path = prepended_items[0].primPath
            if not os.path.isabs(mesh_dir_abs_path):
                if mesh_dir_abs_path[:2] == "./":
                    mesh_dir_abs_path = mesh_dir_abs_path[2:]
                mesh_dir_abs_path = os.path.join(os.path.dirname(in_ABox_usd_file), mesh_dir_abs_path)
            prim.GetPrim().GetReferences().AddReference(mesh_dir_abs_path, prim_path)
        elif len(prepended_items) > 1:
            mesh_dir_abs_paths = []
            prim_paths = []
            for prepended_item in prepended_items:
                mesh_dir_abs_path = prepended_item.assetPath
                prim_paths.append(prepended_item.primPath)
                if not os.path.isabs(mesh_dir_abs_path):
                    if mesh_dir_abs_path[:2] == "./":
                        mesh_dir_abs_path = mesh_dir_abs_path[2:]
                    mesh_dir_abs_path = os.path.join(os.path.dirname(in_ABox_usd_file), mesh_dir_abs_path)
                mesh_dir_abs_paths.append(mesh_dir_abs_path)
            prim.GetPrim().GetReferences().ClearReferences()
            for mesh_dir_abs_path, prim_path in zip(mesh_dir_abs_paths, prim_paths):
                prim.GetPrim().GetReferences().AddReference(mesh_dir_abs_path, prim_path)

        if prim.GetName() in sem_labels:
            semanticTagAPI = UsdOntology.SemanticTagAPI.Apply(prim)
            for sem_class in sem_labels[prim.GetName()]:
                if sem_class in sem_TBox:
                    semanticTagAPI.CreateSemanticLabelRel().AddTarget(sem_TBox[sem_class])

    print(f"Save usd stage to {out_ABox_usd_file} that has semantic labels from {in_TBox_Usd_file}")
    stage_ABox.GetRootLayer().Save()
    os.rename(tmp_out_ABox_usd_file, out_ABox_usd_file)

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
