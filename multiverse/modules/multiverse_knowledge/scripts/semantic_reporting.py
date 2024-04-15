#!/usr/bin/env python3

import argparse
import shutil
import os
from pxr import Usd, UsdGeom, UsdOntology

# import dfl.semrep as semrep

# keyFRED = 'b67a0577-8c76-3889-89b2-cf3dceab4a0e'

# keyWikiData = 'eyJ0eXAiOiJKV1QiLCJhbGciOiJSUzI1NiJ9.eyJhdWQiOiI2M2FmZjQ4YTFlOWQxZTUwOWU4YjkwNzkwMDZmZDQzOSIsImp0aSI6IjM0YzRkMjM3N2FjMDU2YmU0NjYxNGViZTc2MDEzMjYyYjE2NGQ5ZWJhOTVjN2FjN2EwZTg3MDhiMWZkNTI0OTViNzY3OTJkN2E3ZTQ4NWFkIiwiaWF0IjoxNzEyNzYzMDIzLjY0NzI2MywibmJmIjoxNzEyNzYzMDIzLjY0NzI2NywiZXhwIjozMzI2OTY3MTgyMy42NDIxOTcsInN1YiI6Ijc1Mzk5MTQ3IiwiaXNzIjoiaHR0cHM6Ly9tZXRhLndpa2ltZWRpYS5vcmciLCJyYXRlbGltaXQiOnsicmVxdWVzdHNfcGVyX3VuaXQiOjUwMDAsInVuaXQiOiJIT1VSIn0sInNjb3BlcyI6WyJiYXNpYyIsImhpZ2h2b2x1bWUiXX0.b2oZdt2sqNacRJ5_uAJNHELJyxQk-If9qFLhGb9mFqMzcK3XjpQu9HlxjTWmKbTVlcJIyCGCR9xmT9j2AN5LgbWyn1vuooXii94CJw5PSvppK8kWDMSIftbrJ4yHs2YXtD0DF31RoaSv9vDL_kmKcifNdV5W6F7TUE7Ik2HgYXg8-hsZa1hMW1Qm-cnf6l2ysDz4TK1skJTkzgMgFJSkjHSkK3Ehh7G9vKVujhkJLq5CkkNb6slwpfw4D7uUk4pjGiL3wIJqG0Mn6AjgGvj-cjKsi1cFN8WLvt0IdCjZ24tvDJJgufVrnbLhFji_mpnwqkPgrSV7jzGdV_UXht5M1Ex7rPjDtMZX9ZswQs4V-sa8h_wQV8SsxDnR4fE--_QuLPfHLv3l2DQASSBP3d1XljdmU-eJVjUbRnvanaa32_yyZ3sSvYK1EhXx4HTrxakPZ8kWGo-KAo0KGOy9gYjr9BzKk6EuEKtyArzMsvk7AuKfe_Wxw7c0Z8TGQ4-vawUSyUFsm-bw9CSr9kQ8SymaEDYG2TjLQeBYN_kzObCEpLqrJts_cewN27nCwsz8e5cRKqoJ5j2F2yAbbV83ewp7HT_Ze7pAtiCKgp42ExzkuvrS8Cgkc_ZvQ8J_R7OqNboG4l5sRB5Kjz-JCjt6Nfyn5nW-a5-KyeMr-mCeKCEwNA8'

# semrep.setFREDKey(keyFRED)
# semrep.setWikiDataKey(keyWikiData)


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

    "SM_Cup": ["_class_Cup"],
    "SM_CerealBox": ["_class_CerealBox"],
    "SM_SmallBowl": ["_class_Bowl"],
    "SM_MilkBox": ["_class_MilkBottle"],
    "SM_Apartment": ["_class_Kitchen"],
    "SM_Kitchen_01_Base": ["_class_KitchenCabinet"],
    "SM_Kitchen_02_Base": ["_class_KitchenCabinet"],
    "SM_Kitchen_03_Fridge": ["_class_KitchenCabinet"],
    "SM_Kitchen_04_Base": ["_class_KitchenCabinet"],
    "SM_Kitchen_05_Base": ["_class_KitchenCabinet"],
    "SM_Kitchen_06_Base": ["_class_KitchenCabinet"],
    "SM_Kitchen_08_Sink_Base": ["_class_KitchenCabinet"],
    "SM_Kitchen_09_Base": ["_class_KitchenCabinet"],
    "SM_Kitchen_10_Base": ["_class_KitchenCabinet"],
    "SM_Kitchen_11_Base": ["_class_KitchenCabinet"],
    "SM_Fridge_Base": ["_class_Refrigerator"],
    "SM_Kitchen_01_Drawer_01": ["_class_Drawer"],
    "SM_Kitchen_01_Drawer_02": ["_class_Drawer"],
    "SM_Kitchen_02_Drawer_01": ["_class_Drawer"],
    "SM_Kitchen_02_Drawer_02": ["_class_Drawer"],
    "SM_Kitchen_02_Drawer_3": ["_class_Drawer"],
    "SM_Kitchen_02_Drawer_4": ["_class_Drawer"],
    "SM_Kitchen_02_Drawer_5": ["_class_Drawer"],
    "SM_Kitchen_02_Drawer_6": ["_class_Drawer"],
    "SM_Kitchen_05_Base_Drawer_01": ["_class_Drawer"],
    "SM_Kitchen_05_Base_Drawer_02": ["_class_Drawer"],
    "SM_Kitchen_05_Base_Drawer_03": ["_class_Drawer"],
    "SM_Kitchen_06_Base_Drawer_01": ["_class_Drawer"],
    "SM_Kitchen_06_Base_Drawer_02": ["_class_Drawer"],
    "SM_Kitchen_06_Base_Drawer_03": ["_class_Drawer"],
    "SM_Kitchen_09_Drawer_01": ["_class_Drawer"],
    "SM_Kitchen_09_Drawer_02": ["_class_Drawer"],
    "SM_Kitchen_09_Drawer_03": ["_class_Drawer"],
    "SM_Kitchen_10_Drawer_01": ["_class_Drawer"],
    "SM_Kitchen_10_Drawer_02": ["_class_Drawer"],
    "SM_Kitchen_10_Drawer_03": ["_class_Drawer"],
    "SM_Kitchen_11_Base_Drawer_01": ["_class_Drawer"],
    "SM_Kitchen_11_Base_Drawer_02": ["_class_Drawer"],
    "SM_Kitchen_11_Base_Drawer_03": ["_class_Drawer"],
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

        # if prim.IsA(UsdGeom.Xform): #and prim.GetName() in sem_labels:
        #     semanticTagAPI = UsdOntology.SemanticTagAPI.Apply(prim)
        #     # TODO: consider some preprocessing of string obtained from prim.GetName to remove numbers at the end, or prefixes that are not 
        #     # useful for semantic tagging.
        #     report = [x.get("SOMA_DFL") for x in semrep.semanticReport(prim.GetName()) if "SOMA_DFL" in x]
        #     for sem_class in report:#sem_labels[prim.GetName()]:
        #         if sem_class in sem_TBox:
        #             # This assumes that SOMA_DFL has previously been converted to the USDA TBox.
        #             semanticTagAPI.CreateSemanticLabelRel().AddTarget(sem_TBox[sem_class])

        if prim.IsA(UsdGeom.Xform) and prim.GetName() in sem_labels:
            semanticTagAPI = UsdOntology.SemanticTagAPI.Apply(prim)
            for sem_class in sem_labels[prim.GetName()]:
                if sem_class in sem_TBox:
                    semanticTagAPI.CreateSemanticReportsRel().AddTarget(sem_TBox[sem_class])

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
