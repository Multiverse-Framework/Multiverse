#!/usr/bin/env python3

import argparse
import shutil
import os
import re
from pxr import Usd, UsdGeom, UsdOntology
import time

synonyms = {
    "box": ["carton"],
    "milch": ["milk"],
    "behaelter": ["container"],
    "tur": ["door"],
    "griff": ["handle"],
    "recht": ["right"],
    "links": ["left"],
    "tv": ["television"],
    "kaffee": ["coffee"],
    "maschine": ["machine"],
    "tisch": ["table"],
    "stuhl": ["chair"],
    "schublade": ["drawer"],
    "kleiderschrank": ["wardrobe"],
}

def semantic_reporting(semrep, in_ABox_usd_file: str, in_TBox_Usd_file: str, out_ABox_usd_file: str) -> None:
    tmp_out_ABox_usd_file = os.path.join(os.path.dirname(in_ABox_usd_file), "tmp.usda")
    shutil.copy(src=in_ABox_usd_file, dst=tmp_out_ABox_usd_file)

    stage_TBox = Usd.Stage.Open(in_TBox_Usd_file)

    stage_ABox = Usd.Stage.Open(tmp_out_ABox_usd_file)
    stage_ABox.GetRootLayer().subLayerPaths = [in_TBox_Usd_file]

    report_cached = {}

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

        if prim.IsA(UsdGeom.Xform):
            semanticTagAPI = UsdOntology.SemanticTagAPI.Apply(prim)
            # Preprocessing of string obtained from prim.GetName to remove numbers at the end, 
            # or prefixes that are not useful for semantic tagging.
            prim_name = re.sub(r'\d+', '', prim.GetName())
            if prim_name.lower().startswith("sm"):
                prim_name = prim_name[2:]

            report = []

            prim_name_list = [prim_name.lower()]

            prim_names = prim_name.split("_")
            for prim_name in prim_names:
                if prim_name == '':
                    continue

                prim_name_list += [prim_name.lower()]

                prim_name_synonym = prim_name.lower()
                for word in synonyms:
                    if word in prim_name_synonym:
                        for synonym in synonyms[word]:
                            prim_name_synonym = prim_name_synonym.replace(word, synonym)
                if prim_name_synonym != prim_name.lower():
                    prim_name_list.append(prim_name_synonym)

                if (any(char.isupper() for char in prim_name)):
                    prim_name = prim_name[0].upper() + prim_name[1:]
                    prim_name_list += [p.lower() for p in re.findall(r'[A-Z](?:[a-z]+|[A-Z]*(?=[A-Z]|$))', prim_name)]

                prim_name_list += [synonym for p in prim_name_list if p.lower() in synonyms for synonym in synonyms[p.lower()]]

            prim_name_set = set(p for p in prim_name_list if p != '')

            for p in prim_name_set:
                if p not in report_cached:
                    report_cached[p] = [x["SOMA_DFL"] for x in semrep.semanticReport(p) if "SOMA_DFL" in x]
                report += report_cached[p]

            report = sorted(tuple(set(report)))

            print(f"Found {report} for {prim.GetName()}")

            for sem_class in report:
                soma_dfl, sem_class = sem_class.split("#")
                sem_class = sem_class.replace('.', '')
                if soma_dfl == 'http://www.ease-crc.org/ont/SOMA_DFL.owl':
                    sem_path = f"/SOMA_DFL/_class_{sem_class}"
                    if stage_TBox.GetPrimAtPath(sem_path):
                        semanticTagAPI.CreateSemanticReportsRel().AddTarget(sem_path)
                    else:
                        print(f"Semantic path {sem_path} not found in TBox")

    print(f"Save usd stage to {out_ABox_usd_file} that has semantic labels from {in_TBox_Usd_file}")
    stage_ABox.GetRootLayer().Save()
    os.rename(tmp_out_ABox_usd_file, out_ABox_usd_file)

    return None


def main():
    parser = argparse.ArgumentParser(description="Auto semantic reporting based on object names")
    parser.add_argument("--in_usd", type=str, required=True, help="Input USD")
    parser.add_argument("--in_TBox_usd", type=str, required=True, help="Input TBox USD")
    parser.add_argument("--out_usd", type=str, required=True, help="Output USD")
    args = parser.parse_args()

    import dfl.semrep as semrep

    print("Loading semantic reporting module...")

    keyFRED = 'b67a0577-8c76-3889-89b2-cf3dceab4a0e'

    keyWikiData = 'eyJ0eXAiOiJKV1QiLCJhbGciOiJSUzI1NiJ9.eyJhdWQiOiI2M2FmZjQ4YTFlOWQxZTUwOWU4YjkwNzkwMDZmZDQzOSIsImp0aSI6IjM0YzRkMjM3N2FjMDU2YmU0NjYxNGViZTc2MDEzMjYyYjE2NGQ5ZWJhOTVjN2FjN2EwZTg3MDhiMWZkNTI0OTViNzY3OTJkN2E3ZTQ4NWFkIiwiaWF0IjoxNzEyNzYzMDIzLjY0NzI2MywibmJmIjoxNzEyNzYzMDIzLjY0NzI2NywiZXhwIjozMzI2OTY3MTgyMy42NDIxOTcsInN1YiI6Ijc1Mzk5MTQ3IiwiaXNzIjoiaHR0cHM6Ly9tZXRhLndpa2ltZWRpYS5vcmciLCJyYXRlbGltaXQiOnsicmVxdWVzdHNfcGVyX3VuaXQiOjUwMDAsInVuaXQiOiJIT1VSIn0sInNjb3BlcyI6WyJiYXNpYyIsImhpZ2h2b2x1bWUiXX0.b2oZdt2sqNacRJ5_uAJNHELJyxQk-If9qFLhGb9mFqMzcK3XjpQu9HlxjTWmKbTVlcJIyCGCR9xmT9j2AN5LgbWyn1vuooXii94CJw5PSvppK8kWDMSIftbrJ4yHs2YXtD0DF31RoaSv9vDL_kmKcifNdV5W6F7TUE7Ik2HgYXg8-hsZa1hMW1Qm-cnf6l2ysDz4TK1skJTkzgMgFJSkjHSkK3Ehh7G9vKVujhkJLq5CkkNb6slwpfw4D7uUk4pjGiL3wIJqG0Mn6AjgGvj-cjKsi1cFN8WLvt0IdCjZ24tvDJJgufVrnbLhFji_mpnwqkPgrSV7jzGdV_UXht5M1Ex7rPjDtMZX9ZswQs4V-sa8h_wQV8SsxDnR4fE--_QuLPfHLv3l2DQASSBP3d1XljdmU-eJVjUbRnvanaa32_yyZ3sSvYK1EhXx4HTrxakPZ8kWGo-KAo0KGOy9gYjr9BzKk6EuEKtyArzMsvk7AuKfe_Wxw7c0Z8TGQ4-vawUSyUFsm-bw9CSr9kQ8SymaEDYG2TjLQeBYN_kzObCEpLqrJts_cewN27nCwsz8e5cRKqoJ5j2F2yAbbV83ewp7HT_Ze7pAtiCKgp42ExzkuvrS8Cgkc_ZvQ8J_R7OqNboG4l5sRB5Kjz-JCjt6Nfyn5nW-a5-KyeMr-mCeKCEwNA8'

    # semrep.setFREDKey(keyFRED)
    # semrep.setWikiDataKey(keyWikiData)
    semrep.initializeOntology()

    semantic_reporting(semrep, args.in_usd, args.in_TBox_usd, args.out_usd)


if __name__ == "__main__":
    main()
