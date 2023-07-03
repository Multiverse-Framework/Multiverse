#!/usr/bin/env python3

import sys
import shutil
import os
from pxr import Usd, UsdOntology


sem_labels = {
    'box': ['_class_Box'],
    'cat': ['_class_Cat']
}

sem_TBox = {}


def auto_sem_tag(in_usd_ABox_file: str, in_usd_TBox_file: str, out_usd_ABox_file: str) -> None:
    in_mesh_dir = in_usd_ABox_file.replace('.usda', '')
    out_mesh_dir = os.path.join(os.path.dirname(out_usd_ABox_file), os.path.basename(in_usd_ABox_file.replace('.usda', '')))
    shutil.copytree(in_mesh_dir, out_mesh_dir, dirs_exist_ok=True)
    shutil.copy(in_usd_ABox_file, out_usd_ABox_file)

    stage_TBox = Usd.Stage.Open(in_usd_TBox_file)
    for prim in stage_TBox.Traverse():
        for prim_class in prim.GetAllChildren():
            sem_TBox[prim_class.GetName()] = prim_class.GetPrimPath()

    stage_ABox = Usd.Stage.Open(out_usd_ABox_file)
    stage_ABox.GetRootLayer().subLayerPaths = ['./' + os.path.relpath(in_usd_TBox_file, os.path.dirname(out_usd_ABox_file))]

    for prim in stage_ABox.Traverse():
        if prim.GetName() in sem_labels:
            semanticTagAPI = UsdOntology.SemanticTagAPI.Apply(prim)
            for sem_class in sem_labels[prim.GetName()]:
                if sem_class in sem_TBox:
                    semanticTagAPI.CreateSemanticLabelRel().AddTarget(sem_TBox[sem_class])
    
    print(f'Save usd stage to {out_usd_ABox_file} that has semantic labels from {in_usd_TBox_file}')
    stage_ABox.GetRootLayer().Save()

    return None


if __name__ == '__main__':
    if len(sys.argv) >= 3:
        (in_usd_ABox_file, in_usd_TBox_file, out_usd_ABox_file) = (sys.argv[1], sys.argv[2], sys.argv[3])
    else:
        print('Usage: in_ABox_usd.usda in_TBox_usd.usda out_ABox_usd.usda')
        sys.exit(1)
    auto_sem_tag(in_usd_ABox_file, in_usd_TBox_file, out_usd_ABox_file)
