#!/usr/bin/env python3

import sys
import os
from pxr import Usd, UsdOntology


sem_labels = {
    'box': ['_class_Box'],
    'cat': ['_class_Cat']
}

sem_TBox = {}


def auto_sem_tag(usd_ABox_file: str, usd_TBox_file: str) -> None:
    stage_TBox = Usd.Stage.Open(usd_TBox_file)
    for prim in stage_TBox.Traverse():
        for prim_class in prim.GetAllChildren():
            sem_TBox[prim_class.GetName()] = prim_class.GetPrimPath()

    stage_ABox = Usd.Stage.Open(usd_ABox_file)
    stage_ABox.GetRootLayer().subLayerPaths = [usd_TBox_file]

    for prim in stage_ABox.Traverse():
        if prim.GetName() in sem_labels:
            semanticTagAPI = UsdOntology.SemanticTagAPI.Apply(prim)
            for sem_class in sem_labels[prim.GetName()]:
                if sem_class in sem_TBox:
                    semanticTagAPI.CreateSemanticLabelRel().AddTarget(sem_TBox[sem_class])
    
    print(f'Save usd stage to {usd_ABox_file} that has semantic labels from {usd_TBox_file}')
    stage_ABox.GetRootLayer().Save()

    return None


if __name__ == '__main__':
    if len(sys.argv) >= 3:
        (usd_ABox_file, usd_TBox_file) = (sys.argv[1], sys.argv[2])
    else:
        print('Usage: in_ABox_usd.usda in_TBox_usd.usda')
        sys.exit(1)
    auto_sem_tag(usd_ABox_file, usd_TBox_file)
