#!/usr/bin/env python3

import sys
import os
import shutil
from pxr import Usd, UsdOntology
from owlready2 import onto_path, get_ontology
import re
import rospkg

onto_set = set()

N = 1


def create_path(name: str, is_ns: bool, prefix = '/_class_') -> str:
    usd_path = name.replace('https://', '')
    usd_path = usd_path.replace('http://', '')
    usd_path = usd_path.replace('www', '')
    usd_path = usd_path.replace('.owl', '')
    usd_path = re.sub(r'[^a-zA-Z/]+', '', usd_path)
    words = usd_path.split('/')[-N:]
    if is_ns:
        usd_path = prefix + '/'.join(words) + '_namespace'
    else:
        usd_path = prefix + '/'.join(words)
    return usd_path


def owl_to_usd_impl(stage: Usd.Stage, concepts: list) -> None:
    S = dict()
    for concept in concepts:
        (iri_prefix, iri_name) = (concept.namespace.base_iri, concept.name)
        if S.get(iri_prefix) == None:
            prim = stage.CreateClassPrim(create_path(iri_prefix, True))
            rdfAPI = UsdOntology.RdfAPI.Apply(prim)
            rdfAPI.CreateRdfNamespaceAttr().Set(iri_prefix)
            S[iri_prefix] = prim

        prim_child = stage.CreateClassPrim(create_path(
            iri_prefix, False, '/') + create_path(iri_name, False))
        prim_child.GetInherits().AddInherit(S.get(iri_prefix).GetPrimPath())

        rdfAPI = UsdOntology.RdfAPI.Apply(prim_child)
        rdfAPI.CreateRdfConceptNameAttr().Set(iri_name)
    return None


def import_ontos(onto) -> None:
    for imported_onto in onto.imported_ontologies:
        imported_onto.load()
        if not imported_onto in onto_set:
            onto_set.add(imported_onto)
            import_ontos(imported_onto)
    return None


def usd_to_owl(onto_file : str, usd_file: str) -> None:
    rospack = rospkg.RosPack()
    upper_onto_path = rospack.get_path('mujoco_sim') + '/model/owl/'
    onto_path.append(upper_onto_path)

    for file in os.listdir(upper_onto_path):
        src_file = os.path.join(upper_onto_path, file)
        dst_file = os.path.join(os.path.dirname(usd_file), file)
        shutil.copy(src_file, dst_file)

    TBox_onto = get_ontology('file://' + onto_file)
    TBox_onto.load()
    onto_set.add(TBox_onto)

    import_ontos(TBox_onto)

    stage = Usd.Stage.CreateNew(usd_file)

    for onto in onto_set:
        owl_to_usd_impl(stage, list(onto.classes()))

    stage.GetRootLayer().Save()



    return None


if __name__ == '__main__':
    if len(sys.argv) >= 3:
        (onto_file, usd_file) = (sys.argv[1], sys.argv[2])
    else:
        print('Usage: in_onto.owl out_usd.usda')
        sys.exit(1)
    usd_to_owl(onto_file, usd_file)
