#!/usr/bin/env python3

import argparse
import os
import shutil
from pxr import Usd, UsdOntology
from owlready2 import onto_path, get_ontology
import re

onto_set = set()

N = 1


def create_path(name: str, is_ns: bool, prefix="/_class_") -> str:
    usd_path = name.replace("https://", "")
    usd_path = usd_path.replace("http://", "")
    usd_path = usd_path.replace("www", "")
    usd_path = usd_path.replace(".owl/", ".owl#")
    usd_path = usd_path.replace(".owl", "")
    usd_path = re.sub(r"[^a-zA-Z/]+", "", usd_path)
    words = usd_path.split("/")[-N:]
    if is_ns:
        usd_path = prefix + "/".join(words) + "_namespace"
    else:
        usd_path = prefix + "/".join(words)
    return usd_path


def owl_to_usd_impl(stage: Usd.Stage, concepts: list) -> None:
    S = dict()
    for concept in concepts:
        (iri_prefix, iri_name) = (concept.namespace.base_iri, concept.name)
        if iri_prefix == "https://ease-crc.org/ont/USD.owl#":
            continue

        if S.get(iri_prefix) == None:
            prim = stage.CreateClassPrim(create_path(iri_prefix, True))
            rdfAPI = UsdOntology.RdfAPI.Apply(prim)
            rdfAPI.CreateRdfNamespaceAttr().Set(iri_prefix)
            S[iri_prefix] = prim

        prim_child = stage.CreateClassPrim(create_path(iri_prefix, False, "/") + create_path(iri_name, False))
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


def usd_to_owl(onto_file: str, usd_file: str, onto_dir_copy: str) -> None:
    upper_onto_path = os.path.dirname(onto_file)
    onto_path.append(upper_onto_path)

    if onto_dir_copy is not None:
        os.makedirs(os.path.dirname(onto_dir_copy), exist_ok=True)
        for file in os.listdir(upper_onto_path):
            src_file = os.path.join(upper_onto_path, file)
            shutil.copy(src_file, onto_dir_copy)

    TBox_onto = get_ontology("file://" + onto_file)
    TBox_onto.load()
    onto_set.add(TBox_onto)

    import_ontos(TBox_onto)

    stage = Usd.Stage.CreateNew(usd_file)

    for onto in onto_set:
        owl_to_usd_impl(stage, list(onto.classes()))

    stage.GetRootLayer().Save()

    return None


def main():
    parser = argparse.ArgumentParser(description="Converting the TBox ontology from OWL to USD, which represents the TBox")
    parser.add_argument("--in_owl", type=str, required=True, help="Input OWL")
    parser.add_argument("--out_usd", type=str, required=True, help="Output USD")
    parser.add_argument("--out_owl", type=str, default=None, required=False, help="Output OWL (copy from input OWL)")
    args = parser.parse_args()
    usd_to_owl(args.in_owl, args.out_usd, args.out_owl)


if __name__ == "__main__":
    main()
