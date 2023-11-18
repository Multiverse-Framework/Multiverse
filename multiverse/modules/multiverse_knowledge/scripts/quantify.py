#!/usr/bin/env python3

import argparse
from rdflib import Graph
from rdflib import RDF
from rdflib import OWL


def ontology_stats(path):
    g = Graph()
    g.parse(path)
    print("%s:" % path)
    print("  number of nodes: %d" % len(g.all_nodes()))
    print("  number of edges: %d" % len(g))
    numClasses = 0
    for s, p, o in g.triples((None, RDF.type, OWL.Class)):
        numClasses += 1
    print("  number of classes: %d" % numClasses)
    numInstances = 0
    for s, p, o in g.triples((None, RDF.type, OWL.NamedIndividual)):
        numInstances += 1
    print("  number of instances: %d" % numInstances)
    return (len(g), len(g.all_nodes()), numClasses, numInstances)


def main():
    parser = argparse.ArgumentParser(description="Count number of nodes and edge of an OWL file")
    parser.add_argument("--in_owl", type=str, required=True, help="Input OWL")
    args = parser.parse_args()
    ontology_stats(args.in_owl)


if __name__ == "__main__":
    main()
