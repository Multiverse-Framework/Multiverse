#!/usr/bin/env python3

from rdflib import Graph
import argparse


def main():
    parser = argparse.ArgumentParser(description="Count number of nodes and edge of an OWL file")
    parser.add_argument("--in_owl", type=str, required=True, help="Input OWL")
    args = parser.parse_args()
    g = Graph()
    g.parse(args.in_owl)

    print("number of edges: %d" % len(g))
    print("number of nodes: %d" % len(g.all_nodes()))


if __name__ == "__main__":
    main()
