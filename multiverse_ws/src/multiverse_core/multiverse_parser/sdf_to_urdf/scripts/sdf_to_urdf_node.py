#!/usr/bin/env python3

import argparse
import os

def main():
    parser = argparse.ArgumentParser(description="Convert from SDF to URDF")
    parser.add_argument("--in_sdf", type=str, required=True, help="Input SDF")
    parser.add_argument("--out_urdf", type=str, required=True, help="Output URDF")
    parser.add_argument("--mesh_path", type=str, help="Mesh path")

    args = parser.parse_args()
    if args.mesh_path is None:
        os.environ["MESH_WORKSPACE_PATH"] = os.path.dirname(args.in_sdf)
    from sdf_to_urdf import sdf_to_urdf

    sdf_to_urdf(args.in_sdf, args.out_urdf)


if __name__ == "__main__":
    main()
