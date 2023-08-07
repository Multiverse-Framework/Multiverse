#!/usr/bin/python3.10

import argparse
from urdf_to_fbx import urdf_to_fbx


def main():
    parser = argparse.ArgumentParser(description="Convert from URDF to FBX")
    parser.add_argument("--in_urdf", type=str, required=True, help="Input URDF")
    parser.add_argument("--out_fbx", type=str, required=True, help="Output FBX")
    parser.add_argument(
        "--merge_materials_with_name",
        type=bool,
        default=True,
        help="Merge materials if they have the same name and same content",
    )
    parser.add_argument(
        "--rename_materials", type=bool, default=True, help="Rename materials"
    )
    parser.add_argument(
        "--apply_weld", type=bool, default=True, help="Apply weld modifier"
    )
    parser.add_argument(
        "--unique_name", type=bool, default=True, help="Each texture has an unique name"
    )
    parser.add_argument(
        "--scale_unit",
        type=float,
        default=0.01,
        help="Scale unit (for Unreal Engine is 0.01)",
    )

    args = parser.parse_args()
    urdf_to_fbx(
        args.in_urdf,
        args.out_fbx,
        args.merge_materials_with_name,
        args.rename_materials,
        args.apply_weld,
        args.unique_name,
        args.scale_unit,
    )


if __name__ == "__main__":
    main()
