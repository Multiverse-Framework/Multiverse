#!/usr/bin/env python3.10

import argparse
from urdf_to_urdf import urdf_to_urdf


def main():
    parser = argparse.ArgumentParser(description="Clean up URDF, change *.dae to *.obj, add missing data")
    parser.add_argument("--in_urdf", type=str, required=True, help="Input URDF")
    parser.add_argument("--out_urdf", type=str, required=True, help="Output URDF")
    parser.add_argument(
        "--rename_materials", type=bool, default=True, help="Rename materials"
    )
    parser.add_argument(
        "--apply_weld", type=bool, default=True, help="Apply weld modifier"
    )
    parser.add_argument(
        "--scale_unit",
        type=float,
        default=0.01,
        help="Scale unit (for Unreal Engine is 0.01)",
    )

    args = parser.parse_args()
    urdf_to_urdf(
        args.in_urdf,
        args.out_urdf,
    )


if __name__ == "__main__":
    main()
