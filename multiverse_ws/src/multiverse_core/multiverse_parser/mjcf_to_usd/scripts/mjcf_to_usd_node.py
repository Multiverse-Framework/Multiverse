#!/usr/bin/env python3.10

import argparse
from mjcf_to_usd import mjcf_to_usd_with_physics, mjcf_to_usd_no_physics


def main():
    parser = argparse.ArgumentParser(description="Convert from MJCF to USD")
    parser.add_argument("--in_mjcf", type=str, required=True, help="Input MJCF")
    parser.add_argument("--out_usd", type=str, required=True, help="Output USD")
    parser.add_argument(
        "--with_physics",
        type=bool,
        default=True,
        help="Whether to include physics properties or not",
    )

    args = parser.parse_args()
    if args.with_physics:
        mjcf_to_usd_with_physics(args.in_mjcf, args.out_usd)
    else:
        mjcf_to_usd_no_physics(args.in_mjcf, args.out_usd)


if __name__ == "__main__":
    main()
