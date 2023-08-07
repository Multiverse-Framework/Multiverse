#!/usr/bin/env python3.10

import argparse
from usd_to_urdf import usd_to_urdf


def main():
    parser = argparse.ArgumentParser(description="Convert from USD to URDF")
    parser.add_argument("--in_usd", type=str, required=True, help="Input USD")
    parser.add_argument("--out_urdf", type=str, required=True, help="Output URDF")

    args = parser.parse_args()
    usd_to_urdf(args.in_usd, args.out_urdf)


if __name__ == "__main__":
    main()
