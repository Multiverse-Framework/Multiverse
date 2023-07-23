#!/usr/bin/env python3

import sys

from mjcf_to_usd import mjcf_to_usd_with_physics

if __name__ == "__main__":
    if len(sys.argv) >= 3:
        (xml_file, usd_file) = (sys.argv[1], sys.argv[2])
    else:
        print("Usage: in_mjcf.mjcf out_usd.usda")
        sys.exit(1)

    mjcf_to_usd_with_physics(xml_file, usd_file)
