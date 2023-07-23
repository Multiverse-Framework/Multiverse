#!/usr/bin/env python3

import sys
from usd_to_urdf import usd_to_urdf


if __name__ == "__main__":
    if len(sys.argv) >= 3:
        (usd_file, urdf_file) = (sys.argv[1], sys.argv[2])
    else:
        print("Usage: in_usd.usda out_urdf.urdf")
        sys.exit(1)

    usd_to_urdf(usd_file, urdf_file)
