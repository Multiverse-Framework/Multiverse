#!/usr/bin/env python3

import sys

from sdf_to_urdf import sdf_to_urdf


if __name__ == "__main__":
    if len(sys.argv) >= 3:
        (sdf_file, urdf_file) = (sys.argv[1], sys.argv[2])
    else:
        print("Usage: in_sdf.sdf out_urdf.urdf")
        sys.exit(1)
    sdf_to_urdf(sdf_file, urdf_file)
