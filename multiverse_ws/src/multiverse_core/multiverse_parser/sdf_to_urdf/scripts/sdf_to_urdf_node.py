#!/usr/bin/env python3

import sys

from sdf_to_urdf import SDF


def sdf_to_urdf(sdf_file, urdf_file):
    sdf = SDF(file=sdf_file)
    world = sdf.world
    if len(world.models) != 1:
        print("SDF contains %s instead of exactly one model. Aborting." % len(world.models))
        sys.exit(1)

    model = world.models[0]
    model.save_urdf(urdf_file, prefix=None)


if __name__ == "__main__":
    if len(sys.argv) >= 3:
        (sdf_file, urdf_file) = (sys.argv[1], sys.argv[2])
    else:
        print("Usage: in_sdf.sdf out_urdf.urdf")
        sys.exit(1)
    sdf_to_urdf(sdf_file, urdf_file)
