#!/usr/bin/env python3

import sys

from sdf_to_urdf import SDF


def sdf_to_urdf(sdf_file, urdf_file, prefix=None):
    sdf = SDF(file=sdf_file)
    world = sdf.world
    if len(world.models) != 1:
        print(
            "SDF contains %s instead of exactly one model. Aborting."
            % len(world.models)
        )
        sys.exit(1)

    model = world.models[0]
    model.save_urdf(urdf_file, prefix)
