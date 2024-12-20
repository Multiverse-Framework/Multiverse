#!/usr/bin/env python3

import time
from multiverse_simulator import MultiverseSimulatorConstraints
from mujoco_connector import MultiverseMujocoConnector

import argparse
import sys
import json


def main():
    try:
        parser = argparse.ArgumentParser(description="Run the Mujoco Connector")
        parser.add_argument("--file_path", type=str, required=True,
                            help="Path to the Mujoco XML file")
        parser.add_argument("--use_mjx", required=False, action='store_true',
                            help="Use MJX (https://mujoco.readthedocs.io/en/stable/mjx.html)")
        parser.add_argument("--headless", required=False, action='store_true',
                            help="Run in headless mode")
        parser.add_argument("--real_time_factor", type=float, required=False, default=1.0,
                            help="Real time factor")
        parser.add_argument("--step_size", type=float, required=False, default=0.01,
                            help="Step size")
        parser.add_argument("--integrator", type=str, required=False, default="IMPLICITFAST",
                            help="This attribute selects the numerical integrator to be used "
                                 "(EULER, RK4, IMPLICIT, IMPLICITFAST)")
        parser.add_argument("--noslip_iterations", type=int, required=False, default=0,
                            help="Maximum number of iterations of the Noslip solver")
        parser.add_argument("--noslip_tolerance", type=float, required=False, default=1e-6,
                            help="Tolerance of the Noslip solver")
        parser.add_argument("--cone", type=str, required=False, default="ELLIPTIC",
                            help="This attribute selects the contact friction model to be used "
                                 "(PYRAMIDAL, ELLIPTIC)")
        parser.add_argument("--impratio", type=int, required=False, default=100,
                            help="This attribute determines the ratio of frictional-to-normal constraint impedance "
                                 "for elliptic friction cones")
        parser.add_argument("--multiccd", type=bool, required=False, default=True,
                            help="This flag enables multiple-contact collision detection for geom pairs "
                                 "that use a general-purpose convex-convex collider e.g., mesh-mesh collisions")
        parser.add_argument("--max_real_time", type=float, required=False, default=None,
                            help="Maximum real time")
        parser.add_argument("--max_number_of_steps", type=float, required=False, default=None,
                            help="Maximum number of steps")
        parser.add_argument("--max_simulation_time", type=float, required=False, default=None,
                            help="Maximum simulation time")
        parser.add_argument("--multiverse_params", type=str, required=False, help="JSON string with multiverse' data")

        args = parser.parse_args()

        if args.multiverse_params is None:
            multiverse_params = {}
        else:
            try:
                multiverse_params = json.loads(args.multiverse_params.replace("'", '"'))
            except json.JSONDecodeError as e:
                print(f"Failed to parse {args.multiverse_params}: {str(e)}")
                multiverse_params = {}

        simulator = MultiverseMujocoConnector(file_path=args.file_path,
                                              use_mjx=args.use_mjx,
                                              headless=args.headless,
                                              real_time_factor=args.real_time_factor,
                                              step_size=args.step_size,
                                              integrator=args.integrator,
                                              noslip_iterations=args.noslip_iterations,
                                              noslip_tolerance=args.noslip_tolerance,
                                              cone=args.cone,
                                              impratio=args.impratio,
                                              multiccd=args.multiccd)
        if args.max_real_time is not None or args.max_number_of_steps is not None or args.max_simulation_time is not None:
            constraints = MultiverseSimulatorConstraints(max_real_time=args.max_real_time,
                                                         max_number_of_steps=args.max_number_of_steps,
                                                         max_simulation_time=args.max_simulation_time)
            simulator.start(constraints=constraints, run_in_thread=False)
        else:
            simulator.start(run_in_thread=False)
        simulator.run()  # TODO: Implement viewer using multiverse_params
        simulator.stop()
        time.sleep(1)  # Extra time to clean up
    except KeyboardInterrupt:
        time.sleep(1)  # Extra time to clean up
    sys.exit(0)


if __name__ == "__main__":
    main()
