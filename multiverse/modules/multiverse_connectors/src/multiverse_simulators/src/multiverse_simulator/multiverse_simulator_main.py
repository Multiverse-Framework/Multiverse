#!/usr/bin/env python3

import argparse
import sys
import time
from multiverse_simulator import MultiverseSimulator, MultiverseSimulatorConstraints
from typing import Type
from .utils import str_to_dict


def multiverse_simulator_main(Simulator: Type[MultiverseSimulator]):
    try:
        parser = argparse.ArgumentParser(description=f"Run the {Simulator.name} Connector")
        parser.add_argument("--file_path", type=str, required=True, help=f"Path to the {Simulator.ext} file")
        parser.add_argument("--headless", required=False, action='store_true', help="Run in headless mode")
        parser.add_argument("--real_time_factor", type=float, required=False, default=1.0, help="Real time factor")
        parser.add_argument("--step_size", type=float, required=False, default=0.01, help="Step size")
        parser.add_argument("--max_real_time", type=float, required=False, default=None, help="Maximum real time")
        parser.add_argument("--max_number_of_steps", type=float, required=False, default=None,
                            help="Maximum number of steps")
        parser.add_argument("--max_simulation_time", type=float, required=False, default=None,
                            help="Maximum simulation time")
        parser.add_argument("--multiverse_params", type=str, required=False, help="JSON string with multiverse' data")

        args = parser.parse_args()

        multiverse_params = str_to_dict(args.multiverse_params)

        simulator = MultiverseSimulator(file_path=args.file_path,
                                        headless=args.headless,
                                        real_time_factor=args.real_time_factor,
                                        step_size=args.step_size)
        if args.max_real_time is not None or args.max_number_of_steps is not None or args.max_simulation_time is not None:
            constraints = MultiverseSimulatorConstraints(max_real_time=args.max_real_time,
                                                         max_number_of_steps=args.max_number_of_steps,
                                                         max_simulation_time=args.max_simulation_time)
            simulator.start(constraints=constraints, simulate_in_thread=False)
        else:
            simulator.start(simulate_in_thread=False)
        simulator.run()  # TODO: Implement viewer using multiverse_params
        simulator.stop()
        time.sleep(1)  # Extra time to clean up
    except KeyboardInterrupt:
        time.sleep(1)  # Extra time to clean up
    sys.exit(0)
