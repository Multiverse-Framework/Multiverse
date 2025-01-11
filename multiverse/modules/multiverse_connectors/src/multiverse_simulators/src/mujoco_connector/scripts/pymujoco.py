#!/usr/bin/env python3

import time
from typing import List, Dict, Any

from multiverse_simulator import MultiverseSimulatorConstraints, MultiverseViewer, str_to_dict
from mujoco_connector import MultiverseMujocoConnector

import argparse
import re
import sys


def parse_unknown_args(unknown: List[str]) -> Dict[str, Any]:
    """
    Parse the list of unknown arguments from argparse into a dictionary.
    This version specifically handles arguments in the form '--key=value'.
    """
    arg_dict = {}
    pattern = re.compile(r'--([^=]+)=(.*)')
    for arg in unknown:
        match = pattern.match(arg)
        if match:
            key, value = match.groups()
            # Convert value to boolean if it's 'True' or 'False'
            if value == 'True':
                value = True
            elif value == 'False':
                value = False
            # Check if the key already exists
            if key in arg_dict:
                if isinstance(arg_dict[key], list):
                    arg_dict[key].append(value)
                else:
                    arg_dict[key] = [arg_dict[key], value]
            else:
                arg_dict[key] = value
        else:
            # If the argument doesn't match the pattern, treat it as a flag
            arg_dict[arg.lstrip('-')] = True
    return arg_dict


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
        parser.add_argument("--max_real_time", type=float, required=False, default=None,
                            help="Maximum real time")
        parser.add_argument("--max_number_of_steps", type=float, required=False, default=None,
                            help="Maximum number of steps")
        parser.add_argument("--max_simulation_time", type=float, required=False, default=None,
                            help="Maximum simulation time")
        parser.add_argument("--multiverse_params", type=str, required=False, default=None, help="JSON string with multiverse' data")
        parser.add_argument("--read_objects", type=str, required=False, default=None, help="JSON string with read objects")
        parser.add_argument("--logging_interval", type=float, required=False, default=-1.0, help="Logging interval in seconds")
        parser.add_argument("--save_log_path", type=str, required=False, default="data.csv", help="Path to save log file")

        args, unknown = parser.parse_known_args()

        # Convert unknown arguments into a dictionary
        unknown_args_dict = parse_unknown_args(unknown)

        multiverse_params = str_to_dict(args.multiverse_params)

        read_objects = str_to_dict(args.read_objects)

        viewer = MultiverseViewer(read_objects=read_objects) if read_objects is not None else None
        simulator = MultiverseMujocoConnector(file_path=args.file_path,
                                              viewer=viewer,
                                              use_mjx=args.use_mjx,
                                              headless=args.headless,
                                              real_time_factor=args.real_time_factor,
                                              step_size=args.step_size,
                                              **unknown_args_dict)
        if args.max_real_time is not None or args.max_number_of_steps is not None or args.max_simulation_time is not None:
            constraints = MultiverseSimulatorConstraints(max_real_time=args.max_real_time,
                                                         max_number_of_steps=args.max_number_of_steps,
                                                         max_simulation_time=args.max_simulation_time)
            simulator.start(constraints=constraints, run_in_thread=False)
        else:
            simulator.start(run_in_thread=False)
        simulator.run()
        simulator.stop()
        if viewer is not None and args.logging_interval > 0.0:
            simulator._viewer.logger.save_data(args.save_log_path)
        time.sleep(1)  # Extra time to clean up
    except KeyboardInterrupt:
        time.sleep(1)  # Extra time to clean up
    sys.exit(0)


if __name__ == "__main__":
    main()
