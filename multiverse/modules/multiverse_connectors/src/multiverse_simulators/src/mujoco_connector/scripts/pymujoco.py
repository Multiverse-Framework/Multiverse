#!/usr/bin/env python3

import time
from multiverse_client_py import MultiverseMetaData
from multiverse_simulator import MultiverseSimulatorConstraints
from mujoco_connector import MultiverseMujocoConnector

import argparse
import sys


def main():
    try:
        parser = argparse.ArgumentParser(description="Run the Mujoco Connector")
        parser.add_argument("--file_path", type=str, required=True, help="Path to the Mujoco XML file")
        parser.add_argument("--headless", required=False, action='store_true', help="Run in headless mode")
        parser.add_argument("--real_time_factor", type=float, required=False, default=1.0, help="Real time factor")
        parser.add_argument("--step_size", type=float, required=False, default=0.01, help="Step size")
        parser.add_argument("--host", type=str, required=False, default="tcp://127.0.0.1", help="Multiverse Host")
        parser.add_argument("--server_port", type=str, required=False, default="7000", help="Multiverse Server Port")
        parser.add_argument("--client_port", type=str, required=False, default="8000", help="Multiverse Client Port")
        parser.add_argument("--max_real_time", type=float, required=False, default=None, help="Maximum real time")
        parser.add_argument("--max_number_of_steps", type=float, required=False, default=None,
                            help="Maximum number of steps")
        parser.add_argument("--max_simulation_time", type=float, required=False, default=None,
                            help="Maximum simulation time")

        args = parser.parse_args()

        simulator = MultiverseMujocoConnector(file_path=args.file_path,
                                              host=args.host,
                                              server_port=args.server_port,
                                              client_port=args.client_port,
                                              meta_data=MultiverseMetaData(),
                                              headless=args.headless,
                                              real_time_factor=args.real_time_factor,
                                              step_size=args.step_size)
        if args.max_real_time is not None or args.max_number_of_steps is not None or args.max_simulation_time is not None:
            constraints = MultiverseSimulatorConstraints(max_real_time=args.max_real_time,
                                                         max_number_of_steps=args.max_number_of_steps,
                                                         max_simulation_time=args.max_simulation_time)
            simulator.start(constraints=constraints, run_in_thread=False)
        else:
            simulator.start(run_in_thread=False)
        simulator.run()
        simulator.stop()
        time.sleep(1)  # Extra time to clean up
    except KeyboardInterrupt:
        time.sleep(1)  # Extra time to clean up
    sys.exit(0)


if __name__ == "__main__":
    main()
