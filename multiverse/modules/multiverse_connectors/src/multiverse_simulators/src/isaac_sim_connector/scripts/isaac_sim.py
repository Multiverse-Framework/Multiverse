#!ISAAC_SIM_PATH/python.sh

import argparse
import sys
import json
import time
from typing import Any, Dict
from multiverse_simulator import MultiverseSimulatorConstraints
from isaac_sim_connector import MultiverseIsaacSimConnector


def load_json_str(json_str: str) -> Dict[str, Any]:
    try:
        return json.loads(json_str.replace("'", '"'))
    except json.JSONDecodeError as e:
        print(f"Failed to parse {json_str}: {str(e)}")
        return {}


if __name__ == "__main__":
    try:
        parser = argparse.ArgumentParser(description=f"Run the Isaac Sim Connector")
        parser.add_argument("--file_path", type=str, required=True, help=f"Path to the USD file")
        parser.add_argument("--robots_path", type=str, required=False, help="Paths to the robots' USD files")
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

        multiverse_params = load_json_str(args.multiverse_params) if args.multiverse_params is not None else {}
        robots_path = args.robots_path if args.robots_path is not None else ""

        simulator = MultiverseIsaacSimConnector(file_path=args.file_path,
                                                robots_path=robots_path,
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
        simulator.run()  # TODO: Implement viewer using multiverse_params
        simulator.stop()
        time.sleep(1)  # Extra time to clean up
    except KeyboardInterrupt:
        time.sleep(1)  # Extra time to clean up
    sys.exit(0)
