#!/usr/bin/env python3

import os
import sys
import re
import subprocess
from typing import List, Dict, Any

from multiverse_launch import MultiverseLaunch
from utils import find_files, run_subprocess


def parse_mujoco(resources_paths: List[str], mujoco_data: Dict[str, Any]):
    worlds_path = find_files(resources_paths, mujoco_data["world"]["path"])
    mujoco_args = [f"--world_path={worlds_path}"]

    for entity_str in ["robots", "objects"]:
        if entity_str in mujoco_data:
            for entity_name in mujoco_data[entity_str]:
                if "path" in mujoco_data[entity_str][entity_name]:
                    mujoco_data[entity_str][entity_name]["path"] = find_files(resources_paths,
                                                                              mujoco_data[entity_str][entity_name][
                                                                                  "path"])
            entity_dict = mujoco_data[entity_str]
            mujoco_args.append(f"--{entity_str}={entity_dict}".replace(" ", ""))
    if "references" in mujoco_data:
        mujoco_args.append(f"--references={mujoco_data['references']}".replace(" ", ""))

    return mujoco_args


def parse_isaac_sim(resources_paths: List[str], isaac_sim_data: Dict[str, Any]):
    worlds_path = find_files(resources_paths, isaac_sim_data["world"]["path"])
    isaac_sim_args = [f"--world_path={worlds_path}"]

    for entity_str in ["robots", "objects"]:
        if entity_str in isaac_sim_data:
            for entity_name in isaac_sim_data[entity_str]:
                if "path" in isaac_sim_data[entity_str][entity_name]:
                    isaac_sim_data[entity_str][entity_name]["path"] = find_files(resources_paths,
                                                                                 isaac_sim_data[entity_str][
                                                                                     entity_name][
                                                                                     "path"])
            entity_dict = isaac_sim_data[entity_str]
            isaac_sim_args.append(f"--{entity_str}={entity_dict}".replace(" ", ""))

    return isaac_sim_args


class MultiverseSimulationLaunch(MultiverseLaunch):
    simulator_compilers = {
        "pymujoco": "mujoco_compiler",
        "isaac_sim": "isaac_sim_compiler",
    }

    def __init__(self):
        super().__init__()

    def run_simulations(self) -> List[subprocess.Popen]:
        processes: List[subprocess.Popen] = []
        for simulation_name, simulation_data in self.simulations.items():
            if "simulator" not in simulation_data or simulation_data["simulator"] not in self.simulator_compilers:
                continue

            compiler_name = self.simulator_compilers[simulation_data["simulator"]]
            result = self.run_simulator_compiler(compiler_name, simulation_name, simulation_data)
            processes.append(self.run_simulator(result, simulation_name, simulation_data))
        return processes

    def parse_simulator(self, simulation_data):
        if simulation_data["simulator"] == "pymujoco":
            return parse_mujoco(self.resources_paths, simulation_data)
        elif simulation_data["simulator"] == "isaac_sim":
            return parse_isaac_sim(self.resources_paths, simulation_data)
        else:
            raise NotImplementedError(f"Simulator {simulation_data['simulator']} not implemented")

    def parse_multiverse_params(self, world_name, simulation_name) -> List[str]:
        if self.multiverse_clients.get(simulation_name) is not None:
            multiverse_dict = {
                "host": self.multiverse_server["host"],
                "server_port": self.multiverse_server["port"],
                "client_port": self.multiverse_clients[simulation_name]["port"],
                "world_name": world_name,
                "simulation_name": simulation_name,
                "send": self.multiverse_clients[simulation_name].get("send", {}),
                "receive": self.multiverse_clients[simulation_name].get("receive", {}),
            }
            return [f"--multiverse_params={multiverse_dict}".replace(" ", "").replace("'", '"')]
        return []

    def run_simulator_compiler(self, compiler_name, simulation_name, simulation_data):
        cmd = self.parse_simulator(simulation_data)
        if os.name == "nt":
            current_file = os.path.abspath(__file__)
            multiverse_bin_dir = os.path.join(os.path.dirname(current_file), "..", "..", "..", "bin")
            simulator_compile_file = os.path.join(multiverse_bin_dir, f"{compiler_name}.py")
            cmd = [sys.executable, simulator_compile_file, f"--name={simulation_name}"] + cmd
        elif os.name == "posix":
            cmd = [f"{compiler_name}", f"--name={simulation_name}"] + cmd
        else:
            raise NotImplementedError(f"OS {os.name} not implemented")
        world = simulation_data.get("world")
        world_name = "world" if world is None else world["name"]
        cmd += self.parse_multiverse_params(world_name, simulation_name)
        cmd_str = " ".join(cmd)
        print(f'Execute "{cmd_str}"')
        return subprocess.run(cmd, capture_output=True, text=True)

    def run_simulator(self, compiler_result: subprocess.CompletedProcess, simulation_name,
                      simulation_data) -> subprocess.Popen:
        if compiler_result.returncode != 0:
            print(compiler_result.stderr)
            raise RuntimeError(f"Failed to compile {simulation_name}")

        scene_path = re.search(r"Scene:\s*([^\n]+)", compiler_result.stdout).group(1)
        if os.name == "nt":
            current_file = os.path.abspath(__file__)
            multiverse_bin_dir = os.path.join(os.path.dirname(current_file), "..", "..", "..", "bin")
            simulator_file = os.path.join(multiverse_bin_dir, f"{simulation_data['simulator']}.py")
            cmd = [sys.executable, simulator_file, f"--file_path={scene_path}"]
        elif os.name == "posix":
            cmd = [f"{simulation_data['simulator']}",  f"--file_path={scene_path}"]
        else:
            raise NotImplementedError(f"OS {os.name} not implemented")
        robots_group = re.search(r"Robots:\s*([^\n]+)", compiler_result.stdout)
        if robots_group is not None:
            robots_path = robots_group.group(1)
            cmd += [f"--robots_path={robots_path}"]
        for config_name, config_data in simulation_data.get("config", {}).items():
            cmd.append(f"--{config_name}={config_data}")

        if "viewer" in simulation_data:
            viewer = simulation_data["viewer"]
            read_objects = viewer.get("read", {})
            logging_interval = viewer.get("logging_interval", -1.0)
            save_log_path = viewer.get("save_log_path", "data.csv")
            cmd.append(f"--read_objects={read_objects}".replace(" ", ""))
            cmd.append(f"--logging_interval={logging_interval}")
            cmd.append(f"--save_log_path={save_log_path}")

        return run_subprocess(cmd)


def main():
    multiverse_launch = MultiverseSimulationLaunch()
    multiverse_launch.run_simulations()


if __name__ == "__main__":
    main()
