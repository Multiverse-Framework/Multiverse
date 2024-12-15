#!/usr/bin/env python3

"""Multiverse Isaac Sim Connector class"""
import os.path

from multiverse_simulator import MultiverseSimulator, MultiverseRenderer
from omni.isaac.lab.app import AppLauncher
from omni.isaac.kit import SimulationApp


class MultiverseIsaacSimRenderer(MultiverseRenderer):
    """Multiverse Isaac Sim Renderer class"""

    def __init__(self, app_launcher: AppLauncher):
        self._simulation_app = app_launcher.app
        super().__init__()

    def is_running(self) -> bool:
        return self.simulation_app.is_running()

    def close(self):
        self.simulation_app.close()

    @property
    def simulation_app(self) -> SimulationApp:
        return self._simulation_app


class MultiverseIsaacSimConnector(MultiverseSimulator):
    """Multiverse MuJoCo Connector class"""

    def __init__(self, file_path: str, **kwargs):
        self._file_path = file_path
        self.name = os.path.basename(file_path).split(".")[0]
        super().__init__(**kwargs)

        # Start the omniverse application
        simulation_app = SimulationApp({
            "width": 1280,
            "height": 720,
            "sync_loads": True,
            "headless": self.headless,
            "renderer": "RayTracedLighting",
            "hide_ui": False,
            "open_usd": self.file_path,
        })
        from omni.isaac.lab.sim import SimulationContext, SimulationCfg
        simulation_config = SimulationCfg(dt=0.001)
        self._simulation_context = SimulationContext(cfg=simulation_config)
        self._renderer = MultiverseIsaacSimRenderer(simulation_app)

    def start_callback(self):        
        pass

    def step_callback(self):
        self.simulation_context.step()

    def stop_callback(self):
        self.simulation_context.stop()

    def reset_callback(self):
        self.simulation_context.reset()

    def log_info(self, message: str):
        print(f"INFO: {message}")

    def log_warning(self, message: str):
        print(f"WARN: {message}")

    def log_error(self, message: str):
        print(f"ERROR: {message}")

    @property
    def file_path(self) -> str:
        return self._file_path

    @property
    def current_simulation_time(self) -> float:
        return self._current_number_of_steps * self.step_size

    @property
    def simulation_context(self) -> "SimulationContext":
        return self._simulation_context
