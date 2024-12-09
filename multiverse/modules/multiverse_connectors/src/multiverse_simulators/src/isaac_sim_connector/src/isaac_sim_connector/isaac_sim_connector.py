#!/usr/bin/env python3

"""Multiverse Isaac Sim Connector class"""
import os.path

from multiverse_simulator import MultiverseSimulator, MultiverseRenderer
from isaacsim import SimulationApp


class MultiverseIsaacSimRenderer(MultiverseRenderer):
    """Multiverse Isaac Sim Renderer class"""

    def __init__(self, simulation_app: SimulationApp):
        self._simulation_app = simulation_app
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

    def start_callback(self):
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

        import carb
        import omni
        from omni.isaac.core import World
        from omni.isaac.core.utils import stage
        from omni.physx.bindings import _physx

        self._omni = omni

        settings = carb.settings.acquire_settings_interface()
        settings.set(_physx.SETTING_RESET_ON_STOP, False)
        self._world = World()
        self.log_info("Loading stage...")
        while stage.is_stage_loading():
            simulation_app.update()
        self.log_info("Loading Complete")
        self.world.play()
        self.world.reset()
        self._renderer = MultiverseIsaacSimRenderer(simulation_app)

    def step_callback(self):
        self.world.step(render=not self.headless)

    def stop_callback(self):
        self.world.stop()
        super().stop_callback()

    def reset_callback(self):
        self.world.reset()

    def log_info(self, message: str):
        self.omni.kit.app.get_app().print_and_log(f"INFO: {message}")

    def log_warning(self, message: str):
        self.omni.kit.app.get_app().print_and_log(f"WARN: {message}")

    def log_error(self, message: str):
        self.omni.kit.app.get_app().print_and_log(f"ERROR: {message}")

    @property
    def file_path(self) -> str:
        return self._file_path

    @property
    def current_simulation_time(self) -> float:
        return self._current_number_of_steps * self.step_size

    @property
    def scene_registry(self) -> "SceneRegistry":
        return self.world.scene._scene_registry

    @property
    def world(self) -> "World":
        return self._world

    @property
    def omni(self):
        return self._omni
