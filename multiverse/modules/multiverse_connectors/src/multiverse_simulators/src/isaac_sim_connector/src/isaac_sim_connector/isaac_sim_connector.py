#!/usr/bin/env python3

"""Multiverse Isaac Sim Connector class"""
import os.path

from multiverse_simulator import MultiverseSimulator, MultiverseRenderer
from omni.isaac.lab.app import AppLauncher
from omni.isaac.kit import SimulationApp
from omni.isaac.lab.terrains import TerrainImporterCfg


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

    def __init__(
            self,
            world_path: str,
            robots_path: str,
            number_of_envs: int = 1,
            env_spacing: float = 2.0,
            **kwargs,
    ):
        self.name = os.path.basename(world_path).split(".")[0]
        super().__init__(**kwargs)

        self._app_launcher = AppLauncher(headless=self.headless)

        import omni.isaac.lab.sim as sim_utils
        from omni.isaac.lab.assets import ArticulationCfg, AssetBaseCfg
        from omni.isaac.lab.scene import InteractiveScene, InteractiveSceneCfg
        from omni.isaac.lab.sim import SimulationContext
        from omni.isaac.lab.utils import configclass
        from pxr import Usd, UsdGeom

        world_stage = Usd.Stage.Open(world_path)
        world_prim = world_stage.GetDefaultPrim()
        if world_prim.IsValid():
            @configclass
            class WorldCfg(InteractiveSceneCfg):
                terrain = TerrainImporterCfg(prim_path="/World/ground", terrain_type="plane", debug_vis=False)

                # lights
                dome_light = AssetBaseCfg(
                    prim_path="/World/Light",
                    spawn=sim_utils.DomeLightCfg(
                        intensity=3000.0, color=(0.75, 0.75, 0.75)
                    ),
                )

                # world articulation
                world = ArticulationCfg(
                    spawn=sim_utils.UsdFileCfg(
                        usd_path=world_path,
                        rigid_props=sim_utils.RigidBodyPropertiesCfg(
                            rigid_body_enabled=True,
                            max_linear_velocity=1000.0,
                            max_angular_velocity=1000.0,
                            max_depenetration_velocity=100.0,
                            enable_gyroscopic_forces=True,
                        ),
                        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                            enabled_self_collisions=True,
                            solver_position_iteration_count=4,
                            solver_velocity_iteration_count=0,
                            sleep_threshold=0.005,
                            stabilization_threshold=0.001,
                        ),
                    ),
                ).replace(prim_path="{ENV_REGEX_NS}/World")
        else:
            @configclass
            class WorldCfg(InteractiveSceneCfg):
                terrain = TerrainImporterCfg(prim_path="/World/ground", terrain_type="plane", debug_vis=False)

                # lights
                dome_light = AssetBaseCfg(
                    prim_path="/World/Light",
                    spawn=sim_utils.DomeLightCfg(
                        intensity=3000.0, color=(0.75, 0.75, 0.75)
                    ),
                )

        robots_stage = Usd.Stage.Open(robots_path)
        robots_prim = robots_stage.GetDefaultPrim()
        if robots_prim.IsValid():
            robots_xform = UsdGeom.Xform(robots_prim)
            robots_pos = robots_xform.GetLocalTransformation().ExtractTranslation()
            robots_quat = robots_xform.GetLocalTransformation().ExtractRotation().GetQuat()

            @configclass
            class SceneCfg(WorldCfg):
                # robots articulation
                robots = ArticulationCfg(
                    spawn=sim_utils.UsdFileCfg(
                        usd_path=robots_path,
                        rigid_props=sim_utils.RigidBodyPropertiesCfg(
                            rigid_body_enabled=True,
                            max_linear_velocity=1000.0,
                            max_angular_velocity=1000.0,
                            max_depenetration_velocity=100.0,
                            enable_gyroscopic_forces=True,
                        ),
                        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                            enabled_self_collisions=True,
                            solver_position_iteration_count=4,
                            solver_velocity_iteration_count=0,
                            sleep_threshold=0.005,
                            stabilization_threshold=0.001,
                        ),
                    ),
                    init_state=ArticulationCfg.InitialStateCfg(
                        pos=robots_pos,
                        rot=(robots_quat.GetReal(), *robots_quat.GetImaginary())
                    ),
                    actuators={},
                ).replace(prim_path="{ENV_REGEX_NS}/Robot")
        else:
            @configclass
            class SceneCfg(WorldCfg):
                pass

        simulation_config = sim_utils.SimulationCfg(dt=self.step_size)
        self._simulation_context = SimulationContext(cfg=simulation_config)
        self.simulation_context.set_camera_view([2.5, 0.0, 4.0], [0.0, 0.0, 2.0])
        scene_cfg = SceneCfg(num_envs=number_of_envs, env_spacing=env_spacing)
        self._scene = InteractiveScene(scene_cfg)

    def start_callback(self):
        self._renderer = MultiverseIsaacSimRenderer(self.app_launcher)
        self.simulation_context.reset()

    def step_callback(self):
        self.simulation_context.step()
        self.scene.update(self.step_size)

    def stop_callback(self):
        self.simulation_context.stop()

    def reset_callback(self):
        self.scene.reset()

    def log_info(self, message: str):
        print(f"INFO: {message}")

    def log_warning(self, message: str):
        print(f"WARN: {message}")

    def log_error(self, message: str):
        print(f"ERROR: {message}")

    @property
    def current_simulation_time(self) -> float:
        return self._current_number_of_steps * self.step_size

    @property
    def app_launcher(self) -> AppLauncher:
        return self._app_launcher

    @property
    def simulation_context(self) -> "SimulationContext":
        return self._simulation_context

    @property
    def scene(self) -> "InteractiveScene":
        return self._scene
