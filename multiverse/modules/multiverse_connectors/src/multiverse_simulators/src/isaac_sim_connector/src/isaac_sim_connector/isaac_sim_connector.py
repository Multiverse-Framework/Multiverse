#!/usr/bin/env python3

"""Multiverse Isaac Sim Connector class"""
import os.path
import shutil
from typing import Optional

import numpy
import torch
from isaaclab.app import AppLauncher

from multiverse_simulator import MultiverseSimulator, MultiverseRenderer, MultiverseViewer


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
    def simulation_app(self) -> "SimulationApp":
        return self._simulation_app


class MultiverseIsaacSimConnector(MultiverseSimulator):
    """Multiverse MuJoCo Connector class"""

    def __init__(
            self,
            world_path: str,
            robots_path: Optional[str] = None,
            objects_path: Optional[str] = None,
            joint_state: Optional[dict] = None,
            number_of_envs: int = 1,
            env_spacing: float = 2.0,
            viewer: Optional[MultiverseViewer] = None,
            headless: bool = False,
            real_time_factor: float = 1.0,
            step_size: float = 1E-3,
            **kwargs,
    ):
        self.name = os.path.basename(world_path).split(".")[0]
        super().__init__(viewer, number_of_envs, headless, real_time_factor, step_size, **kwargs)

        self._app_launcher = AppLauncher(headless=self.headless)

        import isaaclab.sim as sim_utils
        from isaaclab.terrains import TerrainImporterCfg
        from isaaclab.assets import ArticulationCfg, AssetBaseCfg
        from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
        from isaaclab.actuators import ImplicitActuatorCfg
        from isaaclab.sim import SimulationContext
        from isaaclab.utils import configclass
        from isaacsim.core.utils.extensions import enable_extension

        from pxr import Usd, UsdGeom, UsdPhysics

        world_stage = Usd.Stage.Open(world_path)

        customLayerData = world_stage.GetRootLayer().customLayerData
        if "multiverse_connector" in customLayerData:
            enable_extension("multiverse_connector") # Extension name

        @configclass
        class WorldCfg(InteractiveSceneCfg):
            # world asset
            world = AssetBaseCfg(
                prim_path="{ENV_REGEX_NS}/World",
                spawn=sim_utils.UsdFileCfg(
                    usd_path=world_path,
                ),
                init_state=AssetBaseCfg.InitialStateCfg(),
            )

        @configclass
        class SceneWithRobotsCfg(WorldCfg):
            pass

        if robots_path is not None:
            robots_stage = Usd.Stage.Open(robots_path)
            robots_prim = robots_stage.GetDefaultPrim()
            if robots_prim.IsValid():
                robots_xform = UsdGeom.Xform(robots_prim)
                robots_pos = robots_xform.GetLocalTransformation().ExtractTranslation()
                robots_quat = robots_xform.GetLocalTransformation().ExtractRotation().GetQuat()
                robots_joint_pos = {}
                robots_actuators = {}
                for prim in [prim for prim in robots_stage.TraverseAll() if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint)]:
                    prim_name = prim.GetName()
                    if joint_state is not None and prim_name in joint_state:
                        joint_pos = joint_state[prim_name]
                    else:
                        joint_pos = 0.0
                    if prim.HasAPI(UsdPhysics.DriveAPI):
                        drive_api = UsdPhysics.DriveAPI(prim, "angular" if prim.IsA(UsdPhysics.RevoluteJoint) else "linear")
                        robots_actuators[prim_name] = (drive_api.GetStiffnessAttr().Get(), drive_api.GetDampingAttr().Get())
                        if drive_api.GetStiffnessAttr().Get() > 0.0:
                            joint_pos = drive_api.GetTargetPositionAttr().Get()
                        # for attr in drive_api.GetSchemaAttributeNames():
                        #     attr = attr.replace("__INSTANCE_NAME__", "angular" if prim.IsA(UsdPhysics.RevoluteJoint) else "linear")
                        #     prim.RemoveProperty(attr)
                        # prim.RemoveAPI(UsdPhysics.DriveAPI, "angular" if prim.IsA(UsdPhysics.RevoluteJoint) else "linear")
                    else:
                        robots_actuators[prim_name] = (0.0, 0.0)
                    robots_joint_pos[prim_name] = joint_pos
                # robots_stage.GetRootLayer().Save()

                @configclass
                class SceneWithRobotsCfg(WorldCfg):
                    # robots articulation
                    robots = ArticulationCfg(
                        prim_path="{ENV_REGEX_NS}/Robot",
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
                            rot=(robots_quat.GetReal(), *robots_quat.GetImaginary()),
                            joint_pos=robots_joint_pos
                        ),
                        actuators={
                            joint_name: ImplicitActuatorCfg(
                                joint_names_expr=[joint_name],
                                stiffness=stiffness,
                                damping=damping,
                            )
                            for joint_name, (stiffness, damping) in robots_actuators.items()
                        },
                    )

        @configclass
        class SceneCfg(SceneWithRobotsCfg):
            pass

        if objects_path is not None:
            objects_stage = Usd.Stage.Open(objects_path)
            objects_prim = objects_stage.GetDefaultPrim()
            if objects_prim.IsValid():
                objects_xform = UsdGeom.Xform(objects_prim)
                objects_pos = objects_xform.GetLocalTransformation().ExtractTranslation()
                objects_quat = objects_xform.GetLocalTransformation().ExtractRotation().GetQuat()
                objects_joint_pos = {}
                for prim in [prim for prim in objects_stage.TraverseAll() if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint)]:
                    prim_name = prim.GetName()
                    if joint_state is not None and prim_name in joint_state:
                        joint_pos = joint_state[prim_name]
                    else:
                        joint_pos = 0.0
                        if prim.HasAPI(UsdPhysics.DriveAPI):
                            drive_api = UsdPhysics.DriveAPI(prim, "angular" if prim.IsA(UsdPhysics.RevoluteJoint) else "linear")
                            if drive_api.GetStiffnessAttr().Get() > 0.0:
                                joint_pos = drive_api.GetTargetPositionAttr().Get()

                    objects_joint_pos[prim.GetName()] = joint_pos

                @configclass
                class SceneCfg(SceneWithRobotsCfg):
                    # objects asset
                    objects = AssetBaseCfg(
                        prim_path="{ENV_REGEX_NS}/Objects",
                        spawn=sim_utils.UsdFileCfg(
                            usd_path=objects_path,
                        ),
                        init_state=AssetBaseCfg.InitialStateCfg(
                            pos=objects_pos,
                            rot=(objects_quat.GetReal(), *objects_quat.GetImaginary())
                        ),
                    )

        simulation_config = sim_utils.SimulationCfg(dt=self.step_size)
        self._simulation_context = SimulationContext(cfg=simulation_config)
        self.simulation_context.set_camera_view([2.5, 0.0, 4.0], [0.0, 0.0, 2.0])
        scene_cfg = SceneCfg(num_envs=number_of_envs, env_spacing=env_spacing)
        self._scene = InteractiveScene(scene_cfg)
        self._write_objects = {}
        self._read_objects = {}
        self._write_ids = {}
        self._read_ids = {}

    def start_callback(self):
        self._renderer = MultiverseIsaacSimRenderer(self.app_launcher)
        self.simulation_context.reset()

    def _process_objects(self, objects, ids_dict):
        """
        Process objects for updating `read_ids` or `write_ids`.

        :param objects: Dictionary of objects and attributes.
        :param ids_dict: Dictionary to store processed IDs.
        """
        attr_map = {
            "position": "body_pos_w",
            "quaternion": "body_quat_w",
            "joint_rvalue": "joint_pos",
            "joint_tvalue": "joint_pos",
            "joint_angular_velocity": "joint_vel",
            "joint_linear_velocity": "joint_vel",
            "cmd_joint_rvalue": "joint_pos_target",
            "cmd_joint_angular_velocity": "joint_vel_target",
            "cmd_joint_torque": "joint_effort_target",
            "cmd_joint_tvalue": "joint_pos_target",
            "cmd_joint_linear_velocity": "joint_vel_target",
            "cmd_joint_force": "joint_effort_target"
        }
        attr_size = {
            "body_pos_w": 3,
            "body_quat_w": 4,
            "joint_pos": 1,
            "joint_vel": 1,
            "qfrc_applied": 1,
            "joint_pos_target": 1,
            "joint_vel_target": 1,
            "joint_effort_target": 1
        }

        ids_dict.clear()
        entity_types = [entity_type for entity_type in ["robots"] if entity_type in self.scene.keys()]
        name_to_index = {"body": {}, "joint": {}}
        for entity_type in entity_types:
            entity = self.scene[entity_type]
            name_to_index["body"].update({
                **{name: (i, entity_type) for i, name in enumerate(entity.data.body_names)}
            })
            name_to_index["joint"].update({
                **{name: (i, entity_type) for i, name in enumerate(entity.data.joint_names)}
            })

        i = [0] * len(entity_types)  # Index counters for world and robots respectively

        for name, attrs in objects.items():
            for obj_type in name_to_index.keys():
                if name not in name_to_index[obj_type]:
                    continue

                index, entity_type = name_to_index[obj_type][name]
                entity_id = entity_types.index(entity_type)

                for attr_name in attrs:
                    if attr_name not in attr_map:
                        raise ValueError(f"Unknown attribute {attr_name} for {name}")

                    isaac_sim_attr_name = attr_map[attr_name]
                    if isaac_sim_attr_name not in ids_dict:
                        ids_dict[isaac_sim_attr_name] = [[[], []]] * len(entity_types)

                    ids_dict[isaac_sim_attr_name][entity_id][0].append(index)
                    ids_dict[isaac_sim_attr_name][entity_id][1].extend(
                        range(i[entity_id], i[entity_id] + attr_size[isaac_sim_attr_name]))
                    i[entity_id] += attr_size[isaac_sim_attr_name]

    def write_data_to_simulator(self, write_data: numpy.ndarray):
        entity_types = [entity_type for entity_type in ["robots"] if entity_type in self.scene.keys()]

        write_data_torch = torch.tensor(write_data).to("cuda").float()
        for entity_id, entity_type in enumerate(entity_types):
            entity = self.scene[entity_type]
            changed = {"joint_state": False, "joint_pos_target": False, "joint_vel_target": False, "joint_effort_target": False}
            joint_pos = entity.data.joint_pos
            joint_vel = entity.data.joint_vel
            joint_pos_target = entity.data.joint_pos_target
            joint_vel_target = entity.data.joint_vel_target
            joint_effort_target = entity.data.joint_effort_target
            for attr_name, indices in self._write_ids.items():
                entity_ids = indices[entity_id]
                if attr_name == "joint_pos":
                    joint_pos[:, entity_ids[0]] = write_data_torch[:, entity_ids[1]]
                    changed["joint_state"] = True
                elif attr_name == "joint_vel":
                    joint_vel[:, entity_ids[0]] = write_data_torch[:, entity_ids[1]]
                    changed["joint_state"] = True
                elif attr_name == "joint_pos_target":
                    joint_pos_target[:, entity_ids[0]] = write_data_torch[:, entity_ids[1]]
                    changed["joint_pos_target"] = True
                elif attr_name == "joint_vel_target":
                    joint_vel_target[:, entity_ids[0]] = write_data_torch[:, entity_ids[1]]
                    changed["joint_vel_target"] = True
                elif attr_name == "joint_effort_target":
                    joint_effort_target[:, entity_ids[0]] = write_data_torch[:, entity_ids[1]]
                    changed["joint_effort_target"] = True
            if changed["joint_state"]:
                entity.write_joint_state_to_sim(position=joint_pos, velocity=joint_vel)
            if changed["joint_pos_target"]:
                entity.set_joint_position_target(target=joint_pos_target)
            if changed["joint_vel_target"]:
                entity.set_joint_velocity_target(target=joint_vel_target)
            if changed["joint_effort_target"]:
                entity.set_joint_effort_target(target=joint_effort_target)

    def read_data_from_simulator(self, read_data: numpy.ndarray):
        entity_types = [entity_type for entity_type in ["robots"] if entity_type in self.scene.keys()]

        for attr, indices in self._read_ids.items():
            for entity_id, entity_type in enumerate(entity_types):
                entity = self.scene[entity_type]
                entity_ids = indices[entity_id]
                if len(entity_ids[0]) == 0:
                    continue
                attr_values = getattr(entity.data, attr).cpu().numpy()
                read_data[:, entity_ids[1]] = attr_values[:, entity_ids[0]].reshape(attr_values.shape[0], -1)

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
