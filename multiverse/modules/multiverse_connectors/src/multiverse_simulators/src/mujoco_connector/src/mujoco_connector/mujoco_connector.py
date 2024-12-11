#!/usr/bin/env python3

"""Multiverse Mujoco Connector class"""

import os
import xml.etree.ElementTree as ET

import mujoco
import mujoco.viewer
import numpy

from multiverse_simulator import MultiverseSimulator, MultiverseRenderer
from .utills import get_multiverse_connector_plugin


class MultiverseMujocoRenderer(MultiverseRenderer):
    """Multiverse Isaac Sim Renderer class"""

    def __init__(self, mj_viewer: mujoco.viewer):
        self._mj_viewer = mj_viewer
        super().__init__()

    def is_running(self) -> bool:
        return self.mj_viewer.is_running()

    def close(self):
        self.mj_viewer.close()

    @property
    def mj_viewer(self) -> mujoco.viewer:
        return self._mj_viewer


class MultiverseMujocoConnector(MultiverseSimulator):
    """Multiverse MuJoCo Connector class"""

    mj_model: mujoco.MjModel
    """MuJoCo model"""

    mj_data = mujoco.MjData
    """MuJoCo data"""

    mjx_model: "mjx.Model" = None
    """MJX model"""

    mjx_data: "mjx.Data" = None
    """MJX data"""

    use_mjx: bool = False
    """Use MJX (https://mujoco.readthedocs.io/en/stable/mjx.html)"""

    _mjx = None

    def __init__(self, file_path: str, **kwargs):
        self._file_path = file_path
        root = ET.parse(file_path).getroot()
        self.name = root.attrib.get("model", self.name)
        self.use_mjx = kwargs.get("use_mjx", False)
        super().__init__(**kwargs)

    def start_callback(self):
        mujoco.mj_loadPluginLibrary(get_multiverse_connector_plugin())
        assert os.path.exists(self.file_path)
        self.mj_model = mujoco.MjModel.from_xml_path(filename=self.file_path)
        assert self.mj_model is not None
        self.mj_model.opt.timestep = self.step_size
        self.mj_data = mujoco.MjData(self.mj_model)
        if self.use_mjx:
            from mujoco import mjx
            self._mjx = mjx
            self.mjx_model = self.mjx.put_model(self.mj_model)
            self.mjx_data = self.mjx.make_data(self.mj_model)
        if not self.headless:
            self._renderer = mujoco.viewer.launch_passive(self.mj_model, self.mj_data)
        else:
            self._renderer = MultiverseRenderer()

    def step_callback(self):
        if self.use_mjx:
            self.mjx.step(self.mjx_model, self.mjx_data)
        else:
            mujoco.mj_step(self.mj_model, self.mj_data)

    def reset_callback(self):
        if self.use_mjx:
            pass  # TODO: Implement reset_callback for MJX
        else:
            mujoco.mj_resetDataKeyframe(self.mj_model, self.mj_data, 0)

    def write_data(self, in_data: numpy.ndarray):
        if self.use_mjx:
            pass  # TODO: Implement write_data for MJX
        else:
            i = 0
            for name, attrs in self._viewer.send_objects.items():
                for attr in attrs:
                    if attr.name in {"cmd_joint_rvalue", "cmd_joint_angular_velocity", "cmd_joint_torque",
                                     "cmd_joint_tvalue", "cmd_joint_linear_velocity", "cmd_joint_force"}:
                        actuator = self.mj_data.actuator(name)
                        actuator.ctrl[0] = in_data[i]
                        i += 1
                    else:
                        self.log_error(f"Unknown attribute {attr.name} for object {name}")
            if i != len(in_data):
                self.log_error(f"Data length mismatch (expected {len(in_data)}, got {i})")

    def read_data(self, out_data: numpy.ndarray):
        if self.use_mjx:
            pass
        else:
            i = 0
            for name, attrs in self._viewer.receive_objects.items():
                for attr in attrs:
                    if attr.name in {"joint_rvalue", "joint_tvalue"}:
                        joint = self.mj_data.joint(name)
                        out_data[i] = joint.qpos[0]
                        i += 1
                    elif attr.name in {"joint_angular_velocity", "joint_linear_velocity"}:
                        joint = self.mj_data.joint(name)
                        out_data[i] = joint.qvel[0]
                        i += 1
                    elif attr.name in {"joint_torque", "joint_force"}:
                        joint = self.mj_data.joint(name)
                        out_data[i] = joint.qfrc_applied[0]
                        i += 1
                    else:
                        self.log_error(f"Unknown attribute {attr.name} for object {name}")
            if i != len(out_data):
                self.log_error(f"Data length mismatch (expected {len(out_data)}, got {i})")

    @property
    def file_path(self) -> str:
        return self._file_path

    @property
    def current_simulation_time(self) -> float:
        return self.mj_data.time

    @property
    def renderer(self):
        return self._renderer

    @property
    def mjx(self):
        return self._mjx
