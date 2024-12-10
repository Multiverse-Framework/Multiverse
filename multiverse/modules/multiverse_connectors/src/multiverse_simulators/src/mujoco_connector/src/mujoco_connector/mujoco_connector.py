#!/usr/bin/env python3

"""Multiverse Mujoco Connector class"""

import os

import mujoco
import mujoco.viewer

from .utills import get_multiverse_connector_plugin
from multiverse_simulator import MultiverseSimulator, MultiverseRenderer
import xml.etree.ElementTree as ET


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
        if not self.use_mjx:
            mujoco.mj_resetDataKeyframe(self.mj_model, self.mj_data, 0)

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
