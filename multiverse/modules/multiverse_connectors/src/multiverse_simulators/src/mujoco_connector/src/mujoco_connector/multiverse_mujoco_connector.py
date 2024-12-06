#!/usr/bin/env python3

"""Multiverse Mujoco Connector class"""

import os

import mujoco
import mujoco.viewer
from multiverse_client_py import MultiverseMetaData

from multiverse_simulator import MultiverseSimulator, MultiverseViewer


class MultiverseMujocoConnector(MultiverseSimulator):
    """Multiverse MuJoCo Connector class"""

    mj_model: mujoco.MjModel
    """MuJoCo model"""

    mj_data = mujoco.MjData
    """MuJoCo data"""

    def __init__(self,
                 file_path: str,
                 host: str,
                 server_port: str,
                 client_port: str,
                 meta_data: MultiverseMetaData,
                 **kwargs):
        self._file_path = file_path
        super().__init__(host, server_port, client_port, meta_data, **kwargs)

    def start_callback(self):
        assert os.path.exists(self.file_path)
        self.mj_model = mujoco.MjModel.from_xml_path(filename=self.file_path)
        assert self.mj_model is not None
        self.mj_model.opt.timestep = self.step_size
        self.mj_data = mujoco.MjData(self.mj_model)
        if not self.headless:
            self._viewer = mujoco.viewer.launch_passive(self.mj_model, self.mj_data)
        else:
            self._viewer = MultiverseViewer()

    def step_callback(self):
        mujoco.mj_step(self.mj_model, self.mj_data)
        self.log_info(f"Step {self.current_number_of_steps}")

    def reset_callback(self):
        mujoco.mj_resetData(self.mj_model, self.mj_data)

    @property
    def file_path(self) -> str:
        return self._file_path

    @property
    def current_simulation_time(self) -> float:
        return self.mj_data.time

    @property
    def viewer(self):
        return self._viewer