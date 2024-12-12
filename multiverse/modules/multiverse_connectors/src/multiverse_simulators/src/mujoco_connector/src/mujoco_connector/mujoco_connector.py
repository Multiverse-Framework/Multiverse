#!/usr/bin/env python3

"""Multiverse Mujoco Connector class"""

import os
import xml.etree.ElementTree as ET

import mujoco
import mujoco.viewer
from mujoco import mjx
import jax
import numpy

from multiverse_simulator import MultiverseSimulator, MultiverseRenderer, MultiverseViewer
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

    use_mjx: bool = False
    """Use MJX (https://mujoco.readthedocs.io/en/stable/mjx.html)"""

    def __init__(self, file_path: str, **kwargs):
        self._batch = None
        self._jit_step = None
        self._mjx_data = None
        self._mjx_model = None
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
            self._mjx_model = mjx.put_model(self.mj_model)
            self._mjx_data = mjx.put_data(self.mj_model, self.mj_data)

            def ctrl_func(ctrl: jax.numpy.ndarray):
                ctrl = self._mjx_data.ctrl
                for object_name, attrs in view.write_objects.items():
                    for attr in attrs:
                        if attr.name in {"cmd_joint_rvalue", "cmd_joint_angular_velocity", "cmd_joint_torque",
                                         "cmd_joint_tvalue", "cmd_joint_linear_velocity", "cmd_joint_force"}:
                            actuator = self.mj_data.actuator(object_name)
                            ctrl[actuator.id] = attr.values
                        else:
                            self.log_error(f"Unknown attribute {attr.name} for object {object_name}")

            self._batch = jax.vmap(lambda write_data: self._mjx_data.replace(ctrl=ctrl_func(write_data)))(x=self._viewer.write_data))
            self._jit_step = jax.jit(jax.vmap(mjx.step, in_axes=(None, 0)))
        if not self.headless:
            self._renderer = mujoco.viewer.launch_passive(self.mj_model, self.mj_data)
        else:
            self._renderer = MultiverseRenderer()

    def step_callback(self):
        if self.use_mjx:
            self._batch = self._jit_step(self._mjx_model, self._batch)
        else:
            mujoco.mj_step(self.mj_model, self.mj_data)

    def reset_callback(self):
        if self.use_mjx:
            pass  # TODO: Implement reset_callback for MJX
        else:
            mujoco.mj_resetDataKeyframe(self.mj_model, self.mj_data, 0)

    def write_data(self, in_data: numpy.ndarray):
        if self.number_of_instances != in_data.shape[0]:
            raise ValueError(f"Data length mismatch (expected {self.number_of_instances}, got {in_data.shape[0]})")
        if self.use_mjx:
            self._write_data = jax.numpy.array(in_data)
        else:
            if self.number_of_instances > 1:
                raise NotImplementedError("Multiple instances are not supported yet")
            self._write_data = in_data[0]
            self._write_data_to_simulator(self._write_data)

    def _write_data_to_simulator(self, write_data: numpy.ndarray | jax.numpy.ndarray):
        i = 0
        act_ids = []
        for name, attrs in self._viewer.write_objects.items():
            for attr in attrs:
                if attr.name in {"cmd_joint_rvalue", "cmd_joint_angular_velocity", "cmd_joint_torque",
                                 "cmd_joint_tvalue", "cmd_joint_linear_velocity", "cmd_joint_force"}:
                    act_id = self.mj_model.actuator(name).id
                    act_ids.append(act_id)
                    i += 1
                else:
                    self.log_error(f"Unknown attribute {attr.name} for object {name}")
        if i != len(write_data):
            self.log_error(f"Data length mismatch (expected {len(write_data)}, got {i})")
        else:
            if self.use_mjx:
                ctrl = self._mjx_data.ctrl.at[jax.numpy.array(act_ids)].set(write_data)
                self._mjx_data.replace(ctrl=ctrl)
            else:
                self.mj_data.ctrl[act_ids] = write_data

    def read_data(self, out_data: numpy.ndarray):
        if self.number_of_instances != out_data.shape[0]:
            raise ValueError(f"Data length mismatch (expected {self.number_of_instances}, got {out_data.shape[0]})")
        if self.use_mjx:
            out_data[:] = self._read_data
        else:
            if self.number_of_instances > 1:
                raise NotImplementedError("Multiple instances are not supported yet")
            i = 0
            for name, attrs in self._viewer.read_objects.items():
                for attr in attrs:
                    if attr.name in {"joint_rvalue", "joint_tvalue"}:
                        joint = self.mj_data.joint(name)
                        out_data[0][i] = joint.qpos[0]
                        i += 1
                    elif attr.name in {"joint_angular_velocity", "joint_linear_velocity"}:
                        joint = self.mj_data.joint(name)
                        out_data[0][i] = joint.qvel[0]
                        i += 1
                    elif attr.name in {"joint_torque", "joint_force"}:
                        joint = self.mj_data.joint(name)
                        out_data[0][i] = joint.qfrc_applied[0]
                        i += 1
                    else:
                        self.log_error(f"Unknown attribute {attr.name} for object {name}")
            if i != len(out_data[0]):
                self.log_error(f"Data length mismatch (expected {len(out_data[0])}, got {i})")

    def _read_data_from_simulator(self):
        i = 0
        for name, attrs in self._viewer.read_objects.items():
            for attr in attrs:
                if attr.name in {"joint_rvalue", "joint_tvalue"}:
                    joint = self.mj_data.joint(name)
                    out_data[0][i] = joint.qpos[0]
                    i += 1
                elif attr.name in {"joint_angular_velocity", "joint_linear_velocity"}:
                    joint = self.mj_data.joint(name)
                    out_data[0][i] = joint.qvel[0]
                    i += 1
                elif attr.name in {"joint_torque", "joint_force"}:
                    joint = self.mj_data.joint(name)
                    out_data[0][i] = joint.qfrc_applied[0]
                    i += 1
                else:
                    self.log_error(f"Unknown attribute {attr.name} for object {name}")
        if i != len(out_data[0]):
            self.log_error(f"Data length mismatch (expected {len(out_data[0])}, got {i})")

    @property
    def file_path(self) -> str:
        return self._file_path

    @property
    def current_simulation_time(self) -> float:
        return self.mj_data.time

    @property
    def renderer(self):
        return self._renderer
