#!/usr/bin/env python3

"""Multiverse Mujoco Connector class"""

import os
import xml.etree.ElementTree as ET

import jax
import mujoco
import mujoco.viewer
import numpy
from mujoco import mjx

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

    use_mjx: bool = False
    """Use MJX (https://mujoco.readthedocs.io/en/stable/mjx.html)"""

    def __init__(self, file_path: str, viewer: MultiverseViewer = None, number_of_instances: int = 1, **kwargs):
        self._file_path = file_path
        root = ET.parse(file_path).getroot()
        self.name = root.attrib.get("model", self.name)
        self.use_mjx = kwargs.get("use_mjx", False)
        super().__init__(viewer, number_of_instances, **kwargs)
        mujoco.mj_loadPluginLibrary(get_multiverse_connector_plugin())
        assert os.path.exists(self.file_path)
        self._mj_model = mujoco.MjModel.from_xml_path(filename=self.file_path)
        assert self._mj_model is not None
        self._mj_model.opt.timestep = self.step_size
        self._mj_data = mujoco.MjData(self._mj_model)
        mujoco.mj_resetDataKeyframe(self._mj_model, self._mj_data, 0)
        if self.use_mjx:
            qpos0 = numpy.array([self._mj_data.qpos for _ in range(number_of_instances)])
            qvel0 = numpy.array([self._mj_data.qvel for _ in range(number_of_instances)])
            act0 = numpy.array([self._mj_data.act for _ in range(number_of_instances)])
            ctrl0 = numpy.array([self._mj_data.ctrl for _ in range(number_of_instances)])
            self._mjx_model = mjx.put_model(self._mj_model)
            self._mjx_data = mjx.put_data(self._mj_model, self._mj_data)
            self._batch0 = jax.vmap(lambda qpos, qvel, act, ctrl:
                                    self._mjx_data.replace(qpos=qpos, qvel=qvel, act=act, ctrl=ctrl))(qpos0, qvel0, act0, ctrl0)
            self._batch = self._batch0
            self._jit_step = jax.jit(jax.vmap(mjx.step, in_axes=(None, 0)))

    def start_callback(self):
        if not self.headless:
            self._renderer = mujoco.viewer.launch_passive(self._mj_model, self._mj_data)
        else:
            self._renderer = MultiverseRenderer()

    def step_callback(self):
        if self.use_mjx:
            self._batch = self._jit_step(self._mjx_model, self._batch)
            if not self.headless:
                self._mj_data = mjx.get_data(self._mj_model, self._batch)
        else:
            mujoco.mj_step(self._mj_model, self._mj_data)

    def reset_callback(self):
        if self.use_mjx:
            self._batch = self._batch0
        else:
            mujoco.mj_resetDataKeyframe(self._mj_model, self._mj_data, 0)

    def write_data_to_simulator(self, write_data: numpy.ndarray):
        if not self.use_mjx and write_data.shape[0] > 1:
            raise NotImplementedError("Multiple instances for non MJX is not supported yet")
        if self.use_mjx:
            qpos = numpy.array(self._batch.qpos)
            qvel = numpy.array(self._batch.qvel)
            qfrc_applied = numpy.array(self._batch.qfrc_applied)
            ctrl = numpy.array(self._batch.ctrl)
        else:
            qpos = self._mj_data.qpos
            qvel = self._mj_data.qvel
            qfrc_applied = self._mj_data.qfrc_applied
            ctrl = self._mj_data.ctrl
        for instance_id, data in enumerate(write_data):
            i = 0
            for name, attrs in self._viewer.send_objects.items():
                for attr_name in attrs.keys():
                    if attr_name in {"joint_rvalue", "joint_tvalue"}:
                        joint_id = self._mj_model.joint(name).id
                        if self.use_mjx:
                            qpos[instance_id][joint_id] = data[i]
                        else:
                            qpos[joint_id] = data[i]
                    elif attr_name in {"joint_angular_velocity", "joint_linear_velocity"}:
                        joint_id = self._mj_model.joint(name).id
                        if self.use_mjx:
                            qvel[instance_id][joint_id] = data[i]
                        else:
                            qvel[joint_id] = data[i]
                    elif attr_name in {"joint_torque", "joint_force"}:
                        joint_id = self._mj_model.joint(name).id
                        if self.use_mjx:
                            qfrc_applied[instance_id][joint_id] = data[i]
                        else:
                            qfrc_applied[joint_id] = data[i]
                    elif attr_name in {"cmd_joint_rvalue", "cmd_joint_angular_velocity", "cmd_joint_torque",
                                       "cmd_joint_tvalue", "cmd_joint_linear_velocity", "cmd_joint_force"}:
                        actuator_id = self._mj_data.actuator(name).id
                        if self.use_mjx:
                            ctrl[instance_id][actuator_id] = data[i]
                        else:
                            ctrl[actuator_id] = data[i]
                    else:
                        raise ValueError(f"Unknown attribute {attr_name} for object {name}")
                    i += 1
            if i != len(data):
                raise ValueError(f"Data length mismatch (expected {len(data)}, got {i})")
            if self.use_mjx:
                self._batch = self._batch.replace(qpos=qpos,
                                                  qvel=qvel,
                                                  qfrc_applied=qfrc_applied,
                                                  ctrl=ctrl)
            else:
                self._mj_data.qpos = qpos
                self._mj_data.qvel = qvel
                self._mj_data.qfrc_applied = qfrc_applied
                self._mj_data.ctrl = ctrl

    def read_data_from_simulator(self, read_data: numpy.ndarray):
        if not self.use_mjx and read_data.shape[0] > 1:
            raise NotImplementedError("Multiple instances for non MJX is not supported yet")
        if self.use_mjx:
            xpos = numpy.array(self._batch.xpos)
            xquat = numpy.array(self._batch.xquat)
            qpos = numpy.array(self._batch.qpos)
            qvel = numpy.array(self._batch.qvel)
            qfrc_applied = numpy.array(self._batch.qfrc_applied)
            ctrl = numpy.array(self._batch.ctrl)
        else:
            xpos = self._mj_data.xpos
            xquat = self._mj_data.xquat
            qpos = self._mj_data.qpos
            qvel = self._mj_data.qvel
            qfrc_applied = self._mj_data.qfrc_applied
            ctrl = self._mj_data.ctrl
        for instance_id, data in enumerate(read_data):
            i = 0
            for name, attrs in self._viewer.receive_objects.items():
                for attr_name in attrs.keys():
                    if attr_name == "position":
                        body_id = self._mj_model.body(name).id
                        if self.use_mjx:
                            read_data[instance_id][i:i + 3] = xpos[instance_id][body_id]
                        else:
                            read_data[instance_id][i:i + 3] = xpos[body_id]
                        i += 3
                    elif attr_name == "quaternion":
                        body_id = self._mj_model.body(name).id
                        if self.use_mjx:
                            read_data[instance_id][i:i + 4] = xquat[instance_id][body_id]
                        else:
                            read_data[instance_id][i:i + 4] = xquat[body_id]
                        i += 4
                    elif attr_name in {"joint_rvalue", "joint_tvalue"}:
                        joint_id = self._mj_model.joint(name).id
                        if self.use_mjx:
                            read_data[instance_id][i] = qpos[instance_id][joint_id]
                        else:
                            read_data[instance_id][i] = qpos[joint_id]
                        i += 1
                    elif attr_name in {"joint_angular_velocity", "joint_linear_velocity"}:
                        joint_id = self._mj_model.joint(name).id
                        if self.use_mjx:
                            read_data[instance_id][i] = qvel[instance_id][joint_id]
                        else:
                            read_data[instance_id][i] = qvel[joint_id]
                        i += 1
                    elif attr_name in {"joint_torque", "joint_force"}:
                        joint_id = self._mj_model.joint(name).id
                        if self.use_mjx:
                            read_data[instance_id][i] = qfrc_applied[instance_id][joint_id]
                        else:
                            read_data[instance_id][i] = qfrc_applied[joint_id]
                        i += 1
                    elif attr_name in {"cmd_joint_rvalue", "cmd_joint_angular_velocity", "cmd_joint_torque",
                                       "cmd_joint_tvalue", "cmd_joint_linear_velocity", "cmd_joint_force"}:
                        actuator_id = self._mj_model.actuator(name).id
                        if self.use_mjx:
                            read_data[instance_id][i] = ctrl[instance_id][actuator_id]
                        else:
                            read_data[instance_id][i] = ctrl[actuator_id]
                        i += 1
                    else:
                        self.log_error(f"Unknown attribute {attr_name} for object {name}")
            if i != len(data):
                raise ValueError(f"Data length mismatch (expected {len(data)}, got {i})")

    @property
    def file_path(self) -> str:
        return self._file_path

    @property
    def current_simulation_time(self) -> float:
        return self._mj_data.time if not self.use_mjx else self._batch.time[0]

    @property
    def renderer(self):
        return self._renderer
