#!/usr/bin/env python3

"""Multiverse Mujoco Connector class"""

import os

os.environ['XLA_FLAGS'] = '--xla_gpu_triton_gemm_any=true'
import xml.etree.ElementTree as ET
from typing import Optional, List, Set, Dict

import jax
import mujoco
import mujoco.viewer
import numpy
from mujoco import mjx

from multiverse_simulator import (MultiverseSimulator, MultiverseRenderer, MultiverseViewer,
                                  MultiverseFunction, MultiverseFunctionResult)
from .utills import get_multiverse_connector_plugins


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

    def __init__(self,
                 file_path: str,
                 viewer: Optional[MultiverseViewer] = None,
                 number_of_envs: int = 1,
                 headless: bool = False,
                 real_time_factor: float = 1.0,
                 step_size: float = 1E-3,
                 callbacks: Optional[List[MultiverseFunction]] = None,
                 use_mjx: bool = False,
                 **kwargs):
        self._file_path = file_path
        root = ET.parse(file_path).getroot()
        self.name = root.attrib.get("model", self.name)
        self.use_mjx = use_mjx
        super().__init__(viewer, number_of_envs, headless, real_time_factor, step_size, callbacks, **kwargs)
        for plugin in get_multiverse_connector_plugins():
            plugin_name = os.path.basename(plugin)
            if "multiverse_connector" in plugin_name:
                mujoco.mj_loadPluginLibrary(plugin)
        assert os.path.exists(self.file_path)
        self._mj_spec = mujoco.MjSpec.from_file(filename=self.file_path)

        self._mj_spec.option.integrator = getattr(mujoco.mjtIntegrator,
                                                  f"mjINT_{kwargs.get('integrator', 'RK4')}")
        self._mj_spec.option.noslip_iterations = int(kwargs.get('noslip_iterations', 0))
        self._mj_spec.option.noslip_tolerance = float(kwargs.get('noslip_tolerance', 1E-6))
        self._mj_spec.option.cone = getattr(mujoco.mjtCone,
                                            f"mjCONE_{kwargs.get('cone', 'PYRAMIDAL')}")
        self._mj_spec.option.impratio = float(kwargs.get('impratio', 1))

        self._mj_model = self._mj_spec.compile()
        assert self._mj_model is not None
        self._mj_model.opt.timestep = self.step_size
        if kwargs.get('multiccd', False):
            self._mj_model.opt.enableflags |= mujoco.mjtEnableBit.mjENBL_MULTICCD
        if kwargs.get('nativeccd', False):
            self._mj_model.opt.enableflags |= mujoco.mjtEnableBit.mjENBL_NATIVECCD
        if kwargs.get('energy', True):
            self._mj_model.opt.enableflags |= mujoco.mjtEnableBit.mjENBL_ENERGY
        self._mj_data = mujoco.MjData(self._mj_model)

        mujoco.mj_resetDataKeyframe(self._mj_model, self._mj_data, 0)
        if self.use_mjx:
            self._mjx_model = mjx.put_model(self._mj_model)
            self._mjx_data = mjx.put_data(self._mj_model, self._mj_data)
            qpos0 = numpy.array([self._mj_data.qpos for _ in range(number_of_envs)])
            qvel0 = numpy.array([self._mj_data.qvel for _ in range(number_of_envs)])
            act0 = numpy.array([self._mj_data.act for _ in range(number_of_envs)])
            ctrl0 = numpy.array([self._mj_data.ctrl for _ in range(number_of_envs)])
            self._batch = jax.vmap(lambda qpos, qvel, act, ctrl:
                                   self._mjx_data.replace(qpos=qpos, qvel=qvel, act=act, ctrl=ctrl))(qpos0, qvel0,
                                                                                                     act0, ctrl0)
            self._jit_step = jax.jit(jax.vmap(mjx.step, in_axes=(None, 0)))

    def start_callback(self):
        if not self.headless:
            self._renderer = mujoco.viewer.launch_passive(self._mj_model, self._mj_data)
        else:
            self._renderer = MultiverseRenderer()

    def _process_objects(self, objects, ids_dict):
        """
        Process objects for updating `read_ids` or `write_ids`.

        :param objects: Dictionary of objects and attributes.
        :param ids_dict: Dictionary to store processed IDs.
        """
        attr_map = {
            "position": "xpos",
            "quaternion": "xquat",
            "joint_rvalue": "qpos",
            "joint_tvalue": "qpos",
            "joint_angular_velocity": "qvel",
            "joint_linear_velocity": "qvel",
            "joint_torque": "qfrc_applied",
            "joint_force": "qfrc_applied",
            "cmd_joint_rvalue": "ctrl",
            "cmd_joint_angular_velocity": "ctrl",
            "cmd_joint_torque": "ctrl",
            "cmd_joint_tvalue": "ctrl",
            "cmd_joint_linear_velocity": "ctrl",
            "cmd_joint_force": "ctrl",
            "energy": "energy",
        }
        attr_size = {
            "xpos": 3,
            "xquat": 4,
            "qpos": 1,
            "qvel": 1,
            "qfrc_applied": 1,
            "ctrl": 1,
            "energy": 2,
        }
        i = 0
        ids_dict.clear()
        for name, attrs in objects.items():
            for attr_name in attrs.keys():
                mj_attr_name = attr_map[attr_name]
                if mj_attr_name not in ids_dict:
                    ids_dict[mj_attr_name] = [[], []]

                if attr_name in {"position", "quaternion", "energy"}:
                    mj_attr_id = self._mj_model.body(name).id
                    if attr_name == "energy" and name != "world":
                        raise NotImplementedError("Not supported")
                elif attr_name in {"joint_rvalue", "joint_tvalue"}:
                    mj_attr_id = self._mj_model.joint(name).qposadr[0]
                elif attr_name in {"joint_angular_velocity", "joint_linear_velocity", "joint_torque", "joint_force"}:
                    mj_attr_id = self._mj_model.joint(name).dofadr[0]
                elif attr_name in {"cmd_joint_rvalue", "cmd_joint_angular_velocity", "cmd_joint_torque",
                                   "cmd_joint_tvalue", "cmd_joint_linear_velocity", "cmd_joint_force"}:
                    mj_attr_id = self._mj_data.actuator(name).id
                else:
                    raise ValueError(f"Unknown attribute {attr_name} for {name}")

                ids_dict[mj_attr_name][0].append(mj_attr_id)
                ids_dict[mj_attr_name][1] += [j for j in range(i, i + attr_size[mj_attr_name])]
                i += attr_size[mj_attr_name]

    def step_callback(self):
        if self.use_mjx:
            self._batch = self._jit_step(self._mjx_model, self._batch)
            if not self.headless:
                self._mj_data = mjx.get_data(self._mj_model, self._batch)
        else:
            mujoco.mj_step(self._mj_model, self._mj_data)

    def reset_callback(self):
        mujoco.mj_resetDataKeyframe(self._mj_model, self._mj_data, 0)
        if self.use_mjx:
            number_of_envs = self._batch.time.shape[0]
            qpos0 = numpy.array([self._mj_data.qpos for _ in range(number_of_envs)])
            qvel0 = numpy.array([self._mj_data.qvel for _ in range(number_of_envs)])
            act0 = numpy.array([self._mj_data.act for _ in range(number_of_envs)])
            ctrl0 = numpy.array([self._mj_data.ctrl for _ in range(number_of_envs)])
            self._batch = jax.vmap(lambda qpos, qvel, act, ctrl:
                                   self._mjx_data.replace(qpos=qpos, qvel=qvel, act=act, ctrl=ctrl))(qpos0, qvel0,
                                                                                                     act0, ctrl0)

    def write_data_to_simulator(self, write_data: numpy.ndarray):
        if not self.use_mjx and write_data.shape[0] > 1:
            raise NotImplementedError("Multiple environments for non MJX is not supported yet")
        if self.use_mjx:
            batch_data = {}
            for attr, indices in self._write_ids.items():
                if attr in {"xpos", "xquat"}:
                    for i, body_id in enumerate(indices[0]):
                        jntid = self._mj_model.body(body_id).jntadr[0]
                        jnt = self._mj_model.jnt(jntid)
                        assert jnt.type == mujoco.mjtJoint.mjJNT_FREE
                        qpos_adr = jnt.qposadr[0]
                        batch_data["qpos"] = numpy.array(self._batch.qpos)
                        if attr == "xpos":  # TODO: Check it again
                            batch_data["qpos"][:, qpos_adr:qpos_adr + 3] = write_data[:, indices[1]][3 * i:3 * i + 3]
                        elif attr == "xquat":
                            batch_data["qpos"][:, qpos_adr + 3:qpos_adr + 7] = write_data[:, indices[1]][
                                                                               4 * i:4 * i + 4]
                elif attr == "energy":
                    raise NotImplementedError("Not supported")
                else:
                    batch_data[attr] = numpy.array(getattr(self._batch, attr))
                    batch_data[attr][:, indices[0]] = write_data[:, indices[1]]
            self._batch = self._batch.replace(**batch_data)
        else:
            for attr, indices in self._write_ids.items():
                if attr in {"xpos", "xquat"}:
                    for i, body_id in enumerate(indices[0]):
                        jntid = self._mj_model.body(body_id).jntadr[0]
                        jnt = self._mj_model.jnt(jntid)
                        assert jnt.type == mujoco.mjtJoint.mjJNT_FREE
                        qpos_adr = jnt.qposadr[0]
                        if attr == "xpos":
                            self._mj_data.qpos[qpos_adr:qpos_adr + 3] = write_data[0][indices[1][3 * i:3 * i + 3]]
                        elif attr == "xquat":
                            self._mj_data.qpos[qpos_adr + 3:qpos_adr + 7] = write_data[0][indices[1][4 * i:4 * i + 4]]
                elif attr == "energy":
                    raise NotImplementedError("Not supported")
                else:
                    getattr(self._mj_data, attr)[indices[0]] = write_data[0][indices[1]]

    def read_data_from_simulator(self, read_data: numpy.ndarray):
        if not self.use_mjx and read_data.shape[0] > 1:
            raise NotImplementedError("Multiple environments for non MJX is not supported yet")
        if self.use_mjx:
            for attr, indices in self._read_ids.items():
                if attr == "energy":
                    read_data[:, indices[1]] = self._mj_data.energy
                else:
                    attr_values = getattr(self._batch, attr)
                    read_data[:, indices[1]] = attr_values[:, indices[0]].reshape(attr_values.shape[0], -1)
        else:
            for attr, indices in self._read_ids.items():
                if attr == "energy":
                    read_data[0][indices[1]] = self._mj_data.energy
                else:
                    attr_values = getattr(self._mj_data, attr)
                    read_data[0][indices[1]] = attr_values[indices[0]].reshape(-1)

    def _fix_prefix_and_recompile(self, body_spec: mujoco.MjsBody, dummy_prefix: str, body_name: str):
        body_spec.name = body_name
        try:
            for body_child in (body_spec.bodies +
                               body_spec.joints +
                               body_spec.geoms +
                               body_spec.sites):
                body_child.name = body_child.name.replace(dummy_prefix, "")
        except ValueError:
            self.log_warning(f"Failed to resolve body_spec for {body_name}, this is a bug from MuJoCo")
            self._mj_model, self._mj_data = self._mj_spec.recompile(self._mj_model, self._mj_data)
            for body_child in (body_spec.bodies +
                               body_spec.joints +
                               body_spec.geoms +
                               body_spec.sites):
                body_child.name = body_child.name.replace(dummy_prefix, "")
        self._mj_model, self._mj_data = self._mj_spec.recompile(self._mj_model, self._mj_data)
        for key in self._mj_spec.keys:
            if key.name != "home":
                key.delete()
        self._mj_model, self._mj_data = self._mj_spec.recompile(self._mj_model, self._mj_data)
        self._renderer._sim().load(self._mj_model, self._mj_data, "")

    @property
    def file_path(self) -> str:
        return self._file_path

    @property
    def current_simulation_time(self) -> float:
        return self._mj_data.time if not self.use_mjx else self._batch.time[0]

    @property
    def renderer(self):
        return self._renderer

    @MultiverseSimulator.multiverse_callback
    def get_all_body_names(self) -> MultiverseFunctionResult:
        result = [self._mj_model.body(body_id).name for body_id in
                  range(self._mj_model.nbody)]
        return MultiverseFunctionResult(
            type=MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION,
            info="Getting all body names",
            result=result
        )

    @MultiverseSimulator.multiverse_callback
    def get_body(self, body_name: str) -> MultiverseFunctionResult:
        body_id = mujoco.mj_name2id(m=self._mj_model, type=mujoco.mjtObj.mjOBJ_BODY, name=body_name)
        if body_id == -1:
            return MultiverseFunctionResult(
                type=MultiverseFunctionResult.ResultType.FAILURE_WITHOUT_EXECUTION,
                info=f"Body {body_name} not found"
            )
        body = self._mj_data.body(body_id)
        return MultiverseFunctionResult(
            type=MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION,
            info=f"Getting body id of {body_name}",
            result=body
        )

    @MultiverseSimulator.multiverse_callback
    def get_body_position(self, body_name: str) -> MultiverseFunctionResult:
        get_body = self.get_body(body_name)
        if get_body.type != MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION:
            return get_body
        body = get_body.result
        return MultiverseFunctionResult(
            type=MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION,
            info=f"Getting body position of {body_name}",
            result=body.xpos
        )

    @MultiverseSimulator.multiverse_callback
    def get_body_quaternion(self, body_name: str) -> MultiverseFunctionResult:
        get_body = self.get_body(body_name)
        if get_body.type != MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION:
            return get_body
        body = get_body.result
        return MultiverseFunctionResult(
            type=MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION,
            info=f"Getting body quaternion (WXYZ) of {body_name}",
            result=body.xquat
        )

    @MultiverseSimulator.multiverse_callback
    def get_bodies_positions(self, body_names: List[str]) -> MultiverseFunctionResult:
        result = {}
        for body_name in body_names:
            get_body_position = self.get_body_position(body_name)
            if get_body_position.type != MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION:
                return get_body_position
            result[body_name] = get_body_position.result
        return MultiverseFunctionResult(
            type=MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION,
            info=f"Getting bodies positions of {body_names}",
            result=result
        )

    @MultiverseSimulator.multiverse_callback
    def get_bodies_quaternions(self, body_names: List[str]) -> MultiverseFunctionResult:
        result = {}
        for body_name in body_names:
            get_body_quaternion = self.get_body_quaternion(body_name)
            if get_body_quaternion.type != MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION:
                return get_body_quaternion
            result[body_name] = get_body_quaternion.result
        return MultiverseFunctionResult(
            type=MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION,
            info=f"Getting bodies quaternions of {body_names}",
            result=result
        )

    @MultiverseSimulator.multiverse_callback
    def get_body_joints(self, body_name):
        get_body = self.get_body(body_name)
        if get_body.type != MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION:
            return get_body
        body = get_body.result
        body_id = body.id
        jntids = self._mj_model.body(body_id).jntadr
        joints = [self._mj_model.joint(jntid) for jntid in jntids if jntid != -1]
        return MultiverseFunctionResult(
            type=MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION,
            info=f"Getting body {body_name} joints",
            result=joints
        )

    @MultiverseSimulator.multiverse_callback
    def set_body_position(self, body_name: str, position: numpy.ndarray) -> MultiverseFunctionResult:
        get_body_joints = self.get_body_joints(body_name)
        if get_body_joints.type != MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION:
            return get_body_joints
        joints = get_body_joints.result
        if len(joints) != 1:
            return MultiverseFunctionResult(
                type=MultiverseFunctionResult.ResultType.FAILURE_WITHOUT_EXECUTION,
                info=f"Body {body_name} doesn't have exactly one joint"
            )
        joint = joints[0]
        if joint.type != mujoco.mjtJoint.mjJNT_FREE:
            return MultiverseFunctionResult(
                type=MultiverseFunctionResult.ResultType.FAILURE_WITHOUT_EXECUTION,
                info=f"Body {body_name} joint is not free"
            )
        qpos_adr = joint.qposadr[0]
        if numpy.isclose(self._mj_data.qpos[qpos_adr:qpos_adr + 3], position).all():
            return MultiverseFunctionResult(
                type=MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION,
                info=f"Body {body_name} is already at position {position}"
            )
        self._mj_data.qpos[qpos_adr:qpos_adr + 3] = position
        mujoco.mj_step1(self._mj_model, self._mj_data)
        return MultiverseFunctionResult(
            type=MultiverseFunctionResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_DATA,
            info=f"Set body {body_name} to position {position}"
        )

    @MultiverseSimulator.multiverse_callback
    def set_bodies_positions(self, bodies_positions: Dict[str, numpy.ndarray]) -> MultiverseFunctionResult:
        for body_name, position in bodies_positions.items():
            set_body_position = self.set_body_position(body_name, position)
            if set_body_position.type not in [MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION,
                                              MultiverseFunctionResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_DATA]:
                return set_body_position
        return MultiverseFunctionResult(
            type=MultiverseFunctionResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_DATA,
            info=f"Set bodies positions of {bodies_positions}"
        )

    @MultiverseSimulator.multiverse_callback
    def set_body_quaternion(self, body_name: str, quaternion: numpy.ndarray) -> MultiverseFunctionResult:
        get_body_joints = self.get_body_joints(body_name)
        if get_body_joints.type != MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION:
            return get_body_joints
        joints = get_body_joints.result
        if len(joints) != 1:
            return MultiverseFunctionResult(
                type=MultiverseFunctionResult.ResultType.FAILURE_WITHOUT_EXECUTION,
                info=f"Body {body_name} doesn't have exactly one joint"
            )
        joint = joints[0]
        if joint.type != mujoco.mjtJoint.mjJNT_FREE:
            return MultiverseFunctionResult(
                type=MultiverseFunctionResult.ResultType.FAILURE_WITHOUT_EXECUTION,
                info=f"Body {body_name} joint is not free"
            )
        qpos_adr = joint.qposadr[0]
        if numpy.isclose(self._mj_data.qpos[qpos_adr + 3:qpos_adr + 7], quaternion).all():
            return MultiverseFunctionResult(
                type=MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION,
                info=f"Body {body_name} is already at quaternion {quaternion}"
            )
        self._mj_data.qpos[qpos_adr + 3:qpos_adr + 7] = quaternion
        mujoco.mj_step1(self._mj_model, self._mj_data)
        return MultiverseFunctionResult(
            type=MultiverseFunctionResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_DATA,
            info=f"Set body {body_name} to quaternion (WXYZ) {quaternion}"
        )

    @MultiverseSimulator.multiverse_callback
    def set_bodies_quaternions(self, bodies_quaternions: Dict[str, numpy.ndarray]) -> MultiverseFunctionResult:
        for body_name, quaternion in bodies_quaternions.items():
            set_body_quaternion = self.set_body_quaternion(body_name, quaternion)
            if set_body_quaternion.type not in [MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION,
                                                MultiverseFunctionResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_DATA]:
                return set_body_quaternion
        return MultiverseFunctionResult(
            type=MultiverseFunctionResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_DATA,
            info=f"Set bodies quaternions of {bodies_quaternions}"
        )

    @MultiverseSimulator.multiverse_callback
    def get_all_joint_names(self, joint_types: Optional[List[mujoco.mjtJoint]] = None) -> MultiverseFunctionResult:
        if joint_types is None:
            joint_types = [mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE]
        result = [self._mj_model.joint(joint_id).name for joint_id in
                  range(self._mj_model.njnt) if self._mj_model.joint(joint_id).type in joint_types]
        return MultiverseFunctionResult(
            type=MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION,
            info="Getting all joint names",
            result=result
        )

    @MultiverseSimulator.multiverse_callback
    def get_joint(self, joint_name: str,
                  allowed_joint_types: Optional[mujoco.mjtJoint] = None) -> MultiverseFunctionResult:
        if allowed_joint_types is None:
            allowed_joint_types = [mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE]
        joint_id = mujoco.mj_name2id(m=self._mj_model, type=mujoco.mjtObj.mjOBJ_JOINT, name=joint_name)
        if joint_id == -1:
            return MultiverseFunctionResult(
                type=MultiverseFunctionResult.ResultType.FAILURE_WITHOUT_EXECUTION,
                info=f"Joint {joint_name} not found"
            )
        if self._mj_model.joint(joint_id).type not in allowed_joint_types:
            return MultiverseFunctionResult(
                type=MultiverseFunctionResult.ResultType.FAILURE_WITHOUT_EXECUTION,
                info=f"Joint {joint_name} does not have allowed joint types {allowed_joint_types}"
            )
        joint = self._mj_data.joint(joint_id)
        return MultiverseFunctionResult(
            type=MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION,
            info=f"Getting joint id of {joint_name}",
            result=joint
        )

    @MultiverseSimulator.multiverse_callback
    def get_joint_value(self, joint_name: str) -> MultiverseFunctionResult:
        get_joint = self.get_joint(joint_name)
        if get_joint.type != MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION:
            return get_joint
        joint = get_joint.result
        return MultiverseFunctionResult(
            type=MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION,
            info=f"Getting joint value of {joint_name}",
            result=joint.qpos[0]
        )

    @MultiverseSimulator.multiverse_callback
    def get_joints_values(self, joint_names: List[str]) -> MultiverseFunctionResult:
        result = {}
        for joint_name in joint_names:
            joint_value = self.get_joint_value(joint_name)
            if joint_value.type != MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION:
                return joint_value
            result[joint_name] = joint_value.result
        return MultiverseFunctionResult(
            type=MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION,
            info=f"Getting joints values of {joint_names}",
            result=result
        )

    @MultiverseSimulator.multiverse_callback
    def set_joint_value(self, joint_name: str, value: float) -> MultiverseFunctionResult:
        get_joint = self.get_joint(joint_name)
        if get_joint.type != MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION:
            return get_joint
        joint = get_joint.result
        if numpy.isclose(joint.qpos[0], value):
            return MultiverseFunctionResult(
                type=MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION,
                info=f"Joint {joint_name} is already at value {value}"
            )
        joint.qpos[0] = value
        mujoco.mj_step1(self._mj_model, self._mj_data)
        return MultiverseFunctionResult(
            type=MultiverseFunctionResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_DATA,
            info=f"Set joint {joint_name} to value {value}"
        )

    @MultiverseSimulator.multiverse_callback
    def set_joints_values(self, joints_values: Dict[str, float]) -> MultiverseFunctionResult:
        for joint_name, value in joints_values.items():
            set_joint_value = self.set_joint_value(joint_name, value)
            if set_joint_value.type not in [MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION,
                                            MultiverseFunctionResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_DATA]:
                return set_joint_value
        return MultiverseFunctionResult(
            type=MultiverseFunctionResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_DATA,
            info=f"Set joints values of {joints_values}"
        )

    @MultiverseSimulator.multiverse_callback
    def attach(self,
               body_1_name: str,
               body_2_name: Optional[str] = None,
               relative_position: Optional[numpy.ndarray] = None,
               relative_quaternion: Optional[numpy.ndarray] = None) -> MultiverseFunctionResult:
        body_1_id = mujoco.mj_name2id(m=self._mj_model, type=mujoco.mjtObj.mjOBJ_BODY, name=body_1_name)
        if body_1_id == -1:
            return MultiverseFunctionResult(
                type=MultiverseFunctionResult.ResultType.FAILURE_BEFORE_EXECUTION_ON_MODEL,
                info=f"Body 1 {body_1_name} not found"
            )
        if body_2_name is not None:
            body_2_id = mujoco.mj_name2id(m=self._mj_model, type=mujoco.mjtObj.mjOBJ_BODY, name=body_2_name)
            if body_2_id == -1:
                return MultiverseFunctionResult(
                    type=MultiverseFunctionResult.ResultType.FAILURE_BEFORE_EXECUTION_ON_MODEL,
                    info=f"Body 2 {body_2_name} not found"
                )
        else:
            body_2_id = 0
            body_2_name = mujoco.mj_id2name(m=self._mj_model, type=mujoco.mjtObj.mjOBJ_BODY, id=body_2_id)
        if body_1_id == body_2_id:
            return MultiverseFunctionResult(
                type=MultiverseFunctionResult.ResultType.FAILURE_BEFORE_EXECUTION_ON_MODEL,
                info="Body 1 and body 2 are the same"
            )

        body_1_xpos = self._mj_data.body(body_1_id).xpos
        body_1_xquat = self._mj_data.body(body_1_id).xquat
        body_2_xpos = self._mj_data.body(body_2_id).xpos
        body_2_xquat = self._mj_data.body(body_2_id).xquat

        body_1_in_2_pos = numpy.zeros(3)
        body_1_in_2_quat = numpy.zeros(4)

        body_2_neq_quat = numpy.zeros(4)
        mujoco.mju_negQuat(body_2_neq_quat, body_2_xquat)
        mujoco.mju_sub3(body_1_in_2_pos, body_1_xpos, body_2_xpos)
        mujoco.mju_rotVecQuat(body_1_in_2_pos, body_1_in_2_pos, body_2_neq_quat)
        mujoco.mju_mulQuat(body_1_in_2_quat, body_2_neq_quat, body_1_xquat)

        body_1 = self._mj_model.body(body_1_id)
        if relative_position is not None:
            if len(relative_position) != 3 or any(not isinstance(x, (int, float)) for x in relative_position):
                return MultiverseFunctionResult(
                    type=MultiverseFunctionResult.ResultType.FAILURE_BEFORE_EXECUTION_ON_MODEL,
                    info=f"Invalid relative position {relative_position}"
                )
        else:
            if body_1.parentid[0] == body_2_id:
                relative_position = body_1.pos
            else:
                relative_position = body_1_in_2_pos

        if relative_quaternion is not None:
            if len(relative_quaternion) != 4 or any(not isinstance(x, (int, float)) for x in relative_quaternion):
                return MultiverseFunctionResult(
                    type=MultiverseFunctionResult.ResultType.FAILURE_BEFORE_EXECUTION_ON_MODEL,
                    info=f"Invalid relative quaternion {relative_quaternion}"
                )
        else:
            if body_1.parentid[0] == body_2_id:
                relative_quaternion = body_1.quat
            else:
                relative_quaternion = body_1_in_2_quat

        if (body_1.parentid[0] == body_2_id and
                numpy.isclose(body_1.pos, relative_position).all() and
                numpy.isclose(body_1.quat, relative_quaternion).all()):
            return MultiverseFunctionResult(
                type=MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION,
                info=f"Body 1 {body_1_name} is already attached to body 2 {body_2_name}"
            )

        body_1_spec = self._mj_spec.find_body(body_1_name)
        if body_1_spec is None:
            self.log_warning(f"Body 1 {body_1_name} not found in the model specification, this is a bug from MuJoCo")
            body_1_spec = next(body for body in self._mj_spec.bodies if body.name == body_1_name)
        body_2_spec = self._mj_spec.find_body(body_2_name)
        if body_2_spec is None:
            self.log_warning(f"Body 2 {body_2_name} not found in the model specification, this is a bug from MuJoCo")
            body_2_spec = next(body for body in self._mj_spec.bodies if body.name == body_2_name)
        first_joint: mujoco.MjsJoint = body_1_spec.first_joint()
        if first_joint is not None and first_joint.type == mujoco.mjtJoint.mjJNT_FREE:
            first_joint.delete()
        body_2_frame = body_2_spec.add_frame()
        dummy_prefix = "AVeryDumbassPrefixThatIsUnlikelyToBeUsedBecauseMuJoCoRequiresIt"
        body_1_spec_new = body_2_frame.attach_body(body_1_spec, dummy_prefix, "")
        body_1_spec_new.pos = relative_position
        body_1_spec_new.quat = relative_quaternion
        self._mj_spec.detach_body(body_1_spec)
        self._fix_prefix_and_recompile(body_1_spec_new, dummy_prefix, body_1_name)
        mujoco.mj_step1(self._mj_model, self._mj_data)

        return MultiverseFunctionResult(
            type=MultiverseFunctionResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_MODEL,
            info=f"Attached body 1 {body_1_name} to body 2 {body_2_name} "
                 f"at relative position {relative_position}, relative quaternion {relative_quaternion}"
        )

    @MultiverseSimulator.multiverse_callback
    def detach(self, body_name: str, add_freejoint: bool = True) -> MultiverseFunctionResult:
        body_id = mujoco.mj_name2id(m=self._mj_model, type=mujoco.mjtObj.mjOBJ_BODY, name=body_name)
        if body_id == -1:
            return MultiverseFunctionResult(
                type=MultiverseFunctionResult.ResultType.FAILURE_BEFORE_EXECUTION_ON_MODEL,
                info=f"Body {body_name} not found"
            )

        parent_body_id = self._mj_model.body(body_id).parentid[0]
        if parent_body_id == 0:
            return MultiverseFunctionResult(
                type=MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION,
                info=f"Body {body_name} is already detached"
            )

        absolute_position = self._mj_data.body(body_id).xpos
        absolute_quaternion = self._mj_data.body(body_id).xquat
        parent_body_name = self._mj_model.body(parent_body_id).name
        body_spec = self._mj_spec.find_body(body_name)
        dummy_prefix = "AVeryDumbassPrefixThatIsUnlikelyToBeUsedBecauseMuJoCoRequiresIt"
        body_spec_new = self._mj_spec.worldbody.add_frame().attach_body(body_spec, dummy_prefix, "")
        body_spec_new.pos = absolute_position
        body_spec_new.quat = absolute_quaternion
        if add_freejoint:
            body_spec_new.add_freejoint()
        self._mj_spec.detach_body(body_spec)
        self._fix_prefix_and_recompile(body_spec_new, dummy_prefix, body_name)
        mujoco.mj_step1(self._mj_model, self._mj_data)

        return MultiverseFunctionResult(
            type=MultiverseFunctionResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_MODEL,
            info=f"Detached body {body_name} from body {parent_body_name}"
        )

    @MultiverseSimulator.multiverse_callback
    def get_children_ids(self, body_id: int) -> Set[int]:
        children_ids = set()
        for child_body_id in range(body_id + 1, self._mj_model.nbody):
            if self._mj_model.body(child_body_id).parentid[0] == body_id:
                children_ids.add(child_body_id)
            else:
                break
        return children_ids

    @MultiverseSimulator.multiverse_callback
    def get_contact_bodies(self, body_name: str, including_children: bool = True) -> MultiverseFunctionResult:
        body_id = mujoco.mj_name2id(m=self._mj_model, type=mujoco.mjtObj.mjOBJ_BODY, name=body_name)
        if body_id == -1:
            return MultiverseFunctionResult(
                type=MultiverseFunctionResult.ResultType.FAILURE_WITHOUT_EXECUTION,
                info=f"Body {body_name} not found"
            )

        body_ids = {body_id}
        if including_children:
            body_ids.update(self.get_children_ids(body_id))

        contact_body_ids = set()
        for contact_id in range(self._mj_data.ncon):
            contact = self._mj_data.contact[contact_id]
            if contact.exclude != 0 and contact.exclude != 1:
                continue
            geom_1_id = contact.geom1
            geom_2_id = contact.geom2
            body_1_id = self._mj_model.geom_bodyid[geom_1_id]
            body_2_id = self._mj_model.geom_bodyid[geom_2_id]
            if body_1_id in body_ids and body_2_id not in body_ids:
                contact_body_ids.add(body_2_id)
            elif body_2_id in body_ids and body_1_id not in body_ids:
                contact_body_ids.add(body_1_id)

        contact_body_names = {self._mj_model.body(contact_body_id).name for contact_body_id in contact_body_ids}
        including_children_str = f"with its {len(body_ids) - 1} children" if including_children else "without children"
        return MultiverseFunctionResult(
            type=MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION,
            info=f"There are {len(contact_body_names)} contact bodies with body {body_name} {including_children_str}",
            result=contact_body_names
        )

    @MultiverseSimulator.multiverse_callback
    def get_body_root_name(self,
                           body_name: str) -> MultiverseFunctionResult:
        body_id = mujoco.mj_name2id(m=self._mj_model, type=mujoco.mjtObj.mjOBJ_BODY, name=body_name)
        if body_id == -1:
            return MultiverseFunctionResult(
                type=MultiverseFunctionResult.ResultType.FAILURE_WITHOUT_EXECUTION,
                info=f"Body {body_name} not found"
            )
        body_root_id = body_id
        while self._mj_model.body(body_root_id).parentid[0] != 0:
            body_root_id = self._mj_model.body(body_root_id).parentid[0]
        body_root_name = self._mj_model.body(body_root_id).name
        return MultiverseFunctionResult(
            type=MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION,
            info=f"Getting root body name of {body_name}",
            result=body_root_name)

    @MultiverseSimulator.multiverse_callback
    def get_contact_points(self,
                           body_1_name: str,
                           body_2_name: Optional[str] = None,
                           including_children: bool = True,
                           contact_style: str = "pybullet") -> MultiverseFunctionResult:
        if contact_style != "pybullet":
            return MultiverseFunctionResult(
                type=MultiverseFunctionResult.ResultType.FAILURE_WITHOUT_EXECUTION,
                info=f"Contact style {contact_style} is not supported"
            )

        get_body_1_root_name = self.get_body_root_name(body_1_name)
        if get_body_1_root_name.type != MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION:
            return get_body_1_root_name
        body_root_map = {body_1_name: get_body_1_root_name.result}

        if including_children:
            body_1_id = mujoco.mj_name2id(m=self._mj_model, type=mujoco.mjtObj.mjOBJ_BODY, name=body_1_name)
            body_root_map.update({self._mj_model.body(child_body_id).name: get_body_1_root_name.result
                                  for child_body_id in self.get_children_ids(body_1_id)})
        body_1_names = list(body_root_map.keys())

        if body_2_name is not None:
            get_body_2_root_name = self.get_body_root_name(body_2_name)
            if get_body_2_root_name.type != MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION:
                return get_body_2_root_name
            body_root_map[body_2_name] = get_body_1_root_name.result
        else:
            for body_2_id in range(self._mj_model.nbody):
                body_2_name = self._mj_model.body(body_2_id).name
                body_2_root_id = body_2_id
                while self._mj_model.body(body_2_root_id).parentid[0] != 0:
                    body_2_root_id = self._mj_model.body(body_2_root_id).parentid[0]
                body_2_root_name = self._mj_model.body(body_2_root_id).name
                body_root_map[body_2_name] = body_2_root_name

        contact_points = []
        contact_bodies = set()
        for contact_id in range(self._mj_data.ncon):
            contact = self._mj_data.contact[contact_id]
            if contact.exclude != 0 and contact.exclude != 1:
                continue
            geom_A_id = contact.geom[1]
            geom_B_id = contact.geom[0]
            body_A_id = self._mj_model.geom(geom_A_id).bodyid
            body_B_id = self._mj_model.geom(geom_B_id).bodyid
            body_A_name = self._mj_model.body(body_A_id).name
            body_B_name = self._mj_model.body(body_B_id).name
            if body_A_name not in body_root_map and body_B_name not in body_root_map:
                continue
            contact_bodies.add(body_A_name)
            contact_bodies.add(body_B_name)

            contact_position = contact.pos
            contact_normal = contact.frame[0:3]
            contact_distance = contact.dist
            contact_effort = numpy.zeros(6)
            mujoco.mj_contactForce(self._mj_model, self._mj_data, contact_id, contact_effort)

            contact_point = {"bodyUniqueNameA": body_root_map[body_A_name],
                             "bodyUniqueNameB": body_root_map[body_B_name], "linkNameA": body_A_name,
                             "linkNameB": body_B_name,
                             "positionOnA": contact_position + contact_normal * contact_distance / 2,
                             "positionOnB": contact_position - contact_normal * contact_distance / 2,
                             "contactNormalOnB": contact_normal, "contactDistance": contact_distance,
                             "normalForce": contact_effort[0], "lateralFriction1": contact_effort[1],
                             "lateralFrictionDir1": contact.frame[3:6], "lateralFriction2": contact_effort[2],
                             "lateralFrictionDir2": contact.frame[6:9]}
            contact_points.append(contact_point)

        contact_points_str = f" with bodies {[body_2_name for body_2_name in contact_bodies if body_2_name not in body_1_names]}" if len(contact_bodies) > 1 else ""
        return MultiverseFunctionResult(
            type=MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION,
            info=f"There are {len(contact_points)} contact points of bodies {body_1_names}{contact_points_str}.",
            result=contact_points
        )

    @MultiverseSimulator.multiverse_callback
    def get_contact_bodies_and_points(self,
                                      body_1_name: str,
                                      body_2_name: Optional[str] = None,
                                      including_children: bool = True):
        pass

    @MultiverseSimulator.multiverse_callback
    def save(self,
             file_path: Optional[str] = None,
             key_name: Optional[str] = None) -> MultiverseFunctionResult:
        if key_name is None:
            key_id = 0
        else:
            key_id = mujoco.mj_name2id(m=self._mj_model, type=mujoco.mjtObj.mjOBJ_KEY, name=key_name)
            if key_id == -1:
                self._mj_spec.add_key(name=key_name,
                                      qpos=self._mj_data.qpos,
                                      qvel=self._mj_data.qvel,
                                      act=self._mj_data.act,
                                      ctrl=self._mj_data.ctrl,
                                      time=self.current_simulation_time)
                self._mj_model, self._mj_data = self._mj_spec.recompile(self._mj_model, self._mj_data)
                self._renderer._sim().load(self._mj_model, self._mj_data, "")
                key_id = mujoco.mj_name2id(m=self._mj_model, type=mujoco.mjtObj.mjOBJ_KEY, name=key_name)
        mujoco.mj_setKeyframe(self._mj_model, self._mj_data, key_id)
        if file_path is not None:
            if os.path.exists(file_path):
                os.remove(file_path)
            xml_string = self._mj_spec.to_xml()
            with open(file_path, "w") as f:
                f.write(xml_string)
            return MultiverseFunctionResult(
                type=MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION,
                info=f"Saved simulation with key {key_name} to {file_path}",
                result=key_id
            )
        return MultiverseFunctionResult(
            type=MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION,
            info=f"Saved simulation with key {key_name}",
            result=key_id
        )

    @MultiverseSimulator.multiverse_callback
    def load(self,
             file_path: Optional[str] = None,
             key_id: int = 0) -> MultiverseFunctionResult:
        if file_path is not None:
            if not os.path.exists(file_path):
                return MultiverseFunctionResult(
                    type=MultiverseFunctionResult.ResultType.FAILURE_WITHOUT_EXECUTION,
                    info=f"File {file_path} not found"
                )
            self._mj_model, self._mj_data = self._mj_spec.recompile(self._mj_model, self._mj_data)
            self._renderer._sim().load(self._mj_model, self._mj_data, "")
            if key_id >= self._mj_model.nkey:
                return MultiverseFunctionResult(
                    type=MultiverseFunctionResult.ResultType.FAILURE_WITHOUT_EXECUTION,
                    info=f"Key {key_id} not found"
                )
            mujoco.mj_resetDataKeyframe(self._mj_model, self._mj_data, key_id)
            return MultiverseFunctionResult(
                type=MultiverseFunctionResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_DATA,
                info=f"Loaded simulation with key_id {key_id} from {file_path}",
                result=key_id
            )
        if key_id >= self._mj_model.nkey:
            return MultiverseFunctionResult(
                type=MultiverseFunctionResult.ResultType.FAILURE_WITHOUT_EXECUTION,
                info=f"Key {key_id} not found"
            )
        mujoco.mj_resetDataKeyframe(self._mj_model, self._mj_data, key_id)
        return MultiverseFunctionResult(
            type=MultiverseFunctionResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_DATA,
            info=f"Loaded simulation with key_id {key_id}",
            result=key_id
        )
