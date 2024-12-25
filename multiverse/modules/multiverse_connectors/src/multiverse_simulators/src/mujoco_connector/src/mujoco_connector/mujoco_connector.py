#!/usr/bin/env python3

"""Multiverse Mujoco Connector class"""

import os

os.environ['XLA_FLAGS'] = '--xla_gpu_triton_gemm_any=true'
import xml.etree.ElementTree as ET
from typing import Optional, List, Callable, Set

import jax
import mujoco
import mujoco.viewer
import numpy
from mujoco import mjx

from multiverse_simulator import (MultiverseSimulator, MultiverseRenderer, MultiverseViewer,
                                  MultiverseFunction, MultiverseFunctionResult)
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
        mujoco.mj_loadPluginLibrary(get_multiverse_connector_plugin())
        assert os.path.exists(self.file_path)
        self._mj_spec = mujoco.MjSpec.from_file(filename=self.file_path)

        self._mj_spec.option.integrator = getattr(mujoco.mjtIntegrator,
                                                  f"mjINT_{kwargs.get('integrator', 'EULER')}")
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
            "cmd_joint_force": "ctrl"
        }
        attr_size = {
            "xpos": 3,
            "xquat": 4,
            "qpos": 1,
            "qvel": 1,
            "qfrc_applied": 1,
            "ctrl": 1
        }
        i = 0
        ids_dict.clear()
        for name, attrs in objects.items():
            for attr_name in attrs.keys():
                mj_attr_name = attr_map[attr_name]
                if mj_attr_name not in ids_dict:
                    ids_dict[mj_attr_name] = [[], []]

                if attr_name in {"position", "quaternion"}:
                    mj_attr_id = self._mj_model.body(name).id
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
                if attr not in {"xpos", "xquat"}:
                    batch_data[attr] = numpy.array(getattr(self._batch, attr))
                    batch_data[attr][:, indices[0]] = write_data[:, indices[1]]
                else:
                    body_id = indices[0][0]
                    jntid = self._mj_model.body(body_id).jntadr[0]
                    jnt = self._mj_model.jnt(jntid)
                    assert jnt.type == mujoco.mjtJoint.mjJNT_FREE
                    qpos_adr = jnt.qposadr
                    batch_data["qpos"] = numpy.array(self._batch.qpos)
                    if attr == "xpos":
                        batch_data["qpos"][:, qpos_adr:qpos_adr + 3] = write_data[:, indices[1]]
                    elif attr == "xquat":
                        batch_data["qpos"][:, qpos_adr + 3:qpos_adr + 7] = write_data[:, indices[1]]
            self._batch = self._batch.replace(**batch_data)
        else:
            for attr, indices in self._write_ids.items():
                if attr not in {"xpos", "xquat"}:
                    getattr(self._mj_data, attr)[indices[0]] = write_data[0][indices[1]]
                else:
                    body_id = indices[0][0]
                    jntid = self._mj_model.body(body_id).jntadr[0]
                    jnt = self._mj_model.jnt(jntid)
                    assert jnt.type == mujoco.mjtJoint.mjJNT_FREE
                    qpos_adr = jnt.qposadr[0]
                    if attr == "xpos":
                        self._mj_data.qpos[qpos_adr:qpos_adr + 3] = write_data[0][indices[1]]
                    elif attr == "xquat":
                        self._mj_data.qpos[qpos_adr + 3:qpos_adr + 7] = write_data[0][indices[1]]

    def read_data_from_simulator(self, read_data: numpy.ndarray):
        if not self.use_mjx and read_data.shape[0] > 1:
            raise NotImplementedError("Multiple environments for non MJX is not supported yet")
        if self.use_mjx:
            for attr, indices in self._read_ids.items():
                attr_values = getattr(self._batch, attr)
                read_data[:, indices[1]] = attr_values[:, indices[0]].reshape(attr_values.shape[0], -1)
        else:
            for attr, indices in self._read_ids.items():
                attr_values = getattr(self._mj_data, attr)
                read_data[0][indices[1]] = attr_values[indices[0]]

    @property
    def file_path(self) -> str:
        return self._file_path

    @property
    def current_simulation_time(self) -> float:
        return self._mj_data.time if not self.use_mjx else self._batch.time[0]

    @property
    def renderer(self):
        return self._renderer

    def _make_functions(self) -> List[MultiverseFunction | Callable]:
        def get_all_body_names() -> MultiverseFunctionResult:
            result = [self._mj_model.body(body_id).name for body_id in range(self._mj_model.nbody)]
            return MultiverseFunctionResult(
                type=MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION,
                info="Getting all body names",
                result=result
            )

        def get_all_joint_names() -> MultiverseFunctionResult:
            result = [self._mj_model.joint(joint_id).name for joint_id in range(self._mj_model.njnt)]
            return MultiverseFunctionResult(
                type=MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION,
                info="Getting all body names",
                result=result
            )

        def attach(body_1_name: str,
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
            body_2_spec = self._mj_spec.find_body(body_2_name)
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

            return MultiverseFunctionResult(
                type=MultiverseFunctionResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_MODEL,
                info=f"Attached body 1 {body_1_name} to body 2 {body_2_name} "
                     f"at relative position {relative_position}, relative quaternion {relative_quaternion}"
            )

        def detach(body_name: str, add_freejoint: bool = True) -> MultiverseFunctionResult:
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

            return MultiverseFunctionResult(
                type=MultiverseFunctionResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_MODEL,
                info=f"Detached body {body_name} from body {parent_body_name}"
            )

        def get_children_ids(body_id: int) -> Set[int]:
            children_ids = set()
            for child_body_id in range(body_id + 1, self._mj_model.nbody):
                if self._mj_model.body(child_body_id).parentid[0] == body_id:
                    children_ids.add(child_body_id)
                else:
                    break
            return children_ids

        def get_contact_bodies(body_name: str, including_children: bool = True):
            body_id = mujoco.mj_name2id(m=self._mj_model, type=mujoco.mjtObj.mjOBJ_BODY, name=body_name)
            if body_id == -1:
                return MultiverseFunctionResult(
                    type=MultiverseFunctionResult.ResultType.FAILURE_WITHOUT_EXECUTION,
                    info=f"Body {body_name} not found"
                )

            body_ids = {body_id}
            if including_children:
                body_ids.update(get_children_ids(body_id))

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

        def get_contact_points(body_1_name: str, body_2_name: Optional[str] = None, including_children: bool = True):
            body_1_id = mujoco.mj_name2id(m=self._mj_model, type=mujoco.mjtObj.mjOBJ_BODY, name=body_1_name)
            if body_1_id == -1:
                return MultiverseFunctionResult(
                    type=MultiverseFunctionResult.ResultType.FAILURE_WITHOUT_EXECUTION,
                    info=f"Body {body_1_id} not found"
                )

            if body_2_name is not None:
                body_2_id = mujoco.mj_name2id(m=self._mj_model, type=mujoco.mjtObj.mjOBJ_BODY, name=body_2_name)
                if body_2_id == -1:
                    return MultiverseFunctionResult(
                        type=MultiverseFunctionResult.ResultType.FAILURE_WITHOUT_EXECUTION,
                        info=f"Body {body_2_id} not found"
                    )
            else:
                body_2_id = None

            contact_body_points = []
            for contact_id in range(self._mj_data.ncon):
                contact = self._mj_data.contact[contact_id]
                if contact.exclude != 0 and contact.exclude != 1:
                    continue
                geom_1_id = contact.geom1
                geom_2_id = contact.geom2
                if body_1_id == self._mj_model.geom_bodyid[geom_1_id]:
                    if body_2_id is None or body_2_id == self._mj_model.geom_bodyid[geom_2_id]:
                        contact_body_points.append(contact.frame)
                elif body_1_id == self._mj_model.geom_bodyid[geom_2_id]:
                    if body_2_id is None or body_2_id == self._mj_model.geom_bodyid[geom_1_id]:
                        contact_body_points.append(-contact.frame)

            contact_body_str = f" with body {body_2_name}" if body_2_name is not None else ""
            return MultiverseFunctionResult(
                type=MultiverseFunctionResult.ResultType.SUCCESS_WITHOUT_EXECUTION,
                info=f"There are {len(contact_body_points)} contact points of body {body_1_name}{contact_body_str}",
                result=contact_body_points
            )

        def get_contact_bodies_and_points(body_1_name: str, body_2_name: Optional[str] = None,
                                          including_children: bool = True):
            pass

        return [get_all_body_names, get_all_joint_names, attach, detach, get_contact_bodies, get_contact_points]

    def _fix_prefix_and_recompile(self, body_spec: mujoco.MjsBody, dummy_prefix: str, body_name: str):
        body_spec.name = body_name
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
