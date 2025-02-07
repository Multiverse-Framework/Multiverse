# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import sys
import os

for path in os.environ["PYTHONPATH"].split(os.pathsep):
    multiverse_client_path = os.path.join(path, "multiverse_client_py")
    if os.path.exists(multiverse_client_path):
        if multiverse_client_path not in sys.path:
            sys.path.append(path)
        break
else:
    for path in sys.path:
        multiverse_client_path = os.path.join(path, "multiverse_client_py")
        if os.path.exists(multiverse_client_path):
            break
    else:
        raise ImportError("multiverse_client_py not found in PYTHONPATH")

from multiverse_client_py import MultiverseClient, MultiverseMetaData

class MultiverseConnector(MultiverseClient):
    def __init__(self, port: str, multiverse_meta_data: MultiverseMetaData) -> None:
        super().__init__(port, multiverse_meta_data)

    def loginfo(self, message: str) -> None:
        print(f"INFO: {message}")

    def logwarn(self, message: str) -> None:
        print(f"WARN: {message}")

    def _run(self) -> None:
        self.loginfo("Start running the client.")
        self._connect_and_start()

    def send_and_receive_meta_data(self) -> None:
        # self.loginfo("Sending request meta data: " + str(self.request_meta_data))
        self._communicate(True)
        # self.loginfo("Received response meta data: " + str(self.response_meta_data))

    def send_and_receive_data(self) -> None:
        # self.loginfo("Sending data: " + str(self.send_data))
        self._communicate(False)
        # self.loginfo("Received data: " + str(self.receive_data))

from dataclasses import dataclass

@dataclass
class MultiverseObject:
    prim: ...
    root_prim: ...

import numpy
import json
import torch
import omni.timeline
import omni.ui as ui
from isaaclab.sim import SimulationContext
from isaacsim.gui.components.element_wrappers import CollapsableFrame, TextBlock, StringField, IntField, CheckBox
from isaacsim.gui.components.ui_utils import get_style
from isaacsim.core.api.world.world import World
from isaacsim.core.api.scenes.scene import Scene
from isaacsim.core.api.scenes.scene_registry import SceneRegistry
from isaacsim.core.prims import XFormPrim, RigidPrim, Articulation
from isaacsim.core.utils.types import ArticulationActions
from isaacsim.core.utils import stage
from omni.isaac.dynamic_control import _dynamic_control
from pxr import UsdGeom, UsdPhysics

dc = _dynamic_control.acquire_dynamic_control_interface()

class UIBuilder(MultiverseConnector):
    def __init__(self):
        # Frames are sub-windows that can contain multiple UI elements
        self.frames = []

        # UI elements created using a UIElementWrapper from isaacsim.gui.components.element_wrappers
        self.wrapped_ui_elements = []

        self._world = None
        self._scene = None

        self._clean_up()
        self._ignore_names = ["defaultGroundPlane", "Environment", "OmniKit_Viewport_LightRig", "Lights", "Robot", "Objects"]

        # Get access to the timeline to control stop/pause/play programmatically
        self._timeline = omni.timeline.get_timeline_interface()

    ###################################################################################
    #           The Functions Below Are Called Automatically By extension.py
    ###################################################################################

    def on_menu_callback(self):
        """Callback for when the UI is opened from the toolbar.
        This is called directly after build_ui().
        """
        # Handles the case where the user loads their Articulation and
        # presses play before opening this extension
        if self._timeline.is_playing():
            pass
        elif self._timeline.is_stopped():
            pass

    def on_timeline_event(self, event):
        """Callback for Timeline events (Play, Pause, Stop)

        Args:
            event (omni.timeline.TimelineEventType): Event Type
        """
        pass

    def on_physics_step(self, step):
        """Callback for Physics Step.
        Physics steps only occur when the timeline is playing

        Args:
            step (float): Size of physics step
        """
        if len(self._send_objects) + len(self._receive_objects) > 0:
            self.send_and_receive_data()

    def on_stage_event(self, event):
        """Callback for Stage Events

        Args:
            event (omni.usd.StageEventType): Event Type
        """
        if event.type == int(omni.usd.StageEventType.ASSETS_LOADED):
            if isinstance(self.world, SimulationContext) and self._timeline.is_playing():
                self._clean_up()
                self.build_ui()
                self._init()
        elif event.type == int(omni.usd.StageEventType.SIMULATION_START_PLAY):
            if isinstance(self.world, omni.isaac.core.world.world.World):
                # self.build_ui()
                self._init()
        elif event.type == int(omni.usd.StageEventType.ANIMATION_START_PLAY):
            if isinstance(self.world, SimulationContext):
                # self._init()
                pass # TODO: Make pause and unpause in IsaacLab work
        elif event.type == int(omni.usd.StageEventType.SIMULATION_STOP_PLAY):
            # Ignore pause events
            if isinstance(self.world, SimulationContext) or self._timeline.is_stopped():
                self._clean_up(clean_ui=False)

    def cleanup(self):
        """
        Called when the stage is closed or the extension is hot reloaded.
        Perform any necessary cleanup such as removing active callback functions
        Buttons imported from isaacsim.gui.components.element_wrappers implement a cleanup function that should be called
        """
        self._clean_up()
        for ui_elem in self.wrapped_ui_elements:
            ui_elem.cleanup()

    def build_ui(self):
        """
        Build a custom UI tool to run your extension.
        This function will be called any time the UI window is closed and reopened.
        """
        current_stage = stage.get_current_stage()
        customLayerData = current_stage.GetRootLayer().customLayerData
        host = "tcp://127.0.0.1"
        server_port = 7000
        client_port = 8000
        world_name = "world"
        simulation_name = "isaac_sim"
        send = {}
        receive = {}

        multiverse_params = {}
        if "multiverse_connector" in customLayerData:
            multiverse_params = customLayerData["multiverse_connector"]
        else:
            tmp_path = "/tmp/multiverse_isaacsim_connector.yaml"
            if os.path.exists(tmp_path):
                import yaml
                with open(tmp_path, "r") as file:
                    multiverse_params = yaml.safe_load(file)

        host = multiverse_params.get("host", host)
        server_port = multiverse_params.get("server_port", server_port)
        client_port = multiverse_params.get("client_port", client_port)
        world_name = multiverse_params.get("world_name", world_name)
        simulation_name = multiverse_params.get("simulation_name", simulation_name)
        send = multiverse_params.get("send", send)
        receive = multiverse_params.get("receive", receive)
        for key, values in send.items():
            if isinstance(values, str):
                send[key] = json.loads(values)
        for key, values in receive.items():
            if isinstance(values, str):
                receive[key] = json.loads(values)

        selection_panel_frame = CollapsableFrame("Multiverse Connector", collapsed=False)

        with selection_panel_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._info_text = TextBlock(
                    "Info",
                    "Multiverse Client implementation in Isaac Sim.",
                    include_copy_button=False,
                    num_lines=1,
                )

                self._host_field = StringField(
                    label="Server Host",
                    tooltip="Enter the server host address.",
                    default_value=host
                )
                self._server_port_field = IntField(
                    label="Server Port",
                    tooltip="Enter the server port number.",
                    default_value=server_port,
                    lower_limit=1000,
                    upper_limit=9999
                )
                self._client_port_field = IntField(
                    label="Client Port",
                    tooltip="Enter the client port number.",
                    default_value=client_port,
                    lower_limit=1000,
                    upper_limit=9999
                )
                self._world_name_field = StringField(
                    label="World Name",
                    tooltip="Enter the name of the world.",
                    default_value=world_name
                )
                self._simulation_name_field = StringField(
                    label="Simulation Name",
                    tooltip="Enter the name of the simulation.",
                    default_value=simulation_name
                )

                prims = {
                    "body": [],
                    "revolute_joint": [],
                    "prismatic_joint": []
                }
                for prim in current_stage.TraverseAll():
                    if prim.GetName() in self._ignore_names:
                        continue
                    if prim.IsA(UsdGeom.Xform):
                        prims["body"].append(prim)
                    elif prim.IsA(UsdPhysics.RevoluteJoint):
                        prims["revolute_joint"].append(prim)
                    elif prim.IsA(UsdPhysics.PrismaticJoint):
                        prims["prismatic_joint"].append(prim)
                attributes = {
                    "body": ["position", "quaternion", "relative_velocity"],
                    "revolute_joint": ["joint_rvalue", "joint_angular_velocity", "joint_torque", "cmd_joint_rvalue", "cmd_joint_angular_velocity", "cmd_joint_torque"],
                    "prismatic_joint": ["joint_tvalue", "joint_linear_velocity", "joint_force", "cmd_joint_tvalue", "cmd_joint_linear_velocity", "cmd_joint_force"]
                }
                titles = {
                    "body": "Bodies",
                    "revolute_joint": "Revolute Joints",
                    "prismatic_joint": "Prismatic Joints"
                }

                self._send_objects_frame = CollapsableFrame(
                    title="Send Objects",
                    collapsed=True,
                    enabled=True)
                with self._send_objects_frame:
                    with ui.VStack():
                        for prim_type, prim_list in prims.items():
                            prim_type_frame = CollapsableFrame(
                                title=f"{titles[prim_type]}",
                                collapsed=True,
                                enabled=True)
                            with prim_type_frame:
                                with ui.VStack():
                                    for prim in prim_list:
                                        prim_path = prim.GetPath()
                                        prim_name = prim_path.name
                                        prim_frame = CollapsableFrame(
                                            title=prim_path.pathString,
                                            collapsed=True,
                                            enabled=True)
                                        with prim_frame:
                                            with ui.VStack():
                                                name_field = StringField(
                                                    label="name",
                                                    default_value=prim_name,
                                                    tooltip=f"Send name of {prim_path}."
                                                )
                                                self._send_prims_dict[prim] = name_field
                                                self._send_prims_check_boxes[prim] = []
                                                for attr in attributes[prim_type]:
                                                    default_value = False
                                                    if prim_name in send and attr in send[prim_name]:
                                                        default_value = True
                                                    if prim_type == "body":
                                                        if "body" in send and attr in send["body"]:
                                                            default_value = True
                                                    elif prim_type in ["revolute_joint", "prismatic_joint"]:
                                                        if "joint" in send and attr in send["joint"]:
                                                            default_value = True
                                                    check_box = CheckBox(
                                                        label=attr,
                                                        default_value=default_value,
                                                        tooltip=f"Send {attr} of {prim_path}."
                                                    )
                                                    self._send_prims_check_boxes[prim].append(check_box)

                self._receive_objects_frame = CollapsableFrame(
                    title="Receive Objects",
                    collapsed=True,
                    enabled=True)
                with self._receive_objects_frame:
                    with ui.VStack():
                        for prim_type, prim_list in prims.items():
                            prim_type_frame = CollapsableFrame(
                                title=f"{titles[prim_type]}",
                                collapsed=True,
                                enabled=True)
                            with prim_type_frame:
                                with ui.VStack():
                                    for prim in prim_list:
                                        prim_path = prim.GetPath()
                                        prim_name = prim_path.name
                                        prim_frame = CollapsableFrame(
                                            title=prim_path.pathString,
                                            collapsed=True,
                                            enabled=True)
                                        with prim_frame:
                                            with ui.VStack():
                                                name_field = StringField(
                                                    label="name",
                                                    default_value=prim_name,
                                                    tooltip=f"Receive name of {prim_path}."
                                                )
                                                self._receive_prims_dict[prim] = name_field
                                                self._receive_prims_check_boxes[prim] = []
                                                for attr in attributes[prim_type]:
                                                    default_value = False
                                                    if prim_name in receive and attr in receive[prim_name]:
                                                        default_value = True
                                                    if prim_type == "body":
                                                        if "body" in receive and attr in receive["body"]:
                                                            default_value = True
                                                    elif prim_type in ["revolute_joint", "prismatic_joint"]:
                                                        if "joint" in receive and attr in receive["joint"]:
                                                            default_value = True
                                                    check_box = CheckBox(
                                                        label=attr,
                                                        default_value=default_value,
                                                        tooltip=f"Receive {attr} of {prim_path}."
                                                    )
                                                    self._receive_prims_check_boxes[prim].append(check_box)

    ######################################################################################
    # Functions Below This Point Support The Provided Example And Can Be Replaced/Deleted
    ######################################################################################

    def _init(self):
        multiverse_meta_data = MultiverseMetaData(
            world_name=self._world_name_field.get_value(),
            simulation_name=self._simulation_name_field.get_value(),
            length_unit="m",
            angle_unit="rad",
            mass_unit="kg",
            time_unit="s",
            handedness="rhs"
        )
        MultiverseConnector._host = self._host_field.get_value()
        super().__init__(port=str(self._client_port_field.get_value()),
                         multiverse_meta_data=multiverse_meta_data)
        
        def init_objects_callback() -> None:
            self._send_objects = {}
            self._receive_objects = {}
            for prim, check_boxes in self._send_prims_check_boxes.items():
                for check_box in check_boxes:
                    if check_box.get_value():
                        self.__add_send_object(prim, check_box.label.text)
                    else:
                        self.__remove_send_object(prim, check_box.label.text)
            for prim, check_boxes in self._receive_prims_check_boxes.items():
                for check_box in check_boxes:
                    if check_box.get_value():
                        self.__add_receive_object(prim, check_box.label.text)
                    else:
                        self.__remove_receive_object(prim, check_box.label.text)

            if len(self.scene_registry.rigid_contact_views) > 0: # Need to stop if rigid_contact_views are present
                self.world.stop()
            current_stage = stage.get_current_stage()
            articulation_prim_paths = []
            self._body_dict = {}
            self._body_collision_geom_path_dict = {}
            self._joint_dict = {}
            self._constrained_bodies = set()

            for prim in current_stage.TraverseAll():
                if prim.IsA(UsdGeom.Xform):
                    prim_path = prim.GetPath()
                    body_name = prim_path.name
                    if body_name in self._ignore_names:
                        continue
                    parent_prim = prim.GetParent()
                    if parent_prim.IsA(UsdGeom.Xform) and parent_prim.GetName() == body_name:
                        root_prim = parent_prim
                    else:
                        root_prim = prim
                    self._body_dict[prim_path] = MultiverseObject(prim, root_prim)

                if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                    if prim.IsA(UsdGeom.Xform):
                        xform_prim_path = prim.GetPath()
                    elif prim.IsA(UsdPhysics.Joint):
                        joint = UsdPhysics.Joint(prim)
                        xform_prim_path = joint.GetBody1Rel().GetTargets()[0]
                    articulation_prim_paths.append(xform_prim_path)

            for body_path, body_prim in self._body_dict.items():
                self._body_collision_geom_path_dict[body_path] = []
                for body_child_prim in body_prim.prim.GetChildren():
                    if body_child_prim.HasAPI(UsdPhysics.CollisionAPI) or body_child_prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                        self._body_collision_geom_path_dict[body_path].append(body_child_prim.GetPath())
                if not body_prim.prim.HasAPI(UsdPhysics.RigidBodyAPI) and not body_prim.root_prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    self._constrained_bodies.add(body_path)
                else:
                    rigid_body_api = UsdPhysics.RigidBodyAPI(prim)
                    if not rigid_body_api.GetRigidBodyEnabledAttr().Get():
                        self._constrained_bodies.add(body_path)

            for joint_prim in [prim for prim in current_stage.TraverseAll() if prim.IsA(UsdPhysics.Joint)]:
                joint = UsdPhysics.Joint(joint_prim)
                body0_targets = joint.GetBody0Rel().GetTargets()
                if len(body0_targets) != 0:
                    body0_path = body0_targets[0]
                    self._constrained_bodies.add(body0_path)
                body1_targets = joint.GetBody1Rel().GetTargets()
                if len(body1_targets) != 0:
                    body1_path = body1_targets[0]
                    self._constrained_bodies.add(body1_path)
                joint_name = joint_prim.GetName()
                if joint_name in self._ignore_names:
                    continue
                if joint_prim.IsA(UsdPhysics.RevoluteJoint) or joint_prim.IsA(UsdPhysics.PrismaticJoint):
                    joint_path = joint_prim.GetPath()
                    self._joint_dict[joint_path] = joint_prim

            body_paths = []

            for prim in [*self._send_objects, *self._receive_objects]:
                prim_path = prim.GetPath()
                prim_name = prim_path.name
                for articulation_prim_path in articulation_prim_paths:
                    articulation = dc.get_articulation(articulation_prim_path.pathString)
                    if prim_path in self._joint_dict:
                        dof_ptr = dc.find_articulation_dof(articulation, prim_name)
                        if dof_ptr == 0:
                            continue
                    elif prim_path in self._body_dict:
                        prim = self._body_dict[prim_path].prim
                        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                            break
                        rigid_body_api = UsdPhysics.RigidBodyAPI(prim)
                        if not rigid_body_api.GetRigidBodyEnabledAttr().Get():
                            break

                    articulation_view_name = f"{articulation_prim_path}_view"
                    if self.scene_registry.name_exists(articulation_view_name):
                        articulation_view = self.scene_registry.articulated_views[articulation_view_name]
                    else:
                        articulation_view = Articulation(prim_paths_expr=articulation_prim_path.pathString, name=articulation_view_name)
                        articulation_view.initialize(self.world.physics_sim_view)
                        self.scene_registry.add_articulated_view(articulation_view.name, articulation_view)
                    if prim_path in self._joint_dict and prim_name in articulation_view.dof_names:
                        joint_idx = articulation_view.get_dof_index(prim_name)
                        self._object_articulation_view_idx_dict[prim_path] = (articulation_view.name, joint_idx)
                    elif prim_path in self._body_dict and prim_path == articulation_prim_path:
                        body_idx = articulation_view.get_link_index(prim_name)
                        self._object_articulation_view_idx_dict[prim_path] = (articulation_view.name, body_idx)
                    if prim_path in self._object_articulation_view_idx_dict:
                        break
                if prim_path not in self._object_articulation_view_idx_dict and prim_path in self._body_dict:
                    body_paths.append(self._body_dict[prim_path].prim.GetPath())

            if len(body_paths) > 0:
                xform_prim_paths = []
                rigid_prim_paths = []
                for body_path in body_paths:
                    prim = current_stage.GetPrimAtPath(body_path)
                    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                        rigid_body_api = UsdPhysics.RigidBodyAPI(prim)
                        rigid_body_enabled_attr = rigid_body_api.GetRigidBodyEnabledAttr().Get()
                        if rigid_body_enabled_attr:
                            rigid_prim_paths.append(body_path)
                            continue
                    xform_prim_paths.append(body_path)

                if len(xform_prim_paths) > 0:
                    xform_prim_view = XFormPrim(prim_paths_expr=[body_path.pathString for body_path in xform_prim_paths])
                    xform_prim_view.initialize(self.world.physics_sim_view)
                    self.scene_registry.add_xform_view(xform_prim_view.name, xform_prim_view)
                    for body_idx, body_path in enumerate(xform_prim_paths):
                        self._object_xform_prim_view_idx_dict[body_path] = (xform_prim_view.name, body_idx)

                if len(rigid_prim_paths) > 0:
                    rigid_prim_view = RigidPrim(prim_paths_expr=[body_path.pathString for body_path in rigid_prim_paths])
                    rigid_prim_view.initialize(self.world.physics_sim_view)
                    self.scene_registry.add_rigid_prim_view(rigid_prim_view.name, rigid_prim_view)
                    for body_idx, body_path in enumerate(rigid_prim_paths):
                        self._object_rigid_prim_view_idx_dict[body_path] = (rigid_prim_view.name, body_idx)
        self.init_objects_callback = init_objects_callback
        
        def bind_request_meta_data() -> None:
            request_meta_data = self.request_meta_data
            for prim, attrs in self._send_objects.items():
                object_name = self._send_prims_dict[prim].get_value()
                self._send_names_dict[object_name] = prim
                request_meta_data["send"][object_name] = attrs
            for prim, attrs in self._receive_objects.items():
                object_name = self._receive_prims_dict[prim].get_value()
                self._receive_names_dict[object_name] = prim
                request_meta_data["receive"][object_name] = attrs
            self.request_meta_data = request_meta_data
        self.bind_request_meta_data_callback = bind_request_meta_data

        def bind_response_meta_data() -> None:
            response_meta_data = self.response_meta_data

            free_body_names = []
            bodies_positions = {}
            bodies_quaternions = {}
            bodies_velocities = {}
            joints_states = {}
            measured_joints_efforts = {}

            for xform_prim_view_name, xform_prim_view in self.scene_registry.xform_prim_views.items():
                bodies_positions[xform_prim_view_name], bodies_quaternions[xform_prim_view_name] = xform_prim_view.get_world_poses()

            for rigid_prim_view_name, rigid_prim_view in self.scene_registry.rigid_prim_views.items():
                for prim in rigid_prim_view.prims:
                    free_body_names.append(prim.GetName())
                bodies_positions[rigid_prim_view_name], bodies_quaternions[rigid_prim_view_name] = rigid_prim_view.get_world_poses()
                bodies_velocities[rigid_prim_view_name] = rigid_prim_view.get_velocities()

            for articulation_view_name, articulation_view in self.scene_registry.articulated_views.items():
                for body_name in articulation_view.body_names:
                    if body_name in [body_path.name for body_path in self._constrained_bodies] and body_name in free_body_names:
                        free_body_names.remove(body_name)
                bodies_positions[articulation_view_name], bodies_quaternions[articulation_view_name] = articulation_view.get_world_poses()
                bodies_velocities[articulation_view_name] = articulation_view.get_velocities()
                joints_states[articulation_view_name] = articulation_view.get_joints_state()
                measured_joints_efforts[articulation_view_name] = articulation_view.get_measured_joint_efforts()

            for send_receive in ["send", "receive"]:
                if send_receive not in response_meta_data:
                    continue
                for object_name, attributes in response_meta_data[send_receive].items():
                    prim = self._send_names_dict[object_name] if send_receive == "send" else self._receive_names_dict[object_name]
                    prim_path = prim.GetPath()
                    if prim_path in self._body_dict:
                        if prim_path in self._object_xform_prim_view_idx_dict:
                            view_name, object_idx = self._object_xform_prim_view_idx_dict[prim_path]
                        elif prim_path in self._object_rigid_prim_view_idx_dict:
                            view_name, object_idx = self._object_rigid_prim_view_idx_dict[prim_path]
                        elif prim_path in self._object_articulation_view_idx_dict:
                            view_name, object_idx = self._object_articulation_view_idx_dict[prim_path]
                        else:
                            raise Exception(f"Body {object_name} not found in any view")

                        for attribute in attributes:
                            data = response_meta_data[send_receive][object_name][attribute]
                            if any(x is None for x in data):
                                continue
                            if isinstance(self.world, SimulationContext):
                                data = torch.tensor(data).to("cuda").float()
                            if attribute == "position":
                                bodies_positions[view_name][object_idx] = data[:3]
                            elif attribute == "quaternion":
                                bodies_quaternions[view_name][object_idx] = data[:4]
                            elif attribute == "relative_velocity":
                                bodies_velocities[view_name][object_idx] = data[:6]

            for rigid_prim_view_name, rigid_prim_view in self.scene_registry.rigid_prim_views.items():
                free_body_idxes = [idx for idx, prim in enumerate(rigid_prim_view.prims) if prim.GetName() in free_body_names]
                if len(free_body_idxes) == 0:
                    continue
                body_positions = bodies_positions[rigid_prim_view_name][free_body_idxes]
                body_quaternions = bodies_quaternions[rigid_prim_view_name][free_body_idxes]
                rigid_prim_view.set_world_poses(body_positions, body_quaternions, indices=free_body_idxes)

            for articulation_view_name, articulation_view in self.scene_registry.articulated_views.items():
                articulation_view.set_world_poses(bodies_positions[articulation_view_name], bodies_quaternions[articulation_view_name])

            if not self.world.is_playing():
                self.world.play()
                self.world.pause()

            for rigid_prim_view_name, rigid_prim_view in self.scene_registry.rigid_prim_views.items():
                free_body_idxes = [idx for idx, prim in enumerate(rigid_prim_view.prims) if prim.GetName() in free_body_names]
                if len(free_body_idxes) == 0:
                    continue

                body_velocities = bodies_velocities[rigid_prim_view_name][free_body_idxes]
                rigid_prim_view.set_velocities(body_velocities, indices=free_body_idxes)

            for articulation_view_name, articulation_view in self.scene_registry.articulated_views.items():
                articulation_view.set_velocities(bodies_velocities[articulation_view_name])
        self.bind_response_meta_data_callback = bind_response_meta_data

        def bind_send_data() -> None:
            send_data = [self.sim_time]
            bodies_positions = {}
            bodies_quaternions = {}
            bodies_velocities = {}
            joints_states = {}
            measured_joints_efforts = {}

            for xform_prim_view_name, xform_prim_view in self.scene_registry.xform_prim_views.items():
                bodies_positions[xform_prim_view_name], bodies_quaternions[xform_prim_view_name] = xform_prim_view.get_world_poses()

            for rigid_prim_view_name, rigid_prim_view in self.scene_registry.rigid_prim_views.items():
                bodies_positions[rigid_prim_view_name], bodies_quaternions[rigid_prim_view_name] = rigid_prim_view.get_world_poses()
                bodies_velocities[rigid_prim_view_name] = rigid_prim_view.get_velocities()

            for articulation_view_name, articulation_view in self.scene_registry.articulated_views.items():
                bodies_positions[articulation_view_name], bodies_quaternions[articulation_view_name] = articulation_view.get_world_poses()
                bodies_velocities[articulation_view_name] = articulation_view.get_velocities()
                joints_states[articulation_view_name] = articulation_view.get_joints_state()
                measured_joints_efforts[articulation_view_name] = articulation_view.get_measured_joint_efforts()

            for object_name, attributes in self.response_meta_data.get("send", {}).items():
                prim = self._send_names_dict[object_name]
                prim_path = prim.GetPath()
                if prim_path in self._body_dict:
                    if prim_path in self._object_xform_prim_view_idx_dict:
                        view_name, object_idx = self._object_xform_prim_view_idx_dict[prim_path]
                    elif prim_path in self._object_rigid_prim_view_idx_dict:
                        view_name, object_idx = self._object_rigid_prim_view_idx_dict[prim_path]
                    elif prim_path in self._object_articulation_view_idx_dict:
                        view_name, object_idx = self._object_articulation_view_idx_dict[prim_path]
                    else:
                        raise Exception(f"Body {prim_path.pathString} not found in any view")

                    for attribute in attributes.keys():
                        if attribute == "position":
                            position = bodies_positions[view_name][object_idx]
                            if isinstance(position, torch.Tensor):
                                position = position.cpu().numpy()
                            send_data += [position[0], position[1], position[2]]
                        elif attribute == "quaternion":
                            quaternion = bodies_quaternions[view_name][object_idx]
                            if isinstance(quaternion, torch.Tensor):
                                quaternion = quaternion.cpu().numpy()
                            send_data += [quaternion[0], quaternion[1], quaternion[2], quaternion[3]]
                        elif attribute == "relative_velocity":
                            velocity = bodies_velocities[view_name][object_idx]
                            if isinstance(velocity, torch.Tensor):
                                velocity = velocity.cpu().numpy()
                            send_data += [velocity[0], velocity[1], velocity[2], velocity[3], velocity[4], velocity[5]]

                elif prim_path in self._joint_dict:
                    if prim_path in self._object_articulation_view_idx_dict:
                        articulation_view_name, joint_idx = self._object_articulation_view_idx_dict[prim_path]
                    else:
                        raise Exception(f"Joint {prim_path.pathString} not found in any view")
                    for attribute in attributes:
                        if attribute == "joint_rvalue":
                            value = joints_states[articulation_view_name].positions[0][joint_idx]
                        elif attribute == "joint_tvalue":
                            value = joints_states[articulation_view_name].positions[0][joint_idx]
                        elif attribute == "joint_linear_velocity":
                            value = joints_states[articulation_view_name].velocities[0][joint_idx]
                        elif attribute == "joint_angular_velocity":
                            value = joints_states[articulation_view_name].velocities[0][joint_idx]
                        elif attribute == "joint_force":
                            value = measured_joints_efforts[articulation_view_name][0][joint_idx]
                        elif attribute == "joint_torque":
                            value = measured_joints_efforts[articulation_view_name][0][joint_idx]
                        if isinstance(value, torch.Tensor):
                            value = float(value.cpu().numpy())
                        send_data += [value]
            self.send_data = send_data
        self.bind_send_data_callback = bind_send_data

        def bind_receive_data() -> None:
            receive_data = self.receive_data
            sim_time = receive_data[0]
            receive_data = receive_data[1:]

            bodies_positions = {}
            bodies_quaternions = {}

            bodies_velocities = {}
            cmd_joints_values = {}

            for xform_prim_view_name, xform_prim_view in self.scene_registry.xform_prim_views.items():
                bodies_positions[xform_prim_view_name], bodies_quaternions[xform_prim_view_name] = xform_prim_view.get_world_poses()

            for rigid_prim_view_name, rigid_prim_view in self.scene_registry.rigid_prim_views.items():
                bodies_positions[rigid_prim_view_name], bodies_quaternions[rigid_prim_view_name] = rigid_prim_view.get_world_poses()
                bodies_velocities[rigid_prim_view_name] = {}

            for articulation_view_name, articulation_view in self.scene_registry.articulated_views.items():
                bodies_positions[articulation_view_name], bodies_quaternions[articulation_view_name] = articulation_view.get_world_poses()
                bodies_velocities[articulation_view_name] = {}
                cmd_joints_values[articulation_view_name] = {}

            for object_name, attributes in self.request_meta_data.get("receive", {}).items():
                prim = self._receive_names_dict[object_name]
                prim_path = prim.GetPath()
                if prim_path in self._body_dict:
                    if prim_path in self._object_xform_prim_view_idx_dict:
                        view_name, object_idx = self._object_xform_prim_view_idx_dict[prim_path]
                    elif prim_path in self._object_rigid_prim_view_idx_dict:
                        view_name, object_idx = self._object_rigid_prim_view_idx_dict[prim_path]
                    elif prim_path in self._object_articulation_view_idx_dict:
                        view_name, object_idx = self._object_articulation_view_idx_dict[prim_path]
                    else:
                        raise Exception(f"Body {prim_path.pathString} not found in any view")

                    for attribute in attributes:
                        if attribute == "odometric_velocity" and prim_path in self.body_dict:
                            quaternion = bodies_quaternions[view_name][object_idx]

                            w = quaternion[0]
                            x = quaternion[1]
                            y = quaternion[2]
                            z = quaternion[3]

                            sinr_cosp = 2 * (w * x + y * z)
                            cosr_cosp = 1 - 2 * (x * x + y * y)
                            odom_ang_x_joint_pos = numpy.arctan2(sinr_cosp, cosr_cosp)

                            sinp = 2 * (w * y - z * x)
                            if numpy.abs(sinp) >= 1:
                                odom_ang_y_joint_pos = numpy.copysign(numpy.pi / 2, sinp)
                            else:
                                odom_ang_y_joint_pos = numpy.arcsin(sinp)

                            siny_cosp = 2 * (w * z + x * y)
                            cosy_cosp = 1 - 2 * (y * y + z * z)
                            odom_ang_z_joint_pos = numpy.arctan2(siny_cosp, cosy_cosp)

                            odom_velocity = receive_data[:6]
                            receive_data = receive_data[6:]

                            linear_velocity = [
                                odom_velocity[0] * numpy.cos(odom_ang_y_joint_pos) * numpy.cos(odom_ang_z_joint_pos)
                                + odom_velocity[1]
                                * (
                                    numpy.sin(odom_ang_x_joint_pos) * numpy.sin(odom_ang_y_joint_pos) * numpy.cos(odom_ang_z_joint_pos)
                                    - numpy.cos(odom_ang_x_joint_pos) * numpy.sin(odom_ang_z_joint_pos)
                                )
                                + odom_velocity[2]
                                * (
                                    numpy.cos(odom_ang_x_joint_pos) * numpy.sin(odom_ang_y_joint_pos) * numpy.cos(odom_ang_z_joint_pos)
                                    + numpy.sin(odom_ang_x_joint_pos) * numpy.sin(odom_ang_z_joint_pos)
                                ),
                                odom_velocity[0] * numpy.cos(odom_ang_y_joint_pos) * numpy.sin(odom_ang_z_joint_pos)
                                + odom_velocity[1]
                                * (
                                    numpy.sin(odom_ang_x_joint_pos) * numpy.sin(odom_ang_y_joint_pos) * numpy.sin(odom_ang_z_joint_pos)
                                    + numpy.cos(odom_ang_x_joint_pos) * numpy.cos(odom_ang_z_joint_pos)
                                )
                                + odom_velocity[2]
                                * (
                                    numpy.cos(odom_ang_x_joint_pos) * numpy.sin(odom_ang_y_joint_pos) * numpy.sin(odom_ang_z_joint_pos)
                                    - numpy.sin(odom_ang_x_joint_pos) * numpy.cos(odom_ang_z_joint_pos)
                                ),
                                odom_velocity[0] * numpy.sin(odom_ang_y_joint_pos)
                                + odom_velocity[1] * numpy.sin(odom_ang_x_joint_pos) * numpy.cos(odom_ang_y_joint_pos)
                                + odom_velocity[2] * numpy.cos(odom_ang_x_joint_pos) * numpy.cos(odom_ang_y_joint_pos),
                            ]

                            angular_velocity = [odom_velocity[3], odom_velocity[4], odom_velocity[5]]

                            bodies_velocities[view_name] = {object_idx: linear_velocity + angular_velocity}

                elif prim_path in self._joint_dict:
                    articulation_view_name, joint_idx = self._object_articulation_view_idx_dict[prim_path]
                    for attribute in attributes:
                        cmd_joints_values[articulation_view_name][joint_idx] = [0.0, 0.0, 0.0]
                        if attribute in ["cmd_joint_rvalue", "cmd_joint_tvalue"]:
                            cmd_joint_value = receive_data[0]
                            receive_data = receive_data[1:]
                            cmd_joints_values[articulation_view_name][joint_idx][0] = cmd_joint_value
                        elif attribute in ["cmd_joint_angular_velocity", "cmd_joint_linear_velocity"]:
                            cmd_joint_velocity = receive_data[0]
                            receive_data = receive_data[1:]
                            cmd_joints_values[articulation_view_name][joint_idx][1] = cmd_joint_velocity
                        elif attribute in ["cmd_joint_force", "cmd_joint_torque"]:
                            cmd_joint_effort = receive_data[0]
                            receive_data = receive_data[1:]
                            cmd_joints_values[articulation_view_name][joint_idx][2] = cmd_joint_effort

            for rigid_prim_view_name, rigid_prim_view in self.scene_registry.rigid_prim_views.items():
                if rigid_prim_view_name in bodies_velocities and len(bodies_velocities[rigid_prim_view_name]) > 0:
                    cmd_body_velocities = list(bodies_velocities[rigid_prim_view_name].values())
                    cmd_body_idxes = list(bodies_velocities[rigid_prim_view_name].keys())
                    rigid_prim_view.set_velocities(velocities=cmd_body_velocities, indices=cmd_body_idxes)

            for articulation_view_name, articulation_view in self.scene_registry.articulated_views.items():
                if articulation_view_name in bodies_velocities and len(bodies_velocities[articulation_view_name]) > 0:
                    cmd_body_velocities = list(bodies_velocities[articulation_view_name].values())
                    cmd_body_idxes = list(bodies_velocities[articulation_view_name].keys())
                    articulation_view.set_velocities(velocities=cmd_body_velocities, indices=cmd_body_idxes)

                if articulation_view_name in cmd_joints_values and len(cmd_joints_values[articulation_view_name]) > 0:
                    cmd_joint_values = cmd_joints_values[articulation_view_name].values()
                    cmd_joint_positions = [cmd_joint_value[0] for cmd_joint_value in cmd_joint_values]
                    cmd_joint_velocities = [cmd_joint_value[1] for cmd_joint_value in cmd_joint_values]
                    cmd_joint_efforts = [cmd_joint_value[2] for cmd_joint_value in cmd_joint_values]
                    if isinstance(self.world, SimulationContext):
                        cmd_joint_positions = torch.tensor(cmd_joint_positions).to("cuda").float()
                        cmd_joint_velocities = torch.tensor(cmd_joint_velocities).to("cuda").float()
                        cmd_joint_efforts = torch.tensor(cmd_joint_efforts).to("cuda").float()
                    cmd_joint_idxes = list(cmd_joints_values[articulation_view_name].keys())
                    action = ArticulationActions(
                        joint_positions=cmd_joint_positions,
                        joint_velocities=cmd_joint_velocities,
                        joint_efforts=cmd_joint_efforts,
                        joint_indices=cmd_joint_idxes)
                    articulation_view.apply_action(action)
        self.bind_receive_data_callback = bind_receive_data

        self.run()

        self.send_and_receive_meta_data()

    def __add_send_object(self, prim, attr: str):
        if prim not in self._send_objects:
            self._send_objects[prim] = []
        if attr not in self._send_objects[prim]:
            self._send_objects[prim].append(attr)

    def __remove_send_object(self, prim, attr: str):
        if prim in self._send_objects and attr in self._send_objects[prim]:
            self._send_objects[prim].remove(attr)
            if len(self._send_objects[prim]) == 0:
                del self._send_objects[prim]

    def __add_receive_object(self, prim, attr: str):
        if prim not in self._receive_objects:
            self._receive_objects[prim] = []
        if attr not in self._receive_objects[prim]:
            self._receive_objects[prim].append(attr)

    def __remove_receive_object(self, prim, attr: str):
        if prim in self._receive_objects and attr in self._receive_objects[prim]:
            self._receive_objects[prim].remove(attr)
            if len(self._receive_objects[prim]) == 0:
                del self._receive_objects[prim]

    def _clean_up(self, clean_ui=True):
        self._world = None
        self._scene = None
        self._send_objects = {}
        self._receive_objects = {}

        if clean_ui:
            self._send_prims_dict = {}
            self._receive_prims_dict = {}
            self._send_names_dict = {}
            self._receive_names_dict = {}
            self._send_prims_check_boxes = {}
            self._receive_prims_check_boxes = {}

            self._body_dict = {}
            self._body_collision_geom_path_dict = {}
            self._object_xform_prim_view_idx_dict = {}
            self._object_rigid_prim_view_idx_dict = {}
            self._object_rigid_contact_view_idx_dict = {}
            self._joint_dict = {}
            self._object_articulation_view_idx_dict = {}
            self._constrained_bodies = set()

    @property
    def scene_registry(self) -> "SceneRegistry":
        if self._scene is None:
            self._scene = Scene()
        return self._scene._scene_registry
    
    @property
    def world(self) -> "World":
        if self._world is None:
            self._world = World()
        return self._world
