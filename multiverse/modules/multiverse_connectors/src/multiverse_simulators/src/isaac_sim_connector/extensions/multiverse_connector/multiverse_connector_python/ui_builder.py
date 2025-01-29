# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import numpy as np
import omni.timeline
import omni.ui as ui
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.prims import get_prim_object_type
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.ui.element_wrappers import CollapsableFrame, DropDown, FloatField, TextBlock, StringField, IntField, CheckBox
from omni.ui import Button
from omni.isaac.ui.ui_utils import get_style


class UIBuilder:
    def __init__(self):
        # Frames are sub-windows that can contain multiple UI elements
        self.frames = []

        # UI elements created using a UIElementWrapper from omni.isaac.ui.element_wrappers
        self.wrapped_ui_elements = []

        self._send_object_names = {}
        self._receive_object_names = {}
        self._send_objects = {}
        self._receive_objects = {}
        self._send_objects_check_boxes = {}
        self._receive_objects_check_boxes = {}

        # Get access to the timeline to control stop/pause/play programmatically
        self._timeline = omni.timeline.get_timeline_interface()

        # Run initialization for the provided example
        self._on_init()

    def add_send_object(self, object_name: str, object_attribute: str):
        object_name = self._send_object_names[object_name].get_value()
        if object_name not in self._send_objects:
            self._send_objects[object_name] = set()
        self._send_objects[object_name].add(object_attribute)

    def remove_send_object(self, object_name: str, object_attribute: str):
        object_name = self._send_object_names[object_name].get_value()
        if object_name in self._send_objects and object_attribute in self._send_objects[object_name]:
            self._send_objects[object_name].remove(object_attribute)
            if len(self._send_objects[object_name]) == 0:
                del self._send_objects[object_name]

    def add_receive_object(self, object_field: StringField, object_attribute: str):
        object_name = self._receive_object_names[object_field]
        if object_name not in self._receive_objects:
            self._receive_objects[object_name] = set()
        self._receive_objects[object_name].add(object_attribute)

    def remove_receive_object(self, object_field: StringField, object_attribute: str):
        object_name = self._receive_object_names[object_field]
        if object_name in self._receive_objects and object_attribute in self._receive_objects[object_name]:
            self._receive_objects[object_name].remove(object_attribute)
            if len(self._receive_objects[object_name]) == 0:
                del self._receive_objects[object_name]

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
        print(f"Send Object Names: {[object_name.get_value() for object_name in self._send_object_names.values()]}")
        print(f"Receive Object Names: {[object_name.get_value() for object_name in self._receive_object_names.keys()]}")
        print(f"Sending objects: {self._send_objects}")
        print(f"Receiving objects: {self._receive_objects}")

    def on_stage_event(self, event):
        """Callback for Stage Events

        Args:
            event (omni.usd.StageEventType): Event Type
        """
        if event.type == int(omni.usd.StageEventType.ASSETS_LOADED):  # Any asset added or removed
            pass
        elif event.type == int(omni.usd.StageEventType.SIMULATION_START_PLAY):  # Timeline played
            self._init_objects()
        elif event.type == int(omni.usd.StageEventType.SIMULATION_STOP_PLAY):  # Timeline stopped
            # Ignore pause events
            if self._timeline.is_stopped():
                pass

    def cleanup(self):
        """
        Called when the stage is closed or the extension is hot reloaded.
        Perform any necessary cleanup such as removing active callback functions
        Buttons imported from omni.isaac.ui.element_wrappers implement a cleanup function that should be called
        """
        for ui_elem in self.wrapped_ui_elements:
            ui_elem.cleanup()

    def build_ui(self):
        """
        Build a custom UI tool to run your extension.
        This function will be called any time the UI window is closed and reopened.
        """
        selection_panel_frame = CollapsableFrame("Multiverse Connector", collapsed=False)

        with selection_panel_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._info_text = TextBlock(
                    "Info",
                    "Multiverse Client implementation in Isaac Sim.",
                    include_copy_button=False,
                    num_lines=1,
                )

                self._server_host_field = StringField(
                    label="Server Host",
                    tooltip="Enter the server host address.",
                    default_value="tcp://127.0.0.1"
                )
                self._server_port_field = IntField(
                    label="Server Port",
                    tooltip="Enter the server port number.",
                    default_value=7000,
                    lower_limit=1000,
                    upper_limit=9999
                )
                self._client_port_field = IntField(
                    label="Client Port",
                    tooltip="Enter the client port number.",
                    default_value=8000,
                    lower_limit=1000,
                    upper_limit=9999
                )
                self._world_name_field = StringField(
                    label="World Name",
                    tooltip="Enter the name of the world.",
                    default_value="world"
                )
                self._simulation_name_field = StringField(
                    label="Simulation Name",
                    tooltip="Enter the name of the simulation.",
                    default_value="isaac_sim"
                )

                import omni.usd
                from pxr import UsdPhysics, UsdGeom
                stage = omni.usd.get_context().get_stage()
                prims = {
                    "body": [],
                    "revolute_joint": [],
                    "prismatic_joint": []
                }
                if stage:
                    for prim in stage.Traverse():
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
                                        prim_name = prim.GetName()
                                        prim_frame = CollapsableFrame(
                                            title=prim_name,
                                            collapsed=True,
                                            enabled=True)
                                        with prim_frame:
                                            with ui.VStack():
                                                name_field = StringField(
                                                    label="name",
                                                    default_value=prim_name,
                                                    tooltip=f"Send name of {prim_name}."
                                                )
                                                self._send_object_names[prim_name] = name_field
                                                self._send_objects_check_boxes[prim_name] = []
                                                for attr in attributes[prim_type]:
                                                    check_box = CheckBox(
                                                        label=attr,
                                                        default_value=False,
                                                        tooltip=f"Send {attr} of {prim_name}."
                                                    )
                                                    self._send_objects_check_boxes[prim_name].append(check_box)

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
                                        prim_name = prim.GetName()
                                        prim_frame = CollapsableFrame(
                                            title=prim_name,
                                            collapsed=True,
                                            enabled=True)
                                        with prim_frame:
                                            with ui.VStack():
                                                name_field = StringField(
                                                    label="name",
                                                    default_value=prim_name,
                                                    tooltip=f"Receive name of {prim_name}."
                                                )
                                                self._receive_object_names[name_field] = prim_name
                                                self._receive_objects_check_boxes[name_field] = []
                                                for attr in attributes[prim_type]:
                                                    check_box = CheckBox(
                                                        label=attr,
                                                        default_value=False,
                                                        tooltip=f"Receive {attr} of {prim_name}."
                                                    )
                                                    self._receive_objects_check_boxes[name_field].append(check_box)

    ######################################################################################
    # Functions Below This Point Support The Provided Example And Can Be Replaced/Deleted
    ######################################################################################

    def _on_init(self):
        pass

    def _init_objects(self):
        self._send_objects = {}
        self._receive_objects = {}
        for object_name, check_boxes in self._send_objects_check_boxes.items():
            for check_box in check_boxes:
                if check_box.get_value():
                    self.add_send_object(object_name, check_box.label.text)
                else:
                    self.remove_send_object(object_name, check_box.label.text)
        for object_field, check_boxes in self._receive_objects_check_boxes.items():
            for check_box in check_boxes:
                if check_box.get_value():
                    self.add_receive_object(object_field, check_box.label.text)
                else:
                    self.remove_receive_object(object_field, check_box.label.text)

    # def _invalidate_articulation(self):
    #     """
    #     This function handles the event that the existing articulation becomes invalid and there is
    #     not a new articulation to select.  It is called explicitly in the code when the timeline is
    #     stopped and when the DropDown menu finds no articulations on the stage.
    #     """
    #     self.articulation = None
    #     self._robot_control_frame.collapsed = True
    #     self._robot_control_frame.enabled = False

    # def _on_articulation_selection(self, selection: str):
    #     """
    #     This function is called whenever a new selection is made in the
    #     "Select Articulation" DropDown.  A new selection may also be
    #     made implicitly any time self._selection_menu.repopulate() is called
    #     since the Articulation they had selected may no longer be present on the stage.

    #     Args:
    #         selection (str): The item that is currently selected in the drop-down menu.
    #     """
    #     # If the timeline is stopped, the Articulation won't be usable.
    #     if selection is None or self._timeline.is_stopped():
    #         self._invalidate_articulation()
    #         return

    #     self.articulation = Articulation(selection)
    #     self.articulation.initialize()

    #     self._robot_control_frame.collapsed = False
    #     self._robot_control_frame.enabled = True
    #     self._robot_control_frame.rebuild()

    # def _setup_joint_control_frames(self):
    #     """
    #     Once a robot has been chosen, update the UI to match robot properties:
    #         Make a frame visible for each robot joint.
    #         Rename each frame to match the human-readable name of the joint it controls.
    #         Change the FloatField for each joint to match the current robot position.
    #         Apply the robot's joint limits to each FloatField.
    #     """
    #     num_dof = self.articulation.num_dof
    #     dof_names = self.articulation.dof_names
    #     joint_positions = self.articulation.get_joint_positions()

    #     lower_joint_limits = self.articulation.dof_properties["lower"]
    #     upper_joint_limits = self.articulation.dof_properties["upper"]

    #     for i in range(num_dof):
    #         frame = self._joint_control_frames[i]
    #         position_float_field = self._joint_position_float_fields[i]

    #         # Write the human-readable names of each joint
    #         frame.title = dof_names[i]
    #         position = joint_positions[i]

    #         position_float_field.set_value(position)
    #         position_float_field.set_upper_limit(upper_joint_limits[i])
    #         position_float_field.set_lower_limit(lower_joint_limits[i])

    # def _on_set_joint_position_target(self, joint_index: int, position_target: float):
    #     """
    #     This function is called when the user changes one of the float fields
    #     to control a robot joint position target.  The index of the joint and the new
    #     desired value are passed in as arguments.

    #     This function assumes that there is a guarantee it is called safely.
    #     I.e. A valid Articulation has been selected and initialized
    #     and the timeline is playing.  These gurantees are given by careful UI
    #     programming.  The joint control frames are only visible to the user when
    #     these guarantees are met.

    #     Args:
    #         joint_index (int): Index of robot joint that was modified
    #         position_target (float): New position target for robot joint
    #     """
    #     robot_action = ArticulationAction(
    #         joint_positions=np.array([position_target]),
    #         joint_velocities=np.array([0]),
    #         joint_indices=np.array([joint_index]),
    #     )
    #     self.articulation.apply_action(robot_action)

    # def _find_all_articulations(self):
    # #    Commented code left in to help a curious user gain a thorough understanding

    #     import omni.usd
    #     from pxr import Usd
    #     items = []
    #     stage = omni.usd.get_context().get_stage()
    #     if stage:
    #         for prim in Usd.PrimRange(stage.GetPrimAtPath("/")):
    #             path = str(prim.GetPath())
    #             # Get prim type get_prim_object_type
    #             type = get_prim_object_type(path)
    #             if type == "articulation":
    #                 items.append(path)
    #     return items
