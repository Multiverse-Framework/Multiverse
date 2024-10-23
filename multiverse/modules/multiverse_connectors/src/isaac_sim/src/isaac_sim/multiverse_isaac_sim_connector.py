from multiverse_client_py import MultiverseClient, MultiverseMetaData, SocketAddress
from typing import Dict, List


class IsaacSimConnector(MultiverseClient):
    def __init__(
        self,
        client_addr: SocketAddress,
        multiverse_meta_data: MultiverseMetaData,
        send_objects: Dict[str, List[str]],
        receive_objects: Dict[str, List[str]],
        resources: List[str],
    ) -> None:
        super().__init__(client_addr, multiverse_meta_data)
        self.body_name_dict = {}
        self.rigid_prim_view = None
        self.rigid_prim_view_idx_dict = {}
        self.joint_name_dict = {}
        self.articulation_views = {}
        self.articulation_views_idx_dict = {}
        self.ignore_names = ["defaultGroundPlane", "GroundPlane", "Environment", "OmniKit_Viewport_LightRig", "Lights"]
        self.send_objects = send_objects
        self.receive_objects = receive_objects
        self.resources = resources
        self.init_callbacks()

    def init_callbacks(self) -> None:
        def is_isaac_sim_response(args: List[str]) -> List[str]:
            return ["yes"]

        def pause_response(args: List[str]) -> List[str]:
            return ["paused"] if not simulation_context.is_playing() else ["failed to pause"]

        def unpause_response(args: List[str]) -> List[str]:
            return ["unpaused"] if simulation_context.is_playing() else ["failed to unpause"]

        self.api_callbacks_response = {"is_isaac_sim": is_isaac_sim_response, "pause": pause_response, "unpause": unpause_response}

        def pause(args: List[str]) -> None:
            simulation_context.pause()

        def unpause(args: List[str]) -> None:
            simulation_context.play()

        self.api_callbacks = {"pause": pause, "unpause": unpause}

        def bind_request_meta_data_callback() -> None:
            objects_to_spawn = set()
            objects_to_destroy = set()

            request_meta_data = self.request_meta_data
            for object_name in request_meta_data["send"]:
                if object_name in ["body", "joint"]:
                    continue
                if object_name not in self.body_name_dict and any(
                    attribute_name in ["position", "quaternion"] for attribute_name in request_meta_data["send"][object_name]
                ):
                    objects_to_spawn.add(object_name)
                elif (
                    len(request_meta_data["send"][object_name]) == 0
                    and object_name in self.request_meta_data["receive"]
                    and len(request_meta_data["receive"][object_name]) == 0
                ):
                    objects_to_destroy.add(object_name)

            for object_name in request_meta_data["receive"]:
                if object_name in ["body", "joint"] or object_name in self.body_name_dict:
                    continue
                if any(attribute_name in ["position", "quaternion"] for attribute_name in request_meta_data["receive"][object_name]):
                    objects_to_spawn.add(object_name)

            for object_name in objects_to_spawn:
                object_file_path = f"{object_name}.usda"
                self.loginfo(f"Searching for object {object_name} in {self.resources}...")
                for resource in self.resources:
                    for root, _, files in os.walk(resource):
                        if object_file_path in files:
                            object_file_path = os.path.join(root, object_file_path)
                            break
                    if os.path.exists(object_file_path):
                        self.loginfo(f"Found object {object_name} in {object_file_path}.")
                        prims_utils.create_prim(prim_path=f"/{object_name}", prim_type="Xform", usd_path=object_file_path)
                        self.loginfo(f"Spawned object {object_name}.")
                        if object_name in request_meta_data["send"]:
                            send_objects[object_name] = request_meta_data["send"][object_name]
                        if object_name in request_meta_data["receive"]:
                            receive_objects[object_name] = request_meta_data["receive"][object_name]
                        break
                else:
                    self.logwarn(f"Object {object_name} not found in {self.resources}.")
                    del request_meta_data["send"][object_name]
                    del request_meta_data["receive"][object_name]

            for object_name in objects_to_destroy:
                if object_name in self.body_name_dict:
                    prims_utils.delete_prim(prim_path=f"/{object_name}")
                    if object_name in send_objects:
                        del send_objects[object_name]
                    if object_name in receive_objects:
                        del receive_objects[object_name]
                    self.loginfo(f"Destroyed object {object_name}.")
                del request_meta_data["send"][object_name]
                del request_meta_data["receive"][object_name]

            self.request_meta_data = request_meta_data
            self.send_objects = self.request_meta_data["send"]
            self.receive_objects = self.request_meta_data["receive"]
            self.loginfo("Sending request meta data: " + str(self.request_meta_data))

        self.bind_request_meta_data_callback = bind_request_meta_data_callback

        def bind_response_meta_data_callback() -> None:
            response_meta_data = self.response_meta_data
            self.loginfo("Received response meta data: " + str(response_meta_data))

            body_positions = {}
            body_quaternions = {}
            body_velocities = {}
            joints_state = {}
            measured_joint_efforts = {}

            if self.rigid_prim_view is not None:
                body_positions[self.rigid_prim_view], body_quaternions[self.rigid_prim_view] = self.rigid_prim_view.get_world_poses()
                body_velocities[self.rigid_prim_view] = self.rigid_prim_view.get_velocities()

            for articulation_view in self.articulation_views.values():
                body_positions[articulation_view], body_quaternions[articulation_view] = articulation_view.get_world_poses()
                body_velocities[articulation_view] = articulation_view.get_velocities()
                joints_state[articulation_view] = articulation_view.get_joints_state()
                measured_joint_efforts[articulation_view] = articulation_view.get_measured_joint_efforts()
            
            for send_receive in ["send", "receive"]:
                if send_receive not in response_meta_data:
                    continue
                for object_name, attributes in response_meta_data[send_receive].items():
                    if object_name in self.body_name_dict:
                        view = None
                        if object_name in self.articulation_views_idx_dict:
                            view, object_idx = self.articulation_views_idx_dict[object_name]
                        elif object_name in self.rigid_prim_view_idx_dict:
                            view = self.rigid_prim_view
                            object_idx = self.rigid_prim_view_idx_dict[object_name]
                        for attribute in attributes:
                            data = response_meta_data[send_receive][object_name][attribute]
                            if any(x is None for x in data):
                                continue
                            if attribute == "position":
                                body_positions[view][object_idx] = [data[0], data[1], data[2]]
                            elif attribute == "quaternion":
                                body_quaternions[view][object_idx] = [data[0], data[1], data[2], data[3]]
            
            if self.rigid_prim_view is not None:
                self.rigid_prim_view.set_world_poses(body_positions[self.rigid_prim_view], body_quaternions[self.rigid_prim_view])

            for articulation_view in self.articulation_views.values():
                articulation_view.set_world_poses(body_positions[articulation_view], body_quaternions[articulation_view])

        self.bind_response_meta_data_callback = bind_response_meta_data_callback

        def init_objects_callback() -> None:
            request_meta_data = self.request_meta_data
            current_stage = stage.get_current_stage()
            articulation_prim_paths = []
            self.body_name_dict = {}
            self.rigid_prim_view = None
            self.rigid_prim_view_idx_dict = {}
            self.joint_name_dict = {}
            self.articulation_views = {}
            self.articulation_views_idx_dict = {}

            for prim in current_stage.TraverseAll():
                if prim.IsA(UsdGeom.Xform):
                    body_name = prim.GetName()
                    if body_name in self.ignore_names:
                        continue
                    self.body_name_dict[body_name] = prim
                if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                    if prim.IsA(UsdGeom.Xform):
                        xform_prim_path = prim.GetPath().pathString
                    elif prim.IsA(UsdPhysics.Joint):
                        joint = UsdPhysics.Joint(prim)
                        xform_prim_path = joint.GetBody1Rel().GetTargets()[0].pathString
                    articulation_prim_paths.append(xform_prim_path)

            non_free_bodies = set()
            for joint_prim in [
                prim for prim in current_stage.TraverseAll() if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint)
            ]:
                joint = UsdPhysics.Joint(joint_prim)
                body0_name = joint.GetBody0Rel().GetTargets()[0].name
                body1_name = joint.GetBody1Rel().GetTargets()[0].name
                non_free_bodies.add(body0_name)
                non_free_bodies.add(body1_name)
                joint_name = joint_prim.GetName()
                if joint_name in self.ignore_names:
                    continue
                self.joint_name_dict[joint_prim.GetName()] = joint_prim

            send_object_names = sorted(request_meta_data["send"].keys())
            for object_name in send_object_names:
                if object_name not in self.body_name_dict and object_name not in self.joint_name_dict:
                    del request_meta_data["send"][object_name]

            receive_object_names = sorted(request_meta_data["receive"].keys())
            for object_name in receive_object_names:
                if object_name not in self.body_name_dict and object_name not in self.joint_name_dict:
                    del request_meta_data["receive"][object_name]

            for object_name in sorted(self.receive_objects.keys()):
                request_meta_data["receive"][object_name] = []
                for attr in self.receive_objects[object_name]:
                    request_meta_data["receive"][object_name].append(attr)

            if "body" in self.send_objects:
                for body_name in sorted(self.body_name_dict.keys()):
                    request_meta_data["send"][body_name] = []
                    for attr in self.send_objects["body"]:
                        if body_name in request_meta_data["receive"] and attr in request_meta_data["receive"][body_name]:
                            continue
                        if attr == "relative_velocity" and body_name in non_free_bodies:
                            continue
                        request_meta_data["send"][body_name].append(attr)

            if "joint" in self.send_objects:
                for joint_name in sorted(self.joint_name_dict.keys()):
                    request_meta_data["send"][joint_name] = []
                    for attr in self.send_objects["joint"]:
                        if joint_name in request_meta_data["receive"] and attr in request_meta_data["receive"][joint_name]:
                            continue
                        joint_prim = self.joint_name_dict[joint_name]
                        if (
                            joint_prim.IsA(UsdPhysics.RevoluteJoint)
                            and attr in ["joint_rvalue", "joint_angular_velocity", "joint_torque"]
                            or joint_prim.IsA(UsdPhysics.PrismaticJoint)
                            and attr in ["joint_tvalue", "joint_linear_velocity", "joint_force"]
                        ):
                            request_meta_data["send"][joint_name].append(attr)

            for object_name in sorted(self.send_objects.keys()):
                if object_name not in ["body", "joint"]:
                    request_meta_data["send"][object_name] = []
                    for attr in self.send_objects[object_name]:
                        request_meta_data["send"][object_name].append(attr)

            body_paths = []
            for object_name in list(request_meta_data["send"].keys()) + list(request_meta_data["receive"].keys()):
                for articulation_prim_path in articulation_prim_paths:
                    articulation = dc.get_articulation(articulation_prim_path)
                    if object_name in self.joint_name_dict:
                        dof_ptr = dc.find_articulation_dof(articulation, object_name)
                        if dof_ptr == 0:
                            continue
                    elif object_name in self.body_name_dict:
                        rigid_body_handle = dc.get_rigid_body(self.body_name_dict[object_name].GetPath().pathString)
                        if rigid_body_handle == 0:
                            continue
                    if articulation_prim_path not in self.articulation_views:
                        articulation_view = ArticulationView(prim_paths_expr=articulation_prim_path)
                        articulation_view.initialize(simulation_context.physics_sim_view)
                        self.articulation_views[articulation_prim_path] = articulation_view
                    else:
                        articulation_view = self.articulation_views[articulation_prim_path]
                    if object_name in self.joint_name_dict and object_name in articulation_view.dof_names:
                        joint_idx = articulation_view.get_dof_index(object_name)
                        self.articulation_views_idx_dict[object_name] = (articulation_view, joint_idx)
                    elif object_name in self.body_name_dict and object_name == Sdf.Path(articulation_prim_path).name:
                        body_idx = articulation_view.get_link_index(object_name)
                        self.articulation_views_idx_dict[object_name] = (articulation_view, body_idx)
                    if object_name in self.articulation_views_idx_dict:
                        break
                else:
                    if object_name in self.body_name_dict:
                        body_paths.append(self.body_name_dict[object_name].GetPath().pathString)
                        self.rigid_prim_view_idx_dict[object_name] = len(body_paths) - 1

            if len(body_paths) > 0:
                self.rigid_prim_view = RigidPrimView(prim_paths_expr=body_paths)

            self.request_meta_data = request_meta_data

        self.init_objects_callback = init_objects_callback

        def bind_send_data() -> None:
            send_data = [self.world_time + self.sim_time]
            body_positions = {}
            body_quaternions = {}
            body_velocities = {}
            joints_state = {}
            measured_joint_efforts = {}

            if self.rigid_prim_view is not None:
                body_positions[self.rigid_prim_view], body_quaternions[self.rigid_prim_view] = self.rigid_prim_view.get_world_poses()
                body_velocities[self.rigid_prim_view] = self.rigid_prim_view.get_velocities()

            for articulation_view in self.articulation_views.values():
                body_positions[articulation_view], body_quaternions[articulation_view] = articulation_view.get_world_poses()
                body_velocities[articulation_view] = articulation_view.get_velocities()
                joints_state[articulation_view] = articulation_view.get_joints_state()
                measured_joint_efforts[articulation_view] = articulation_view.get_measured_joint_efforts()

            request_meta_data = self.request_meta_data
            for object_name, attributes in request_meta_data.get("send", {}).items():
                if object_name in self.body_name_dict:
                    view = None
                    if object_name in self.articulation_views_idx_dict:
                        view, object_idx = self.articulation_views_idx_dict[object_name]
                    elif object_name in self.rigid_prim_view_idx_dict:
                        view = self.rigid_prim_view
                        object_idx = self.rigid_prim_view_idx_dict[object_name]
                    else:
                        raise Exception(f"Object {object_name} not found in any view")

                    for attribute in self.send_objects[object_name]:
                        if attribute == "position":
                            position = body_positions[view][object_idx]
                            send_data += [position[0], position[1], position[2]]
                        elif attribute == "quaternion":
                            quaternion = body_quaternions[view][object_idx]
                            send_data += [quaternion[0], quaternion[1], quaternion[2], quaternion[3]]
                        elif attribute == "relative_velocity":
                            velocity = body_velocities[view][object_idx]
                            send_data += [velocity[0], velocity[1], velocity[2], velocity[3], velocity[4], velocity[5]]

                elif object_name in self.joint_name_dict:
                    articulation_view, joint_idx = self.articulation_views_idx_dict[object_name]
                    for attribute in self.send_objects[object_name]:
                        if attribute == "joint_rvalue":
                            send_data += [joints_state[articulation_view].positions[0][joint_idx]]
                        elif attribute == "joint_tvalue":
                            send_data += [joints_state[articulation_view].positions[0][joint_idx]]
                        elif attribute == "joint_linear_velocity":
                            send_data += [joints_state[articulation_view].velocities[0][joint_idx]]
                        elif attribute == "joint_angular_velocity":
                            send_data += [joints_state[articulation_view].velocities[0][joint_idx]]
                        elif attribute == "joint_force":
                            send_data += [measured_joint_efforts[articulation_view][0][joint_idx]]
                        elif attribute == "joint_torque":
                            send_data += [measured_joint_efforts[articulation_view][0][joint_idx]]

            self.send_data = send_data

        self.bind_send_data_callback = bind_send_data

        def bind_receive_data() -> None:
            receive_data = self.receive_data
            sim_time = receive_data[0]
            receive_data = receive_data[1:]
            cmd_joint_values = {}
            for articulation_view in self.articulation_views.values():
                cmd_joint_values[articulation_view] = articulation_view.get_joint_positions()

            active_articulation_views = set()
            request_meta_data = self.request_meta_data
            for object_name, attributes in request_meta_data.get("receive", {}).items():
                for attribute in self.receive_objects[object_name]:
                    if attribute == "odometric_velocity" and object_name in self.body_name_dict:
                        body_prim = self.body_name_dict[object_name]
                        rigid_body_handle = dc.get_rigid_body(body_prim.GetPath().pathString)
                        body_pose = dc.get_rigid_body_pose(rigid_body_handle)

                        w = body_pose.r[3]
                        x = body_pose.r[0]
                        y = body_pose.r[1]
                        z = body_pose.r[2]

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

                        dc.set_rigid_body_linear_velocity(rigid_body_handle, linear_velocity)
                        dc.set_rigid_body_angular_velocity(rigid_body_handle, angular_velocity)

                    elif attribute == "cmd_joint_rvalue" or attribute == "cmd_joint_tvalue" and object_name in self.joint_name_dict:
                        articulation_view, joint_idx = self.articulation_views_idx_dict[object_name]
                        cmd_joint_value = receive_data[0]
                        receive_data = receive_data[1:]
                        cmd_joint_values[articulation_view][0][joint_idx] = cmd_joint_value
                        active_articulation_views.add(articulation_view)

            for articulation_view in active_articulation_views:
                action = ArticulationActions(joint_positions=cmd_joint_values[articulation_view])
                articulation_view.apply_action(action)
        self.bind_receive_data_callback = bind_receive_data

    def loginfo(self, message: str) -> None:
        print(f"INFO: {message}")

    def logwarn(self, message: str) -> None:
        print(f"WARN: {message}")

    def _run(self) -> None:
        self.loginfo("Start running the Isaac Sim Connector.")
        self._connect_and_start()

    def send_and_receive_meta_data(self) -> None:
        self._communicate(True)

    def send_and_receive_data(self) -> None:
        self._communicate(False)


### Below is the code to run the Isaac Sim Connector

import argparse
import json
import sys
import os
import numpy
from isaacsim import SimulationApp

if __name__ == "__main__":
    # This sample loads a usd stage and starts simulation
    CONFIG = {
        "width": 1280,
        "height": 720,
        "sync_loads": True,
        "headless": False,
        "renderer": "RayTracedLighting",
        "hide_ui": False,
    }

    # Set up command line arguments
    parser = argparse.ArgumentParser("Multiverse Isaac Sim Connector")
    parser.add_argument("--usd_path", type=str, help="Absolute path to usd file", required=True)
    parser.add_argument("--headless", default=False, action="store_true", help="Run stage headless")

    parser.add_argument("--config_params", type=str, required=False)
    parser.add_argument("--multiverse_params", type=str, required=False)

    args, unknown = parser.parse_known_args()

    config_params = json.loads(args.config_params) if args.config_params is not None else {}
    multiverse_params = json.loads(args.multiverse_params) if args.multiverse_params is not None else {}

    # Start the Multiverse Client
    multiverse_server_params = multiverse_params.get("multiverse_server", {})
    SocketAddress.host = multiverse_server_params.get("host", "tcp://127.0.0.1")

    multiverse_client_params = multiverse_params.get("multiverse_client", {})
    client_port = multiverse_client_params.get("port", "2000")

    multiverse_client_meta_data_params = multiverse_client_params.get("meta_data", {})
    world_name = multiverse_client_meta_data_params.get("world_name", "world")
    simulation_name = multiverse_client_meta_data_params.get("simulation_name", "isaac_sim")

    resources = multiverse_client_params.get("resources", [])

    send_objects = multiverse_client_params.get("send", {})
    receive_objects = multiverse_client_params.get("receive", {})

    multiverse_meta_data = MultiverseMetaData(world_name=world_name, simulation_name=simulation_name)

    client_addr = SocketAddress(port=str(client_port))

    usd_path = args.usd_path

    # Start the omniverse application
    CONFIG["headless"] = args.headless
    CONFIG["open_usd"] = usd_path
    simulation_app = SimulationApp(CONFIG)

    import carb
    from omni.isaac.nucleus import is_file
    from pxr import UsdGeom, UsdPhysics, Sdf
    from omni.isaac.core import SimulationContext
    from omni.isaac.core.prims import RigidPrimView
    from omni.isaac.core.articulations import ArticulationView
    from omni.isaac.core.utils.types import ArticulationActions
    from omni.isaac.core.utils import stage
    from omni.isaac.dynamic_control import _dynamic_control
    import omni.isaac.core.utils.prims as prims_utils

    dc = _dynamic_control.acquire_dynamic_control_interface()

    simulation_context = SimulationContext()

    # make sure the file exists before we try to open it
    try:
        result = is_file(usd_path)
    except:
        result = False

    if not result:
        carb.log_error(f"the usd path {usd_path} could not be opened, please make sure that {args.usd_path} is a valid usd file")
        simulation_app.close()
        sys.exit()

    # wait for things to load
    simulation_app.update()

    print("Loading stage...")

    while stage.is_stage_loading():
        simulation_app.update()
    print("Loading Complete")

    simulation_context.initialize_physics()
    simulation_context.play()

    isaac_sim_connector = IsaacSimConnector(
        client_addr=client_addr,
        multiverse_meta_data=multiverse_meta_data,
        send_objects=send_objects,
        receive_objects=receive_objects,
        resources=resources,
    )

    simulation_context.reset()

    isaac_sim_connector.run()

    while simulation_app.is_running():
        isaac_sim_connector.send_and_receive_data()

        # Run in realtime mode, we don't specify the step size
        simulation_context.step(render=True)

    simulation_context.stop()
    simulation_app.close()  # close Isaac Sim

    isaac_sim_connector.stop()
