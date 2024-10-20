from multiverse_client_py import MultiverseClient, MultiverseMetaData, SocketAddress
from typing import Dict, List

class IsaacSimConnector(MultiverseClient):
    def __init__(self, 
                 client_addr: SocketAddress, 
                 multiverse_meta_data: MultiverseMetaData,
                 usd_path: str,
                 send_objects: Dict[str, List[str]],
                 receive_objects: Dict[str, List[str]]) -> None:
        super().__init__(client_addr, multiverse_meta_data)
        self.stage = Usd.Stage.Open(usd_path)
        self.body_name_dict = {}
        self.body_prim_view = None
        self.body_prim_view_idx_dict = {}
        self.joint_name_dict = {}
        self.joint_art_dict = {}
        self.init(send_objects, receive_objects)

    def init(self, send_objects: Dict[str, List[str]], receive_objects: Dict[str, List[str]]) -> None:
        request_meta_data = self.request_meta_data

        art_list = []

        for xform_prim in [prim for prim in self.stage.TraverseAll() if prim.IsA(UsdGeom.Xform)]:
            body_name = xform_prim.GetName()
            self.body_name_dict[body_name] = xform_prim

        for prim in self.stage.TraverseAll():
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                if prim.IsA(UsdGeom.Xform):
                    xform_prim = prim
                elif prim.IsA(UsdPhysics.Joint):
                    joint = UsdPhysics.Joint(prim)
                    xform_prim_path = joint.GetBody1Rel().GetTargets()[0]
                    xform_prim = self.stage.GetPrimAtPath(xform_prim_path)
                art = dc.get_articulation(xform_prim.GetPath().pathString)
                print(f"Articulation Root: {xform_prim.GetPath().pathString}, Articulation Handle: {art}")
                if art != 0 and art not in art_list:
                    art_list.append(art)

        for joint_prim in [prim for prim in self.stage.TraverseAll() if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint)]:
            joint_name = joint_prim.GetName()
            for art in art_list:
                dof_ptr = dc.find_articulation_dof(art, joint_name)
                if dof_ptr != 0:
                    self.joint_name_dict[joint_name] = joint_prim
                    self.joint_art_dict[joint_name] = art
                    break

        for object_name in sorted(receive_objects.keys()):
            request_meta_data["receive"][object_name] = []
            for attr in receive_objects[object_name]:
                request_meta_data["receive"][object_name].append(attr)

        if "body" in send_objects:
            for body_name in sorted(self.body_name_dict.keys()):
                request_meta_data["send"][body_name] = []
                for attr in send_objects["body"]:
                    if body_name in request_meta_data["receive"] and attr in request_meta_data["receive"][body_name]:
                        continue
                    request_meta_data["send"][body_name].append(attr)

        if "joint" in send_objects:
            for joint_name in sorted(self.joint_name_dict.keys()):
                request_meta_data["send"][joint_name] = []
                for attr in send_objects["joint"]:
                    if joint_name in request_meta_data["receive"] and attr in request_meta_data["receive"][joint_name]:
                        continue
                    joint_prim = self.joint_name_dict[joint_name]
                    if joint_prim.IsA(UsdPhysics.RevoluteJoint) and attr in ["joint_rvalue", "joint_angular_velocity", "joint_torque"] or \
                        joint_prim.IsA(UsdPhysics.PrismaticJoint) and attr in ["joint_tvalue", "joint_linear_velocity", "joint_force"]:
                        request_meta_data["send"][joint_name].append(attr)

        for object_name in sorted(send_objects.keys()):
            if object_name not in ["body", "joint"]:
                request_meta_data["send"][object_name] = []
                for attr in send_objects[object_name]:
                    request_meta_data["send"][object_name].append(attr)

        body_paths = []
        for object_name in list(request_meta_data["send"].keys()) + list(request_meta_data["receive"].keys()):
            if object_name in self.body_name_dict:
                body_paths.append(self.body_name_dict[object_name].GetPath().pathString)
                self.body_prim_view_idx_dict[object_name] = len(body_paths) - 1

        self.body_prim_view = RigidPrimView(prim_paths_expr=body_paths)

        self.request_meta_data = request_meta_data

    def bind_send_data(self) -> None:
        send_data = [self.world_time + self.sim_time]
        positions, quaternions = self.body_prim_view.get_world_poses()
        for object_name, attributes in self.request_meta_data["send"].items():
            if object_name in self.body_name_dict:
                for attribute in attributes:
                    if attribute == "position":
                        pos_idx = self.body_prim_view_idx_dict[object_name]
                        position = positions[pos_idx]
                        send_data += [position[0], position[1], position[2]]
                    elif attribute == "quaternion":
                        quat_idx = self.body_prim_view_idx_dict[object_name]
                        quaternion = quaternions[quat_idx]
                        send_data += [quaternion[0], quaternion[1], quaternion[2], quaternion[3]]

            elif object_name in self.joint_name_dict:
                art = self.joint_art_dict[object_name]
                dof_ptr = dc.find_articulation_dof(art, object_name)
                dof_state = dc.get_dof_state(dof_ptr, _dynamic_control.STATE_ALL)

                for attribute in attributes:
                    if attribute == "joint_rvalue":
                        send_data += [dof_state.pos]
                    elif attribute == "joint_tvalue":
                        send_data += [dof_state.pos]
                    elif attribute == "joint_linear_velocity":
                        send_data += [dof_state.vel]
                    elif attribute == "joint_angular_velocity":
                        send_data += [dof_state.vel]
                    elif attribute == "joint_force":
                        send_data += [dof_state.effort]
                    elif attribute == "joint_torque":
                        send_data += [dof_state.effort]

        self.send_data = send_data

    def bind_receive_data(self) -> None:
        receive_data = self.receive_data
        sim_time = receive_data[0]
        receive_data = receive_data[1:]
        for object_name, attributes in self.request_meta_data["receive"].items():
            for attribute in attributes:
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
                        odom_velocity[0] * numpy.cos(odom_ang_y_joint_pos) * numpy.cos(odom_ang_z_joint_pos) + odom_velocity[1] * (numpy.sin(odom_ang_x_joint_pos) * numpy.sin(odom_ang_y_joint_pos) * numpy.cos(odom_ang_z_joint_pos) - numpy.cos(odom_ang_x_joint_pos) * numpy.sin(odom_ang_z_joint_pos)) + odom_velocity[2] * (numpy.cos(odom_ang_x_joint_pos) * numpy.sin(odom_ang_y_joint_pos) * numpy.cos(odom_ang_z_joint_pos) + numpy.sin(odom_ang_x_joint_pos) * numpy.sin(odom_ang_z_joint_pos)),
                        odom_velocity[0] * numpy.cos(odom_ang_y_joint_pos) * numpy.sin(odom_ang_z_joint_pos) + odom_velocity[1] * (numpy.sin(odom_ang_x_joint_pos) * numpy.sin(odom_ang_y_joint_pos) * numpy.sin(odom_ang_z_joint_pos) + numpy.cos(odom_ang_x_joint_pos) * numpy.cos(odom_ang_z_joint_pos)) + odom_velocity[2] * (numpy.cos(odom_ang_x_joint_pos) * numpy.sin(odom_ang_y_joint_pos) * numpy.sin(odom_ang_z_joint_pos) - numpy.sin(odom_ang_x_joint_pos) * numpy.cos(odom_ang_z_joint_pos)),
                        odom_velocity[0] * numpy.sin(odom_ang_y_joint_pos) + odom_velocity[1] * numpy.sin(odom_ang_x_joint_pos) * numpy.cos(odom_ang_y_joint_pos) + odom_velocity[2] * numpy.cos(odom_ang_x_joint_pos) * numpy.cos(odom_ang_y_joint_pos)
                    ]
                    
                    angular_velocity = [
                        odom_velocity[3],
                        odom_velocity[4],
                        odom_velocity[5]
                    ]

                    dc.set_rigid_body_linear_velocity(rigid_body_handle, linear_velocity)
                    dc.set_rigid_body_angular_velocity(rigid_body_handle, angular_velocity)

                elif attribute == "cmd_joint_rvalue" or attribute == "cmd_joint_tvalue" and object_name in self.joint_name_dict:
                    art = self.joint_art_dict[object_name]
                    dof_ptr = dc.find_articulation_dof(art, object_name)
                    joint_value = receive_data[0]
                    receive_data = receive_data[1:]
                    dc.set_dof_position_target(dof_ptr, joint_value)

    def loginfo(self, message: str) -> None:
        print(f"INFO: {message}")

    def logwarn(self, message: str) -> None:
        print(f"WARN: {message}")

    def _run(self) -> None:
        self.loginfo("Start running the Isaac Sim Connector.")
        self._connect_and_start()

    def send_and_receive_meta_data(self) -> None:
        self.loginfo("Sending request meta data: " + str(self.request_meta_data))
        self._communicate(True)
        self.loginfo("Received response meta data: " + str(self.response_meta_data))

    def send_and_receive_data(self) -> None:
        self._communicate(False)


### Below is the code to run the Isaac Sim Connector

import argparse
import json
import sys
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
    parser.add_argument(
        "--usd_path", type=str, help="Absolute path to usd file", required=True
    )
    parser.add_argument(
        "--headless", default=False, action="store_true", help="Run stage headless"
    )

    parser.add_argument("--config_params", type=str, required=False)
    parser.add_argument("--multiverse_params", type=str, required=False)

    args, unknown = parser.parse_known_args()

    config_params = (
        json.loads(args.config_params) if args.config_params is not None else {}
    )
    multiverse_params = (
        json.loads(args.multiverse_params) if args.multiverse_params is not None else {}
    )

    # Start the Multiverse Client
    multiverse_server_params = multiverse_params.get("multiverse_server", {})
    SocketAddress.host = multiverse_server_params.get("host", "tcp://127.0.0.1")

    multiverse_client_params = multiverse_params.get("multiverse_client", {})
    client_port = multiverse_client_params.get("port", "2000")

    multiverse_client_meta_data_params = multiverse_client_params.get("meta_data", {})
    world_name = multiverse_client_meta_data_params.get("world_name", "world")
    simulation_name = multiverse_client_meta_data_params.get(
        "simulation_name", "isaac_sim"
    )

    resources = multiverse_client_params.get("resources", [])

    send_objects = multiverse_client_params.get("send", {})
    receive_objects = multiverse_client_params.get("receive", {})

    multiverse_meta_data = MultiverseMetaData(
        world_name=world_name, simulation_name=simulation_name
    )

    client_addr = SocketAddress(port=str(client_port))

    usd_path = args.usd_path

    # Start the omniverse application
    CONFIG["headless"] = args.headless
    CONFIG["open_usd"] = usd_path
    simulation_app = SimulationApp(CONFIG)

    import carb
    from omni.isaac.nucleus import is_file
    from pxr import Usd, UsdGeom, UsdPhysics, Gf
    from omni.isaac.core import SimulationContext
    from omni.isaac.core.prims import RigidPrimView
    from omni.isaac.core.utils.stage import is_stage_loading
    from omni.isaac.dynamic_control import _dynamic_control

    dc = _dynamic_control.acquire_dynamic_control_interface()

    simulation_context = SimulationContext()

    # make sure the file exists before we try to open it
    try:
        result = is_file(usd_path)
    except:
        result = False

    if not result:
        carb.log_error(
            f"the usd path {usd_path} could not be opened, please make sure that {args.usd_path} is a valid usd file"
        )
        simulation_app.close()
        sys.exit()

    # wait for things to load
    simulation_app.update()

    print("Loading stage...")

    while is_stage_loading():
        simulation_app.update()
    print("Loading Complete")

    simulation_context.initialize_physics()
    simulation_context.play()

    isaac_sim_connector = IsaacSimConnector(client_addr=client_addr, 
                                            multiverse_meta_data=multiverse_meta_data,
                                            usd_path=usd_path,
                                            send_objects=send_objects,
                                            receive_objects=receive_objects)
    isaac_sim_connector.run()

    isaac_sim_connector.send_and_receive_meta_data()

    simulation_context.reset()

    isaac_sim_connector.body_prim_view.initialize(simulation_context.physics_sim_view)

    while simulation_app.is_running():
        isaac_sim_connector.bind_send_data()
        isaac_sim_connector.send_and_receive_data()
        isaac_sim_connector.bind_receive_data()
        
        # Run in realtime mode, we don't specify the step size
        simulation_context.step(render=True)

    simulation_context.stop()
    simulation_app.close() # close Isaac Sim

    isaac_sim_connector.stop()
