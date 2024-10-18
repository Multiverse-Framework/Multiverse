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
        self.body_dict = {}
        self.joint_dict = {}
        self.init(send_objects, receive_objects)

    def init(self, send_objects: Dict[str, List[str]], receive_objects: Dict[str, List[str]]) -> None:
        request_meta_data = self.request_meta_data

        for xform_prim in [prim for prim in self.stage.TraverseAll() if prim.IsA(UsdGeom.Xform)]:
            body_name = xform_prim.GetName()
            self.body_dict[body_name] = xform_prim

        for joint_prim in [prim for prim in self.stage.TraverseAll() if prim.IsA(UsdPhysics.Joint)]:
            joint_name = joint_prim.GetName()
            self.joint_dict[joint_name] = joint_prim

        if "body" in send_objects:
            for body_name in sorted(self.body_dict.keys()):
                request_meta_data["send"][body_name] = []
                for attr in send_objects["body"]:
                    request_meta_data["send"][body_name].append(attr)

        if "joint" in send_objects:
            for joint_name in sorted(self.joint_dict.keys()):
                request_meta_data["send"][joint_name] = []
                for attr in send_objects["joint"]:
                    request_meta_data["send"][joint_name].append(attr)

        self.request_meta_data = request_meta_data

    def bind_send_data(self) -> None:
        send_data = [self.world_time + self.sim_time]
        for object_name, attributes in self.request_meta_data["send"].items():
            if object_name in self.body_dict:
                object_prim = self.body_dict[object_name]
                if not object_prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    parent_prim = object_prim.GetParent()
                    while not parent_prim.IsPseudoRoot():
                        if parent_prim.HasAPI(UsdPhysics.RigidBodyAPI):
                            break
                        parent_prim = parent_prim.GetParent()
                    
                    relative_pose, _ = xform_cache.ComputeRelativeTransform(object_prim, parent_prim)
                    if parent_prim.IsPseudoRoot():
                        body_pose = relative_pose
                    else:
                        rigid_body_handle = dc.get_rigid_body(parent_prim.GetPath().pathString)
                        parent_body_pose = dc.get_rigid_body_pose(rigid_body_handle)
                        parent_body_pose_mat = Gf.Matrix4d()
                        parent_body_pose_mat.SetTranslateOnly(Gf.Vec3d(*parent_body_pose.p))
                        parent_body_pose_mat.SetRotateOnly(Gf.Quatd(parent_body_pose.r[3], Gf.Vec3d(*parent_body_pose.r[:3])))
                        body_pose = relative_pose * parent_body_pose_mat

                    p = body_pose.ExtractTranslation()
                    q = body_pose.ExtractRotation().GetQuaternion()
                    q = [q.GetImaginary()[0], q.GetImaginary()[1], q.GetImaginary()[2], q.GetReal()]
                else:
                    rigid_body_handle = dc.get_rigid_body(object_prim.GetPath().pathString)
                    body_pose = dc.get_rigid_body_pose(rigid_body_handle)
                    p = body_pose.p
                    q = body_pose.r

                position = [p[0], p[1], p[2]]
                quaternion = [q[3], q[0], q[1], q[2]]
                for attribute in attributes:
                    if attribute == "position":
                        send_data += position
                    elif attribute == "quaternion":
                        send_data += quaternion
        self.send_data = send_data

    def bind_receive_data(self) -> None:
        pass

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
    from omni.isaac.core.utils.stage import is_stage_loading
    from omni.isaac.dynamic_control import _dynamic_control

    xform_cache = UsdGeom.XformCache()

    dc = _dynamic_control.acquire_dynamic_control_interface()

    simulation_context = SimulationContext()

    isaac_sim_connector = IsaacSimConnector(client_addr=client_addr, 
                                            multiverse_meta_data=multiverse_meta_data,
                                            usd_path=usd_path,
                                            send_objects=send_objects,
                                            receive_objects=receive_objects)
    isaac_sim_connector.run()

    isaac_sim_connector.send_and_receive_meta_data()

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

    simulation_context.reset()

    while simulation_app.is_running():
        isaac_sim_connector.bind_send_data()
        isaac_sim_connector.send_and_receive_data()
        isaac_sim_connector.bind_receive_data()
        
        # Run in realtime mode, we don't specify the step size
        simulation_context.step(render=True)

    simulation_context.stop()
    simulation_app.close() # close Isaac Sim

    isaac_sim_connector.stop()
