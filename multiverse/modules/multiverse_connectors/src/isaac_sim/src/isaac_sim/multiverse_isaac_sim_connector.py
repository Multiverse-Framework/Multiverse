from multiverse_client_py import MultiverseClient, MultiverseMetaData, SocketAddress
from typing import Dict, List
from dataclasses import dataclass
from time import sleep

@dataclass
class MultiverseObject:
    prim: ...
    root_prim: ...

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
        self.body_dict = {}
        self.object_xform_prim_view_idx_dict = {}
        self.object_rigid_prim_view_idx_dict = {}
        self.joint_dict = {}
        self.object_articulation_view_idx_dict = {}
        self.constrained_bodies = set()
        self.ignore_names = ["defaultGroundPlane", "GroundPlane", "Environment", "OmniKit_Viewport_LightRig", "Lights"]
        self.send_objects = send_objects
        self.receive_objects = receive_objects
        self.resources = resources
        self.init_callbacks()

    @property
    def scene_registry(self) -> "SceneRegistry":
        return world.scene._scene_registry

    def init_callbacks(self) -> None:
        def get_is_isaac_sim_response(args: List[str]) -> List[str]:
            return ["yes"]

        def get_pause_response(args: List[str]) -> List[str]:
            return ["paused"] if not world.is_playing() else ["failed to pause"]

        def get_unpause_response(args: List[str]) -> List[str]:
            return ["unpaused"] if world.is_playing() else ["failed to unpause"]
        
        def get_attach_response(args: List[str]) -> List[str]:
            if not isinstance(args, list) or any(not isinstance(x, str) for x in args) or len(args) < 2 or len(args) > 3:
                return ["failed (Arguments for attach should be an array of strings with 2 or 3 elements.)"]
            
            object_1_name = args[0]
            object_2_name = args[1]

            if object_1_name not in self.body_dict:
                return [f"failed (Object {object_1_name} does not exist.)"]
            
            if object_2_name not in self.body_dict:
                return [f"failed (Object {object_2_name} does not exist.)"]
            
            object_1_path = self.body_dict[object_1_name].root_prim.GetPath()
            object_2_path = self.body_dict[object_2_name].prim.GetPath()
            
            if object_1_path.GetParentPath() == object_2_path:
                if len(args) == 3:
                    if object_1_name in self.object_xform_prim_view_idx_dict:
                        xform_prim_view_name, object_idx = self.object_xform_prim_view_idx_dict[object_1_name]
                        view = self.scene_registry.xform_prim_views[xform_prim_view_name]
                    elif object_1_name in self.object_rigid_prim_view_idx_dict:
                        rigid_prim_view_name, object_idx = self.object_rigid_prim_view_idx_dict[object_1_name]
                        view = self.scene_registry.rigid_prim_views[rigid_prim_view_name]
                    elif object_1_name in self.object_articulation_view_idx_dict:
                        articulation_view_name, object_idx = self.object_articulation_view_idx_dict[object_1_name]
                        view = self.scene_registry.articulated_views[articulation_view_name]
                    else:
                        raise Exception(f"Body {object_1_name} not found in any view")
                    positions, quaternions = view.get_local_poses()
                    position = positions[object_idx]
                    quaternion = quaternions[object_idx]
                    relative_pose = [float(x) for x in args[2].split()]
                    if not numpy.isclose(position, relative_pose[:3], atol=0.01).all() or not numpy.isclose(quaternion, relative_pose[3:], atol=0.01).all():
                        return ["failed (Relative pose are different)"]
                return ["success"]
            else:
                return ["failed (Attachment not found)"]
            
        def get_detach_response(args: List[str]) -> List[str]:
            if not isinstance(args, list) or any(not isinstance(x, str) for x in args) or len(args) < 2 or len(args) > 3:
                return ["failed (Arguments for detach should be an array of strings with 2 or 3 elements.)"]
            
            object_1_name = args[0]
            object_2_name = args[1]

            if object_1_name not in self.body_dict:
                return [f"failed (Object {object_1_name} does not exist.)"]
            
            if object_2_name not in self.body_dict:
                return [f"failed (Object {object_2_name} does not exist.)"]
            
            object_1_path = self.body_dict[object_1_name].root_prim.GetPath()
            object_2_path = self.body_dict[object_2_name].prim.GetPath()

            return  ["failed (Attachment found)"] if object_1_path.GetParentPath() == object_2_path else ["success"]

        self.api_callbacks_response = {"is_isaac_sim": get_is_isaac_sim_response, 
                                       "pause": get_pause_response, 
                                       "unpause": get_unpause_response, 
                                       "attach": get_attach_response,
                                       "detach": get_detach_response}

        def pause(args: List[str]) -> None:
            world.pause()

        def unpause(args: List[str]) -> None:
            world.play()

        def attach(args: List[str]) -> None:
            attach_response = get_attach_response(args)
            if attach_response[0] == "success":
                self.loginfo("Attachment already exists")
                return

            if attach_response[0] != "failed (Relative pose are different)" and attach_response[0] != "failed (Attachment not found)":
                self.logwarn(attach_response[0])
                return

            object_1_name = args[0]
            object_1_root_path = self.body_dict[object_1_name].root_prim.GetPath()
            object_1_path = self.body_dict[object_1_name].prim.GetPath()
            object_2_name = args[1]
            object_2_path = self.body_dict[object_2_name].prim.GetPath()

            if attach_response[0] == "failed (Attachment not found)":
                object_1_new_root_path = object_2_path.AppendChild(object_1_root_path.name)
                object_1_new_path = object_1_new_root_path.AppendChild(object_1_path.name)

                self._remove_all_object_views()
                
                prims_utils.move_prim(object_1_root_path, object_1_new_root_path)

                current_stage = stage.get_current_stage()
                self.body_dict[object_1_name] = MultiverseObject(current_stage.GetPrimAtPath(object_1_new_path), 
                                                                 current_stage.GetPrimAtPath(object_1_new_root_path))

                prims_utils.set_prim_attribute_value(object_1_new_path.pathString, attribute_name="physics:rigidBodyEnabled", value=False)

                object_1_path = self.body_dict[object_1_name].prim.GetPath()
                object_1_root_path = self.body_dict[object_1_name].root_prim.GetPath()

            if len(args) == 3:
                prims_utils.set_prim_attribute_value(object_1_root_path.pathString, attribute_name="xformOpOrder", value=["xformOp:translate", "xformOp:orient", "xformOp:scale"])
                prims_utils.set_prim_attribute_value(object_1_root_path.pathString, attribute_name="xformOp:translate", value=[0, 0, 0])
                prims_utils.set_prim_attribute_value(object_1_root_path.pathString, attribute_name="xformOp:orient", value=[1, 0, 0, 0])

                relative_pose = args[2].split()
                positions = [float(x) for x in relative_pose[:3]]
                quaternions = [float(x) for x in relative_pose[3:]]
                prims_utils.set_prim_attribute_value(object_1_path.pathString, attribute_name="xformOpOrder", value=["xformOp:translate", "xformOp:orient", "xformOp:scale"])
                prims_utils.set_prim_attribute_value(object_1_path.pathString, attribute_name="xformOp:translate", value=positions)
                prims_utils.set_prim_attribute_value(object_1_path.pathString, attribute_name="xformOp:orient", value=quaternions)

        def detach(args: List[str]) -> None:
            detach_response = get_detach_response(args)
            if detach_response[0] == "success":
                self.loginfo("Attachment not found, already detached.")
                return

            if detach_response[0] != "failed (Attachment found)":
                self.logwarn(detach_response[0])
                return

            object_1_name = args[0]
            object_1_root_path = self.body_dict[object_1_name].root_prim.GetPath()
            object_1_path = self.body_dict[object_1_name].prim.GetPath()

            object_1_new_root_path = Sdf.Path(f"/{object_1_root_path.name}")
            object_1_new_path = object_1_new_root_path.AppendChild(object_1_path.name)

            self._remove_all_object_views()
            
            prims_utils.move_prim(object_1_root_path, object_1_new_root_path)

            current_stage = stage.get_current_stage()
            self.body_dict[object_1_name] = MultiverseObject(current_stage.GetPrimAtPath(object_1_new_path), 
                                                             current_stage.GetPrimAtPath(object_1_new_root_path))

            prims_utils.set_prim_attribute_value(object_1_new_path.pathString, attribute_name="physics:rigidBodyEnabled", value=True)

            if not world.is_playing():
                world.play()
                world.pause()

        self.api_callbacks = {"pause": pause, 
                              "unpause": unpause, 
                              "attach": attach,
                              "detach": detach}

        def bind_request_meta_data_callback() -> None:
            objects_to_spawn = set()
            objects_to_destroy = set()

            request_meta_data = self.request_meta_data
            for object_name in request_meta_data["send"]:
                if object_name in ["body", "joint"]:
                    continue
                if object_name not in self.body_dict and any(
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
                if object_name in ["body", "joint"] or object_name in self.body_dict:
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

            if len(objects_to_destroy) > 0:
                self._remove_all_object_views()

            for object_name in objects_to_destroy:
                if object_name in self.body_dict:
                    self.loginfo(f"Destroying object {object_name}...")
                    object_prim = self.body_dict[object_name].root_prim
                    prims_utils.delete_prim(prim_path=object_prim.GetPath().pathString)
                    if object_name in send_objects:
                        del send_objects[object_name]
                    if object_name in receive_objects:
                        del receive_objects[object_name]
                    self.loginfo(f"Destroyed object {object_name}.")
                del request_meta_data["send"][object_name]
                del request_meta_data["receive"][object_name]

            if len(objects_to_spawn) > 0 or len(objects_to_destroy) > 0:
                if not world.is_playing():
                    world.play()
                    world.pause()

            self.request_meta_data = request_meta_data
            self.loginfo("Sending request meta data: " + str(self.request_meta_data))

        self.bind_request_meta_data_callback = bind_request_meta_data_callback

        def bind_response_meta_data_callback() -> None:
            response_meta_data = self.response_meta_data
            self.loginfo("Received response meta data: " + str(response_meta_data))

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
                    if body_name in self.constrained_bodies and body_name in free_body_names:
                        free_body_names.remove(body_name)
                bodies_positions[articulation_view_name], bodies_quaternions[articulation_view_name] = articulation_view.get_world_poses()
                bodies_velocities[articulation_view_name] = articulation_view.get_velocities()
                joints_states[articulation_view_name] = articulation_view.get_joints_state()
                measured_joints_efforts[articulation_view_name] = articulation_view.get_measured_joint_efforts()

            for send_receive in ["send", "receive"]:
                if send_receive not in response_meta_data:
                    continue
                for object_name, attributes in response_meta_data[send_receive].items():
                    if object_name in self.body_dict:
                        if object_name in self.object_xform_prim_view_idx_dict:
                            view_name, object_idx = self.object_xform_prim_view_idx_dict[object_name]
                        elif object_name in self.object_rigid_prim_view_idx_dict:
                            view_name, object_idx = self.object_rigid_prim_view_idx_dict[object_name]
                        elif object_name in self.object_articulation_view_idx_dict:
                            view_name, object_idx = self.object_articulation_view_idx_dict[object_name]
                        else:
                            raise Exception(f"Body {object_name} not found in any view")

                        for attribute in attributes:
                            data = response_meta_data[send_receive][object_name][attribute]
                            if any(x is None for x in data):
                                continue
                            if attribute == "position":
                                bodies_positions[view_name][object_idx] = [data[0], data[1], data[2]]
                            elif attribute == "quaternion":
                                bodies_quaternions[view_name][object_idx] = [data[0], data[1], data[2], data[3]]
                            elif attribute == "relative_velocity":
                                bodies_velocities[view_name][object_idx] = [data[0], data[1], data[2], data[3], data[4], data[5]]

            for rigid_prim_view_name, rigid_prim_view in self.scene_registry.rigid_prim_views.items():
                free_body_idxes = [idx for idx, prim in enumerate(rigid_prim_view.prims) if prim.GetName() in free_body_names]
                if len(free_body_idxes) == 0:
                    continue
                body_positions = bodies_positions[rigid_prim_view_name][free_body_idxes]
                body_quaternions = bodies_quaternions[rigid_prim_view_name][free_body_idxes]
                rigid_prim_view.set_world_poses(body_positions, body_quaternions, indices=free_body_idxes)

                body_velocities = bodies_velocities[rigid_prim_view_name][free_body_idxes]
                rigid_prim_view.set_velocities(body_velocities, indices=free_body_idxes)

            for articulation_view_name, articulation_view in self.scene_registry.articulated_views.items():
                articulation_view.set_world_poses(bodies_positions[articulation_view_name], bodies_quaternions[articulation_view_name])
                articulation_view.set_velocities(bodies_velocities[articulation_view_name])

            if not world.is_playing():
                world.play()
                world.pause()

        self.bind_response_meta_data_callback = bind_response_meta_data_callback

        def init_objects_callback() -> None:
            world.initialize_physics()
            request_meta_data = self.request_meta_data
            current_stage = stage.get_current_stage()
            articulation_prim_paths = []
            self.body_dict = {}
            self.object_xform_prim_view_idx_dict = {}
            self.object_rigid_prim_view_idx_dict = {}
            self.joint_dict = {}
            self.object_articulation_view_idx_dict = {}

            for prim in current_stage.TraverseAll():
                if prim.IsA(UsdGeom.Xform):
                    body_name = prim.GetName()
                    if body_name in self.ignore_names:
                        continue
                    if prim.GetParent().IsA(UsdGeom.Xform) and prim.GetParent().GetName() == body_name:
                        root_prim = prim.GetParent()
                    else:
                        root_prim = prim
                    self.body_dict[body_name] = MultiverseObject(prim, root_prim)
                if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                    if prim.IsA(UsdGeom.Xform):
                        xform_prim_path = prim.GetPath().pathString
                    elif prim.IsA(UsdPhysics.Joint):
                        joint = UsdPhysics.Joint(prim)
                        xform_prim_path = joint.GetBody1Rel().GetTargets()[0].pathString
                    articulation_prim_paths.append(xform_prim_path)

            self.constrained_bodies = set()
            for joint_prim in [prim for prim in current_stage.TraverseAll() if prim.IsA(UsdPhysics.Joint)]:
                joint = UsdPhysics.Joint(joint_prim)
                body0_targets = joint.GetBody0Rel().GetTargets()
                if len(body0_targets) != 0:
                    body0_name = body0_targets[0].name
                    self.constrained_bodies.add(body0_name)
                body1_targets = joint.GetBody1Rel().GetTargets()
                if len(body1_targets) != 0:
                    body1_name = body1_targets[0].name
                    self.constrained_bodies.add(body1_name)
                joint_name = joint_prim.GetName()
                if joint_name in self.ignore_names:
                    continue
                if joint_prim.IsA(UsdPhysics.RevoluteJoint) or joint_prim.IsA(UsdPhysics.PrismaticJoint):
                    self.joint_dict[joint_prim.GetName()] = joint_prim

            send_object_names = sorted(request_meta_data["send"].keys())
            for object_name in send_object_names:
                if object_name not in self.body_dict and object_name not in self.joint_dict:
                    del request_meta_data["send"][object_name]

            receive_object_names = sorted(request_meta_data["receive"].keys())
            for object_name in receive_object_names:
                if object_name not in self.body_dict and object_name not in self.joint_dict:
                    del request_meta_data["receive"][object_name]

            for object_name in sorted(self.receive_objects.keys()):
                request_meta_data["receive"][object_name] = []
                for attr in self.receive_objects[object_name]:
                    request_meta_data["receive"][object_name].append(attr)

            if "body" in self.send_objects:
                for body_name in sorted(self.body_dict.keys()):
                    request_meta_data["send"][body_name] = []
                    for attr in sorted(self.send_objects["body"]):
                        if body_name in request_meta_data["receive"] and attr in request_meta_data["receive"][body_name]:
                            continue
                        if attr == "relative_velocity" and body_name in self.constrained_bodies:
                            continue
                        request_meta_data["send"][body_name].append(attr)

            if "joint" in self.send_objects:
                for joint_name in sorted(self.joint_dict.keys()):
                    request_meta_data["send"][joint_name] = []
                    for attr in sorted(self.send_objects["joint"]):
                        if joint_name in request_meta_data["receive"] and attr in request_meta_data["receive"][joint_name]:
                            continue
                        joint_prim = self.joint_dict[joint_name]
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
                    for attr in sorted(self.send_objects[object_name]):
                        request_meta_data["send"][object_name].append(attr)

            body_paths = []

            for object_name in list(request_meta_data["send"].keys()) + list(request_meta_data["receive"].keys()):
                for articulation_prim_path in articulation_prim_paths:
                    articulation = dc.get_articulation(articulation_prim_path)
                    if object_name in self.joint_dict:
                        dof_ptr = dc.find_articulation_dof(articulation, object_name)
                        if dof_ptr == 0:
                            continue
                    elif object_name in self.body_dict:
                        prim = self.body_dict[object_name].prim
                        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                            break
                        rigid_body_api = UsdPhysics.RigidBodyAPI(prim)
                        if not rigid_body_api.GetRigidBodyEnabledAttr().Get():
                            break

                    articulation_view_name = f"{articulation_prim_path}_view"
                    if self.scene_registry.name_exists(articulation_view_name):
                        articulation_view = self.scene_registry.articulated_views[articulation_view_name]
                    else:
                        articulation_view = ArticulationView(prim_paths_expr=articulation_prim_path, name=articulation_view_name)
                        articulation_view.initialize(world.physics_sim_view)
                        self.scene_registry.add_articulated_view(articulation_view.name, articulation_view)
                    if object_name in self.joint_dict and object_name in articulation_view.dof_names:
                        joint_idx = articulation_view.get_dof_index(object_name)
                        self.object_articulation_view_idx_dict[object_name] = (articulation_view.name, joint_idx)
                    elif object_name in self.body_dict and object_name == Sdf.Path(articulation_prim_path).name:
                        body_idx = articulation_view.get_link_index(object_name)
                        self.object_articulation_view_idx_dict[object_name] = (articulation_view.name, body_idx)
                    if object_name in self.object_articulation_view_idx_dict:
                        break
                if object_name not in self.object_articulation_view_idx_dict and object_name in self.body_dict:
                    body_paths.append(self.body_dict[object_name].prim.GetPath())

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
                    xform_prim_view = XFormPrimView(prim_paths_expr=[body_path.pathString for body_path in xform_prim_paths])
                    xform_prim_view.initialize(world.physics_sim_view)
                    self.scene_registry.add_xform_view(xform_prim_view.name, xform_prim_view)
                    for body_idx, body_path in enumerate(xform_prim_paths):
                        self.object_xform_prim_view_idx_dict[body_path.name] = (xform_prim_view.name, body_idx)

                if len(rigid_prim_paths) > 0:
                    rigid_prim_view = RigidPrimView(prim_paths_expr=[body_path.pathString for body_path in rigid_prim_paths])
                    rigid_prim_view.initialize(world.physics_sim_view)
                    self.scene_registry.add_rigid_prim_view(rigid_prim_view.name, rigid_prim_view)
                    for body_idx, body_path in enumerate(rigid_prim_paths):
                        self.object_rigid_prim_view_idx_dict[body_path.name] = (rigid_prim_view.name, body_idx)

            self.request_meta_data = request_meta_data

        self.init_objects_callback = init_objects_callback

        def bind_send_data() -> None:
            send_data = [self.world_time + self.sim_time]
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

            for object_name, attributes in self.request_meta_data.get("send", {}).items():
                if object_name in self.body_dict:
                    if object_name in self.object_xform_prim_view_idx_dict:
                        view_name, object_idx = self.object_xform_prim_view_idx_dict[object_name]
                    elif object_name in self.object_rigid_prim_view_idx_dict:
                        view_name, object_idx = self.object_rigid_prim_view_idx_dict[object_name]
                    elif object_name in self.object_articulation_view_idx_dict:
                        view_name, object_idx = self.object_articulation_view_idx_dict[object_name]
                    else:
                        raise Exception(f"Body {object_name} not found in any view")

                    for attribute in attributes:
                        if attribute == "position":
                            position = bodies_positions[view_name][object_idx]
                            send_data += [position[0], position[1], position[2]]
                        elif attribute == "quaternion":
                            quaternion = bodies_quaternions[view_name][object_idx]
                            send_data += [quaternion[0], quaternion[1], quaternion[2], quaternion[3]]
                        elif attribute == "relative_velocity":
                            velocity = bodies_velocities[view_name][object_idx]
                            send_data += [velocity[0], velocity[1], velocity[2], velocity[3], velocity[4], velocity[5]]

                elif object_name in self.joint_dict:
                    if object_name in self.object_articulation_view_idx_dict:
                        articulation_view_name, joint_idx = self.object_articulation_view_idx_dict[object_name]
                    else:
                        raise Exception(f"Joint {object_name} not found in any view")
                    for attribute in attributes:
                        if attribute == "joint_rvalue":
                            send_data += [joints_states[articulation_view_name].positions[0][joint_idx]]
                        elif attribute == "joint_tvalue":
                            send_data += [joints_states[articulation_view_name].positions[0][joint_idx]]
                        elif attribute == "joint_linear_velocity":
                            send_data += [joints_states[articulation_view_name].velocities[0][joint_idx]]
                        elif attribute == "joint_angular_velocity":
                            send_data += [joints_states[articulation_view_name].velocities[0][joint_idx]]
                        elif attribute == "joint_force":
                            send_data += [measured_joints_efforts[articulation_view_name][0][joint_idx]]
                        elif attribute == "joint_torque":
                            send_data += [measured_joints_efforts[articulation_view_name][0][joint_idx]]

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
                if object_name in self.body_dict:
                    if object_name in self.object_xform_prim_view_idx_dict:
                        view_name, object_idx = self.object_xform_prim_view_idx_dict[object_name]
                    elif object_name in self.object_rigid_prim_view_idx_dict:
                        view_name, object_idx = self.object_rigid_prim_view_idx_dict[object_name]
                    elif object_name in self.object_articulation_view_idx_dict:
                        view_name, object_idx = self.object_articulation_view_idx_dict[object_name]
                    else:
                        raise Exception(f"Body {object_name} not found in any view")

                    for attribute in attributes:
                        if attribute == "odometric_velocity" and object_name in self.body_dict:
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

                elif object_name in self.joint_dict:
                    articulation_view_name, joint_idx = self.object_articulation_view_idx_dict[object_name]
                    for attribute in attributes:
                        if attribute == "cmd_joint_rvalue" or attribute == "cmd_joint_tvalue":
                            cmd_joint_value = receive_data[0]
                            receive_data = receive_data[1:]
                            cmd_joints_values[articulation_view_name][joint_idx] = cmd_joint_value

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
                    cmd_joint_values = list(cmd_joints_values[articulation_view_name].values())
                    cmd_joint_idxes = list(cmd_joints_values[articulation_view_name].keys())
                    action = ArticulationActions(joint_positions=cmd_joint_values, joint_indices=cmd_joint_idxes)
                    articulation_view.apply_action(action)
        self.bind_receive_data_callback = bind_receive_data

    def loginfo(self, message: str) -> None:
        omni.kit.app.get_app().print_and_log(f"INFO: {message}")

    def logwarn(self, message: str) -> None:
        omni.kit.app.get_app().print_and_log(f"WARN: {message}")

    def _run(self) -> None:
        self.loginfo("Start running the Isaac Sim Connector.")
        self._connect_and_start()

    def send_and_receive_meta_data(self) -> None:
        self._communicate(True)

    def send_and_receive_data(self) -> None:
        self._communicate(False)

    def _remove_all_object_views(self) -> None:
        for articulation_view_name, articulation_view in dict(self.scene_registry.articulated_views).items():
            articulation_view._physics_sim_view.invalidate()
            world.scene.remove_object(articulation_view_name)
        for xform_prim_view_name, _ in dict(self.scene_registry.xform_prim_views).items():
            world.scene.remove_object(xform_prim_view_name)
        for rigid_prim_view_name, rigid_prim_view in dict(self.scene_registry.rigid_prim_views).items():
            rigid_prim_view._physics_sim_view.invalidate()
            world.scene.remove_object(rigid_prim_view_name)

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
    import omni
    from omni.isaac.core import World
    from omni.isaac.core.scenes import Scene, SceneRegistry
    from omni.isaac.core.prims import XFormPrimView, RigidPrimView
    from omni.isaac.core.articulations import ArticulationView
    from omni.isaac.core.utils.types import ArticulationActions
    from omni.isaac.dynamic_control import _dynamic_control
    from omni.isaac.core.utils import stage
    import omni.isaac.core.utils.prims as prims_utils
    from omni.isaac.nucleus import is_file
    from pxr import UsdGeom, UsdPhysics, Sdf

    dc = _dynamic_control.acquire_dynamic_control_interface()

    world = World()

    # make sure the file exists before we try to open it
    try:
        result = is_file(usd_path)
    except:
        result = False

    if not result:
        carb.log_error(f"the usd path {usd_path} could not be opened, please make sure that {args.usd_path} is a valid usd file")
        simulation_app.close()
        sys.exit()

    print("Loading stage...")
    while stage.is_stage_loading():
        simulation_app.update()
    print("Loading Complete")

    world.play()

    isaac_sim_connector = IsaacSimConnector(
        client_addr=client_addr,
        multiverse_meta_data=multiverse_meta_data,
        send_objects=send_objects,
        receive_objects=receive_objects,
        resources=resources,
    )

    world.reset()

    isaac_sim_connector.run()

    while simulation_app.is_running():
        isaac_sim_connector.send_and_receive_data()

        # Run in realtime mode, we don't specify the step size
        world.step(render=True)

    world.stop()
    simulation_app.close()  # close Isaac Sim

    isaac_sim_connector.stop()
