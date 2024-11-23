from multiverse_client_py import MultiverseClient, MultiverseMetaData, SocketAddress

class MyConnector(MultiverseClient):
    def __init__(self, client_addr: SocketAddress, multiverse_meta_data: MultiverseMetaData) -> None:
        super().__init__(client_addr, multiverse_meta_data)

    def loginfo(self, message: str) -> None:
        print(f"INFO: {message}")

    def logwarn(self, message: str) -> None:
        print(f"WARN: {message}")

    def _run(self) -> None:
        self.loginfo("Start running the client.")
        self._connect_and_start()

    def send_and_receive_meta_data(self) -> None:
        self.loginfo("Sending request meta data: " + str(self.request_meta_data))
        self._communicate(True)
        self.loginfo("Received response meta data: " + str(self.response_meta_data))

    def send_and_receive_data(self) -> None:
        self._communicate(False)

import rospy
from std_msgs.msg import Float64
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from typing import Dict, Tuple, List

from enum import Enum

joint_state_inits = {
    "joint1": 0.0,
    "joint2": 0.0,
    "joint3": 0.0,
    "joint4": -1.57079,
    "joint5": 0.0,
    "joint6": 1.57079,
    "joint7": -0.7853,
}

class Stage(Enum):
    READY = 0
    RECORDING = 1
    LOADING = 2

gripper_publisher = rospy.Publisher('/gripper_command', Float64, queue_size=10)
def control_gripper(open: bool):
    gripper_publisher.publish(0.0 if open else 255.0)

class StartClients:
    joint_states: Dict[str, Tuple[float, float]] = {}
    joint_trajectory: JointTrajectory = JointTrajectory()
    gripper_state: float = 0.0
    gripper_states: List[Tuple[float, float]] = []
    stage = Stage.READY

    def __init__(self):
        multiverse_meta_data = MultiverseMetaData(
            world_name="world",
            simulation_name="demo",
            length_unit="m",
            angle_unit="rad",
            mass_unit="kg",
            time_unit="s",
            handedness="rhs",
        )
        self.multiverse_client = MyConnector(client_addr=SocketAddress(port="5000"),
                                   multiverse_meta_data=multiverse_meta_data)
        self.multiverse_client.run()
        self.multiverse_client.send_and_receive_meta_data()

        self.joint_trajectory_publisher = rospy.Publisher('/world/panda/joint_trajectory_velocity_controller/command', JointTrajectory, queue_size=10)
        rospy.init_node('record_play')
        rospy.Subscriber('/world/panda/joint_states', JointState, self.joint_state_callback)
        rospy.Subscriber('/gripper_command', Float64, self.gripper_state_callback)
        rospy.Service('reset', Trigger, self.reset_callback)
        rospy.Service('record', Trigger, self.handle_record)
        rospy.Service('save', Trigger, self.handle_save)
        rospy.Service('load', Trigger, self.handle_load)
        rospy.Service('stop', Trigger, self.handle_stop)
        rospy.loginfo("Ready to record.")

    def reset(self):
        tmp_joint_trajectory = JointTrajectory()
        tmp_joint_trajectory.joint_names = list(self.joint_states.keys())
        tmp_joint_trajectory.points = [JointTrajectoryPoint()]
        tmp_joint_trajectory.points[0].positions = [joint_state_inits[joint_name] for joint_name in tmp_joint_trajectory.joint_names]
        tmp_joint_trajectory.points[0].velocities = [0.0 for _ in tmp_joint_trajectory.joint_names]
        tmp_joint_trajectory.points[0].time_from_start = rospy.Duration(0.1)
        self.joint_trajectory_publisher.publish(tmp_joint_trajectory)
        self.multiverse_client.send_data = [0.0]
        self.multiverse_client.send_and_receive_data()

    def reset_callback(self, req):
        self.stage = Stage.READY
        self.reset()
        return TriggerResponse(success=True, message="Reset successful.")

    def handle_stop(self, req):
        self.stage = Stage.READY
        return TriggerResponse(success=True, message="Stopped successful.")

    def joint_state_callback(self, msg: JointState):
        for i, joint_name in enumerate(msg.name):
            self.joint_states[joint_name] = (msg.position[i], msg.velocity[i])

    def gripper_state_callback(self, msg: Float64):
        self.gripper_state = msg.data

    def handle_record(self, req):
        self.stage = Stage.RECORDING
        self.joint_trajectory.header.frame_id = "map"
        self.joint_trajectory.joint_names = list(self.joint_states.keys())
        self.joint_trajectory.points = []
        self.gripper_states = []
        time_start = rospy.Time.now()

        while self.stage == Stage.RECORDING and not rospy.is_shutdown():
            time_from_start = rospy.Time.now() - time_start
            self.joint_trajectory.header.seq += 1
            trajectory_point = JointTrajectoryPoint()
            for joint_name in self.joint_states.keys():
                trajectory_point.positions.append(self.joint_states[joint_name][0])
                trajectory_point.velocities.append(self.joint_states[joint_name][1])
            trajectory_point.time_from_start = time_from_start
            self.gripper_states.append((time_from_start, self.gripper_state))
            self.joint_trajectory.points.append(trajectory_point)

            tmp_joint_trajectory = JointTrajectory()
            tmp_joint_trajectory.header = self.joint_trajectory.header
            tmp_joint_trajectory.joint_names = self.joint_trajectory.joint_names
            tmp_joint_trajectory.points = [JointTrajectoryPoint()]
            tmp_joint_trajectory.points[0].positions = trajectory_point.positions
            tmp_joint_trajectory.points[0].velocities = trajectory_point.velocities
            tmp_joint_trajectory.points[0].time_from_start = rospy.Duration(0.1)
            self.joint_trajectory_publisher.publish(tmp_joint_trajectory)

            rospy.sleep(0.1)

        rospy.loginfo("Recording successful.")
        self.stage = Stage.READY
        return TriggerResponse(success=True, message="Recording successful.")

    def handle_save(self, req):
        if self.stage == Stage.RECORDING:
            self.stage = Stage.READY
            return TriggerResponse(success=True, message="Saved successful.")
        elif self.stage == Stage.LOADING:
            return TriggerResponse(success=False, message="Cannot save while loading.")
        elif self.stage == Stage.READY:
            return TriggerResponse(success=False, message="No trajectory to save.")

    def handle_load(self, req):
        if self.stage == Stage.RECORDING:
            return TriggerResponse(success=False, message="Cannot load while recording.")
        elif self.stage == Stage.LOADING:
            return TriggerResponse(success=False, message="Already loading.")

        if len(self.joint_trajectory.points) == 0:
            return TriggerResponse(success=False, message="No trajectory to load.")
        
        self.joint_trajectory.header.seq += 1
        self.joint_trajectory_publisher.publish(self.joint_trajectory)

        time_start = rospy.Time.now()

        self.stage = Stage.LOADING
        while self.stage == Stage.LOADING and not rospy.is_shutdown():
            rospy.sleep(0.1)
            time_from_start = rospy.Time.now() - time_start
            for time, gripper_state in self.gripper_states.copy():
                if time < time_from_start:
                    gripper_publisher.publish(gripper_state)
                    self.gripper_states.remove((time, gripper_state))
            if self.joint_trajectory.points[-1].time_from_start < time_from_start:
                break

        return TriggerResponse(success=True, message="Loaded successful.")

import threading

class ButtonHandle:
    def __init__(self) -> None:
        multiverse_meta_data = MultiverseMetaData(
            world_name="world",
            simulation_name="button_handle",
            length_unit="m",
            angle_unit="rad",
            mass_unit="kg",
            time_unit="s",
            handedness="rhs",
        )
        self.multiverse_client = MyConnector(client_addr=SocketAddress(port="5001"),
                                             multiverse_meta_data=multiverse_meta_data)
        self.multiverse_client.run()
        self.multiverse_client.request_meta_data["receive"] = {}
        self.multiverse_client.request_meta_data["receive"]["gripper_button_joint"] = ["joint_tvalue"]
        self.multiverse_client.request_meta_data["receive"]["load_button_joint"] = ["joint_tvalue"]
        self.multiverse_client.request_meta_data["receive"]["record_button_joint"] = ["joint_tvalue"]
        self.multiverse_client.request_meta_data["receive"]["reset_button_joint"] = ["joint_tvalue"]
        self.multiverse_client.send_and_receive_meta_data()

        is_gripper_button_pressed = False
        is_reset_button_pressed = False
        is_record_button_pressed = False
        is_load_button_pressed = False

        self.stage = Stage.READY

        self.record_thread = threading.Thread(target=self.record)
        self.save_thread = threading.Thread(target=self.save)
        self.load_thread = threading.Thread(target=self.load)

        gripper_is_open = True

        while not rospy.is_shutdown():
            self.multiverse_client.send_data = [self.multiverse_client.world_time + self.multiverse_client.sim_time]
            self.multiverse_client.send_and_receive_data()            
            
            gripper_button_pressed = self.multiverse_client.receive_data[1] < -0.05
            load_button_pressed = self.multiverse_client.receive_data[2] < -0.05
            record_button_pressed = self.multiverse_client.receive_data[3] < -0.05
            reset_button_pressed = self.multiverse_client.receive_data[4] < -0.05         
            
            gripper_button_released = self.multiverse_client.receive_data[1] > -0.01
            load_button_released = self.multiverse_client.receive_data[2] > -0.01
            record_button_released = self.multiverse_client.receive_data[3] > -0.01
            reset_button_released = self.multiverse_client.receive_data[4] > -0.01

            if gripper_button_pressed and not is_gripper_button_pressed:
                is_gripper_button_pressed = True
                rospy.loginfo("Gripper button pressed.")
                gripper_is_open = not gripper_is_open
                control_gripper(gripper_is_open)

            if gripper_button_released and is_gripper_button_pressed:
                is_gripper_button_pressed = False
                rospy.loginfo("Gripper button released.")

            if reset_button_pressed and not is_reset_button_pressed and self.stage != Stage.RECORDING:
                is_reset_button_pressed = True
                rospy.loginfo("Reset button pressed.")
                self.reset()
                rospy.sleep(1)
                if self.stage == Stage.LOADING:
                    self.load_thread.join()
                self.stage = Stage.READY
                
            if reset_button_released and is_reset_button_pressed:
                is_reset_button_pressed = False
                rospy.loginfo("Reset button released.")

            if record_button_pressed and not is_record_button_pressed:
                if self.stage == Stage.LOADING:
                    self.stop()
                    self.load_thread.join()
                    self.stage = Stage.READY

                if self.stage == Stage.READY:
                    is_record_button_pressed = True
                    rospy.loginfo("Record button pressed.")
                    self.stage = Stage.RECORDING
                    self.record_thread = threading.Thread(target=self.record)
                    self.record_thread.start()

                elif self.stage == Stage.RECORDING:
                    is_record_button_pressed = True
                    rospy.loginfo("Save button pressed.")
                    self.stage = Stage.READY
                    self.save_thread = threading.Thread(target=self.save)
                    self.save_thread.start()
                    self.record_thread.join()
                    self.save_thread.join()

            if record_button_released and is_record_button_pressed:
                is_record_button_pressed = False
                rospy.loginfo("Record/Save button released.")

            if load_button_pressed and not is_load_button_pressed:
                is_load_button_pressed = True
                rospy.loginfo("Load button pressed.")
                if self.stage == Stage.READY:
                    self.load_thread = threading.Thread(target=self.load)
                    self.load_thread.start()
                    self.stage = Stage.LOADING
                elif self.stage == Stage.LOADING:
                    self.stop()
                    self.load_thread.join()
                    self.load_thread = threading.Thread(target=self.load)
                    self.load_thread.start()

            if load_button_released and is_load_button_pressed:
                is_load_button_pressed = False
                rospy.loginfo("Load button released.")
            rospy.sleep(0.1)

    def reset(self):
        rospy.wait_for_service('reset')
        try:
            reset = rospy.ServiceProxy('reset', Trigger)
            reset()
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def record(self):
        rospy.wait_for_service('record')
        try:
            record = rospy.ServiceProxy('record', Trigger)
            record()
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def save(self):
        rospy.wait_for_service('save')
        try:
            save = rospy.ServiceProxy('save', Trigger)
            save()
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def load(self):
        rospy.wait_for_service('load')
        try:
            load = rospy.ServiceProxy('load', Trigger)
            load()
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def stop(self):
        rospy.wait_for_service('stop')
        try:
            stop = rospy.ServiceProxy('stop', Trigger)
            stop()
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    StartClients()
    ButtonHandle()
    rospy.spin()
