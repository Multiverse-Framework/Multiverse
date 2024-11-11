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

from geometry_msgs.msg import PoseStamped
from giskardpy.python_interface.old_python_interface import OldGiskardWrapper as GiskardWrapper
from time import sleep

from enum import Enum

class Stage(Enum):
    READY = 0
    RECORDING = 1
    LOADING = 2
    RUNNING_DEMO = 3

class Demo:
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

        self.joint_trajectory_publisher = rospy.Publisher('/world/ur5e/joint_trajectory_controller/command', JointTrajectory, queue_size=10)
        self.gripper_publisher = rospy.Publisher('/gripper_command', Float64, queue_size=10)
        rospy.init_node('record_play')
        rospy.Subscriber('/world/ur5e/joint_states', JointState, self.joint_state_callback)
        rospy.Subscriber('/gripper_command', Float64, self.gripper_state_callback)
        rospy.Service('reset', Trigger, self.reset_callback)
        rospy.Service('record', Trigger, self.handle_record)
        rospy.Service('save', Trigger, self.handle_save)
        rospy.Service('load', Trigger, self.handle_load)
        rospy.Service('run_demo', Trigger, self.handle_run_demo)
        rospy.loginfo("Ready to record.")

    def reset(self):
        self.multiverse_client.send_data = [0.0]
        self.multiverse_client.send_and_receive_data()

    def reset_callback(self, req):
        self.stage = Stage.READY
        self.reset()
        return TriggerResponse(success=True, message="Reset successful.")

    def joint_state_callback(self, msg: JointState):
        for i, joint_name in enumerate(msg.name):
            self.joint_states[joint_name] = (msg.position[i], msg.velocity[i])

    def gripper_state_callback(self, msg: Float64):
        self.gripper_state = msg.data

    def handle_record(self, req):
        self.reset()
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
        elif self.stage == Stage.RUNNING_DEMO:
            self.stage = Stage.READY

        if len(self.joint_trajectory.points) == 0:
            return TriggerResponse(success=False, message="No trajectory to load.")
        
        self.joint_trajectory.header.seq += 1
        self.joint_trajectory_publisher.publish(self.joint_trajectory)

        time_start = rospy.Time.now()

        self.reset()
        self.stage = Stage.LOADING
        while self.stage == Stage.LOADING and not rospy.is_shutdown():
            rospy.sleep(0.1)
            time_from_start = rospy.Time.now() - time_start
            for time, gripper_state in self.gripper_states.copy():
                if time < time_from_start:
                    self.gripper_publisher.publish(gripper_state)
                    self.gripper_states.remove((time, gripper_state))
            if self.joint_trajectory.points[-1].time_from_start < time_from_start:
                break

        return TriggerResponse(success=True, message="Loaded successful.")

    def handle_run_demo(self, req):
        if self.stage == Stage.RECORDING:
            return TriggerResponse(success=False, message="Cannot run demo while recording.")

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'base_link'
        goal_pose.pose.orientation.x = 0.5
        goal_pose.pose.orientation.y = 0.5
        goal_pose.pose.orientation.z = -0.5
        goal_pose.pose.orientation.w = -0.5

        giskard = GiskardWrapper()

        self.stage = Stage.RUNNING_DEMO
        while self.stage == Stage.RUNNING_DEMO and not rospy.is_shutdown():
            self.reset()

            rospy.loginfo('Reset the robot')
            goal_pose.pose.position.x = -0.500
            goal_pose.pose.position.y = 0.134
            goal_pose.pose.position.z = 0.567
            giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
            giskard.execute()

            if self.stage != Stage.RUNNING_DEMO:
                break

            rospy.loginfo('Prepare to pick the tool')
            goal_pose.pose.position.x = -0.71
            goal_pose.pose.position.y = 0.102
            goal_pose.pose.position.z = 0.41
            giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
            giskard.execute()

            if self.stage != Stage.RUNNING_DEMO:
                break

            rospy.loginfo('Open the gripper')
            self.gripper_publisher.publish(0.0)

            if self.stage != Stage.RUNNING_DEMO:
                break

            rospy.loginfo('Prepare to pick the tool')
            goal_pose.pose.position.x = -0.71
            goal_pose.pose.position.y = 0.102
            goal_pose.pose.position.z = 0.36
            giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
            giskard.execute()

            if self.stage != Stage.RUNNING_DEMO:
                break

            rospy.loginfo('Close the gripper')
            self.gripper_publisher.publish(255.0)

            if self.stage != Stage.RUNNING_DEMO:
                break

            sleep(1)

            rospy.loginfo('Pick the tool')
            goal_pose.pose.position.x = -0.65
            goal_pose.pose.position.y = 0.102
            goal_pose.pose.position.z = 0.365
            giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
            giskard.execute()

            if self.stage != Stage.RUNNING_DEMO:
                break

            rospy.loginfo('Put the tool')
            goal_pose.pose.position.x = -0.71
            goal_pose.pose.position.y = 0.102
            goal_pose.pose.position.z = 0.375
            giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
            giskard.execute()

            if self.stage != Stage.RUNNING_DEMO:
                break

            sleep(1)

            rospy.loginfo('Open the gripper')
            self.gripper_publisher.publish(0.0)

            if self.stage != Stage.RUNNING_DEMO:
                break
            
            rospy.loginfo('Prepare to press the button 1')
            goal_pose.pose.position.x = -0.652
            goal_pose.pose.position.y = -0.096
            goal_pose.pose.position.z = 0.45
            giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
            giskard.execute()

            if self.stage != Stage.RUNNING_DEMO:
                break

            rospy.loginfo('Close the gripper')
            self.gripper_publisher.publish(255.0)

            if self.stage != Stage.RUNNING_DEMO:
                break

            rospy.loginfo('Press the button 1')
            goal_pose.pose.position.x = -0.652
            goal_pose.pose.position.y = -0.096
            goal_pose.pose.position.z = 0.355
            giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
            giskard.execute()

            if self.stage != Stage.RUNNING_DEMO:
                break
            
            rospy.loginfo('Prepare to press the button 2')
            goal_pose.pose.position.x = -0.685
            goal_pose.pose.position.y = -0.096
            goal_pose.pose.position.z = 0.45
            giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
            giskard.execute()

            if self.stage != Stage.RUNNING_DEMO:
                break

            rospy.loginfo('Press the button 2')
            goal_pose.pose.position.x = -0.685
            goal_pose.pose.position.y = -0.096
            goal_pose.pose.position.z = 0.355
            giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
            giskard.execute()

            if self.stage != Stage.RUNNING_DEMO:
                break

            rospy.loginfo('Prepare to open the lid')
            goal_pose.pose.position.x = -0.64
            goal_pose.pose.position.y = 0.047
            goal_pose.pose.position.z = 0.45
            giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
            giskard.execute()

            if self.stage != Stage.RUNNING_DEMO:
                break

            rospy.loginfo('Prepare to open the lid')
            goal_pose.pose.position.x = -0.64
            goal_pose.pose.position.y = 0.047
            goal_pose.pose.position.z = 0.35
            giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
            giskard.execute()

            if self.stage != Stage.RUNNING_DEMO:
                break

            rospy.loginfo('Open the lid')
            goal_pose.pose.position.x = -0.73
            goal_pose.pose.position.y = 0.047
            goal_pose.pose.position.z = 0.44
            giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
            giskard.execute()

            if self.stage != Stage.RUNNING_DEMO:
                break

            rospy.loginfo('Prepare to move the slider')
            goal_pose.pose.position.x = -0.75
            goal_pose.pose.position.y = -0.062
            goal_pose.pose.position.z = 0.44
            giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
            giskard.execute()

            if self.stage != Stage.RUNNING_DEMO:
                break

            rospy.loginfo('Prepare to move the slider')
            goal_pose.pose.position.x = -0.77
            goal_pose.pose.position.y = -0.062
            goal_pose.pose.position.z = 0.355
            giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
            giskard.execute()

            if self.stage != Stage.RUNNING_DEMO:
                break

            rospy.loginfo('Move the slider')
            goal_pose.pose.position.x = -0.73
            goal_pose.pose.position.y = -0.062
            goal_pose.pose.position.z = 0.355
            giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
            giskard.execute()

            if self.stage != Stage.RUNNING_DEMO:
                break

            rospy.loginfo('Prepare to press the button 1')
            goal_pose.pose.position.x = -0.656
            goal_pose.pose.position.y = -0.096
            goal_pose.pose.position.z = 0.45
            giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
            giskard.execute()

        return TriggerResponse(success=True, message="Demo finished.")
    
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
        self.multiverse_client.request_meta_data["receive"]["demo_button_joint"] = ["joint_tvalue"]
        self.multiverse_client.request_meta_data["receive"]["gripper_button_joint"] = ["joint_tvalue"]
        self.multiverse_client.request_meta_data["receive"]["load_button_joint"] = ["joint_tvalue"]
        self.multiverse_client.request_meta_data["receive"]["record_button_joint"] = ["joint_tvalue"]
        self.multiverse_client.request_meta_data["receive"]["reset_button_joint"] = ["joint_tvalue"]
        self.multiverse_client.send_and_receive_meta_data()

        is_demo_button_pressed = False
        is_gripper_button_pressed = False
        is_reset_button_pressed = False
        is_record_button_pressed = False
        is_load_button_pressed = False

        while not rospy.is_shutdown():
            self.multiverse_client.send_data = [self.multiverse_client.world_time + self.multiverse_client.sim_time]
            self.multiverse_client.send_and_receive_data()            
            
            demo_button_pressed = self.multiverse_client.receive_data[1] < -0.03
            gripper_button_pressed = self.multiverse_client.receive_data[2] < -0.03
            load_button_pressed = self.multiverse_client.receive_data[3] < -0.03
            record_button_pressed = self.multiverse_client.receive_data[4] < -0.03
            reset_button_pressed = self.multiverse_client.receive_data[5] < -0.03           
            
            demo_button_released = self.multiverse_client.receive_data[1] > -0.025
            gripper_button_released = self.multiverse_client.receive_data[2] > -0.025
            load_button_released = self.multiverse_client.receive_data[3] > -0.025
            record_button_released = self.multiverse_client.receive_data[4] > -0.025
            reset_button_released = self.multiverse_client.receive_data[5] > -0.025

            if demo_button_pressed and not is_demo_button_pressed:
                is_demo_button_pressed = True
                rospy.loginfo("Demo button pressed.")
                rospy.wait_for_service('run_demo')
                try:
                    run_demo = rospy.ServiceProxy('run_demo', Trigger)
                    run_demo()
                except rospy.ServiceException as e:
                    rospy.logerr(f"Service call failed: {e}")
            if demo_button_released and is_demo_button_pressed:
                is_demo_button_pressed = False
                rospy.loginfo("Demo button released.")
            if gripper_button_pressed and not is_gripper_button_pressed:
                is_gripper_button_pressed = True
                rospy.loginfo("Gripper button pressed.")
            if gripper_button_released and is_gripper_button_pressed:
                is_gripper_button_pressed = False
                rospy.loginfo("Gripper button released.")
            if reset_button_pressed and not is_reset_button_pressed:
                is_reset_button_pressed = True
                rospy.loginfo("Reset button pressed.")
                rospy.wait_for_service('reset')
                try:
                    reset = rospy.ServiceProxy('reset', Trigger)
                    reset()
                except rospy.ServiceException as e:
                    rospy.logerr(f"Service call failed: {e}")
                rospy.sleep(1.0)
            if reset_button_released and is_reset_button_pressed:
                is_reset_button_pressed = False
                rospy.loginfo("Reset button released.")
            if record_button_pressed and not is_record_button_pressed:
                is_record_button_pressed = True
                rospy.loginfo("Record button pressed.")
            if record_button_released and is_record_button_pressed:
                is_record_button_pressed = False
                rospy.loginfo("Record button released.")
            if load_button_pressed and not is_load_button_pressed:
                is_load_button_pressed = True
                rospy.loginfo("Load button pressed.")
            if load_button_released and is_load_button_pressed:
                is_load_button_pressed = False
                rospy.loginfo("Load button released.")
            rospy.sleep(0.1)

if __name__ == "__main__":
    Demo()
    ButtonHandle()
    rospy.spin()