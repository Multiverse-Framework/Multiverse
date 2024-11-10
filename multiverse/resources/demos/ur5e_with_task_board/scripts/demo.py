import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from giskardpy.python_interface.old_python_interface import OldGiskardWrapper as GiskardWrapper
from time import sleep
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
        self.loginfo("Sending data: " + str(self.send_data))
        self._communicate(False)
        self.loginfo("Received data: " + str(self.receive_data))

if __name__ == '__main__':
    multiverse_meta_data = MultiverseMetaData(
        world_name="world",
        simulation_name="demo",
        length_unit="m",
        angle_unit="rad",
        mass_unit="kg",
        time_unit="s",
        handedness="rhs",
    )
    client_addr = SocketAddress(port="5000")
    my_connector = MyConnector(client_addr=client_addr,
                               multiverse_meta_data=multiverse_meta_data)
    
    my_connector.run()
    my_connector.send_and_receive_meta_data()

    publisher = rospy.Publisher('/gripper_command', Float64, queue_size=10)

    rospy.init_node('demo')

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'base_link'
    goal_pose.pose.orientation.x = 0.5
    goal_pose.pose.orientation.y = 0.5
    goal_pose.pose.orientation.z = -0.5
    goal_pose.pose.orientation.w = -0.5

    giskard = GiskardWrapper()

    while not rospy.is_shutdown():
        my_connector.send_data = [0.0]
        my_connector.send_and_receive_data()

        rospy.loginfo('Reset the robot')
        goal_pose.pose.position.x = -0.500
        goal_pose.pose.position.y = 0.134
        goal_pose.pose.position.z = 0.567
        giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
        giskard.execute()

        rospy.loginfo('Prepare to pick the tool')
        goal_pose.pose.position.x = -0.72
        goal_pose.pose.position.y = 0.102
        goal_pose.pose.position.z = 0.41
        giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
        giskard.execute()

        rospy.loginfo('Open the gripper')
        publisher.publish(0.0)

        rospy.loginfo('Prepare to pick the tool')
        goal_pose.pose.position.x = -0.72
        goal_pose.pose.position.y = 0.102
        goal_pose.pose.position.z = 0.37
        giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
        giskard.execute()

        rospy.loginfo('Close the gripper')
        publisher.publish(255.0)

        sleep(1)

        rospy.loginfo('Pick the tool')
        goal_pose.pose.position.x = -0.65
        goal_pose.pose.position.y = 0.102
        goal_pose.pose.position.z = 0.385
        giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
        giskard.execute()

        rospy.loginfo('Put the tool')
        goal_pose.pose.position.x = -0.72
        goal_pose.pose.position.y = 0.102
        goal_pose.pose.position.z = 0.385
        giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
        giskard.execute()

        sleep(1)

        rospy.loginfo('Open the gripper')
        publisher.publish(0.0)
        
        rospy.loginfo('Prepare to press the button 1')
        goal_pose.pose.position.x = -0.656
        goal_pose.pose.position.y = -0.096
        goal_pose.pose.position.z = 0.45
        giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
        giskard.execute()

        rospy.loginfo('Close the gripper')
        publisher.publish(255.0)

        rospy.loginfo('Press the button 1')
        goal_pose.pose.position.x = -0.656
        goal_pose.pose.position.y = -0.096
        goal_pose.pose.position.z = 0.37
        giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
        giskard.execute()
        
        rospy.loginfo('Prepare to press the button 2')
        goal_pose.pose.position.x = -0.69
        goal_pose.pose.position.y = -0.096
        goal_pose.pose.position.z = 0.45
        giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
        giskard.execute()

        rospy.loginfo('Press the button 2')
        goal_pose.pose.position.x = -0.69
        goal_pose.pose.position.y = -0.096
        goal_pose.pose.position.z = 0.37
        giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
        giskard.execute()

        rospy.loginfo('Prepare to open the lid')
        goal_pose.pose.position.x = -0.64
        goal_pose.pose.position.y = 0.047
        goal_pose.pose.position.z = 0.45
        giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
        giskard.execute()

        rospy.loginfo('Prepare to open the lid')
        goal_pose.pose.position.x = -0.64
        goal_pose.pose.position.y = 0.047
        goal_pose.pose.position.z = 0.36
        giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
        giskard.execute()

        rospy.loginfo('Open the lid')
        goal_pose.pose.position.x = -0.73
        goal_pose.pose.position.y = 0.047
        goal_pose.pose.position.z = 0.44
        giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
        giskard.execute()

        rospy.loginfo('Prepare to move the slider')
        goal_pose.pose.position.x = -0.75
        goal_pose.pose.position.y = -0.062
        goal_pose.pose.position.z = 0.44
        giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
        giskard.execute()

        rospy.loginfo('Prepare to move the slider')
        goal_pose.pose.position.x = -0.77
        goal_pose.pose.position.y = -0.062
        goal_pose.pose.position.z = 0.38
        giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
        giskard.execute()

        rospy.loginfo('Move the slider')
        goal_pose.pose.position.x = -0.73
        goal_pose.pose.position.y = -0.062
        goal_pose.pose.position.z = 0.38
        giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
        giskard.execute()

        rospy.loginfo('Prepare to press the button 1')
        goal_pose.pose.position.x = -0.656
        goal_pose.pose.position.y = -0.096
        goal_pose.pose.position.z = 0.45
        giskard.set_cart_goal(root_link='base_link', tip_link='wrist_3_link', goal_pose=goal_pose)
        giskard.execute()