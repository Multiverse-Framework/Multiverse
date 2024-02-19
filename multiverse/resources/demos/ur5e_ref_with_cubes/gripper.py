import rospy
from std_msgs.msg import Float64
import time

rospy.init_node('gripper_publisher')

pub = rospy.Publisher('/gripper_command', Float64, queue_size=10)

rate = rospy.Rate(0.2)  # 0.2 Hz (every 5 seconds)

close = True

while not rospy.is_shutdown():
    command = Float64()
    # Set the value of the command here
    command.data = close * 255

    pub.publish(command)
    close = not close
    rate.sleep()
