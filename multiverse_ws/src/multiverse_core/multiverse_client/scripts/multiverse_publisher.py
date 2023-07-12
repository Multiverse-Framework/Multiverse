#!/usr/bin/env python3

from multiverse_client.utils.multiverse_utils import set_send_meta_data_json
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Transform
import sys
import threading
import os
sys.path.append(os.path.dirname(os.path.dirname(sys.argv[0])))
from multiverse_socket import MultiverseSocket  # noqa


def start_publish_tf():
    multiverse_socket = MultiverseSocket()

    send_meta_data_dict = set_send_meta_data_json()
    send_meta_data_dict["receive"][""] = ["position", "quaternion"]

    host = rospy.get_param(
        '~multiverse/host') if rospy.has_param('~multiverse/host') else "tcp://127.0.0.1"
    port = int(rospy.get_param(
        '~multiverse/publish/tf/port')) if rospy.has_param('~multiverse/publish/tf/port') else 7300
    rate = int(rospy.get_param(
        '~multiverse/publish/tf/rate')) if rospy.has_param('~multiverse/publish/tf/rate') else 60

    multiverse_socket.init(host, port)
    multiverse_socket.set_send_meta_data(send_meta_data_dict)
    multiverse_socket.connect()
    receive_meta_data_dict = multiverse_socket.get_receive_meta_data()

    object_name: str
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    tf_msgs = []
    root_frame_id = rospy.get_param(
        '~root_frame_id') if rospy.has_param('~root_frame_id') else "map"
    object_names = receive_meta_data_dict["receive"].keys()
    for object_name in object_names:
        tf_msg = TransformStamped()
        tf_msg.header.frame_id = root_frame_id
        tf_msg.child_frame_id = object_name
        tf_msgs.append(tf_msg)

    rate = rospy.Rate(rate)

    while not rospy.is_shutdown():
        multiverse_socket.set_send_data([rospy.Time.now().to_sec()])
        multiverse_socket.communicate()
        receive_data = multiverse_socket.get_receive_data()

        for i, object_name in enumerate(object_names):
            tf_msgs[i].header.stamp = rospy.Time.from_sec(receive_data[0])
            tf_msgs[i].transform.translation.x = receive_data[7 * i + 1]
            tf_msgs[i].transform.translation.y = receive_data[7 * i + 2]
            tf_msgs[i].transform.translation.z = receive_data[7 * i + 3]
            tf_msgs[i].transform.rotation.w = receive_data[7 * i + 4]
            tf_msgs[i].transform.rotation.x = receive_data[7 * i + 5]
            tf_msgs[i].transform.rotation.y = receive_data[7 * i + 6]
            tf_msgs[i].transform.rotation.z = receive_data[7 * i + 7]

        tf_broadcaster.sendTransform(tf_msgs)
        rate.sleep()

    multiverse_socket.disconnect()


def start_multiverse_publisher() -> None:
    rospy.init_node('multiverse_publisher')
    if not rospy.has_param('~multiverse/publish/tf'):
        return
    
    threads = []
    if rospy.has_param('~multiverse/publish/tf'):
        thread = threading.Thread(target=start_publish_tf)
        thread.start()
        threads.append(thread)
        
    rospy.spin()

    for thread in threads:
        thread.join()

if __name__ == "__main__":
    start_multiverse_publisher()
