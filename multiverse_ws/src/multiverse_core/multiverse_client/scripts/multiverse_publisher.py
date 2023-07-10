#!/usr/bin/env python3


import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Transform
import sys
import threading
import os
sys.path.append(os.path.dirname(os.path.dirname(sys.argv[0])))
from multiverse_socket import MultiverseSocket  # noqa

attribute_map = {
    "": [],
    "position":  [0.0, 0.0, 0.0],
    "quaternion":  [1.0, 0.0, 0.0, 0.0],
    "relative_velocity":  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "joint_rvalue":  [0.0],
    "joint_tvalue":  [0.0],
    "joint_position":  [0.0, 0.0, 0.0],
    "joint_quaternion":  [1.0, 0.0, 0.0, 0.0],
    "force":  [0.0, 0.0, 0.0],
    "torque":  [0.0, 0.0, 0.0]
}


def set_meta_data_json() -> dict:
    meta_data_json = {}
    meta_data_json["world"] = rospy.get_param(
        '~multiverse/world') if rospy.has_param('~multiverse/world') else "world"
    meta_data_json["length_unit"] = rospy.get_param(
        "~multiverse/length_unit") if rospy.has_param("~multiverse/length_unit") else "m"
    meta_data_json["angle_unit"] = rospy.get_param(
        "~multiverse/angle_unit") if rospy.has_param("~multiverse/angle_unit") else "rad"
    meta_data_json["force_unit"] = rospy.get_param(
        "~multiverse/force_unit") if rospy.has_param("~multiverse/force_unit") else "N"
    meta_data_json["time_unit"] = rospy.get_param(
        "~multiverse/time_unit") if rospy.has_param("~multiverse/time_unit") else "s"
    meta_data_json["handedness"] = rospy.get_param(
        "~multiverse/handedness") if rospy.has_param("~multiverse/handedness") else "rhs"
    meta_data_json["send"] = {}
    meta_data_json["receive"] = {}

    return meta_data_json


def start_publish_tf():
    multiverse_socket = MultiverseSocket()

    meta_data_json = set_meta_data_json()
    meta_data_json["receive"][""] = ["position", "quaternion"]

    host = rospy.get_param(
        '~multiverse/host') if rospy.has_param('~multiverse/host') else "tcp://127.0.0.1"
    port = int(rospy.get_param(
        '~multiverse/publish/tf/port')) if rospy.has_param('~multiverse/publish/tf/port') else 7300
    rate = int(rospy.get_param(
        '~multiverse/publish/tf/rate')) if rospy.has_param('~multiverse/publish/tf/rate') else 60

    multiverse_socket.init(host, port)
    multiverse_socket.set_meta_data(meta_data_json)
    multiverse_socket.connect()
    meta_data_response = multiverse_socket.get_meta_data_response()

    object_name: str
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    tf_msgs = []
    root_frame_id = rospy.get_param(
        '~root_frame_id') if rospy.has_param('~root_frame_id') else "map"
    object_names = meta_data_response["receive"].keys()
    for object_name in object_names:
        tf_msg = TransformStamped()
        tf_msg.header.frame_id = root_frame_id
        tf_msg.child_frame_id = object_name
        tf_msgs.append(tf_msg)

    rate = rospy.Rate(rate)

    while not rospy.is_shutdown():
        multiverse_socket.set_send_data([rospy.Time.now().to_sec()])
        multiverse_socket.communicate()
        data = multiverse_socket.get_receive_data()
        # print(data)

        for i, object_name in enumerate(object_names):
            tf_msgs[i].header.stamp = rospy.Time.from_sec(data[0])
            tf_msgs[i].transform.translation.x = data[7 * i + 1]
            tf_msgs[i].transform.translation.y = data[7 * i + 2]
            tf_msgs[i].transform.translation.z = data[7 * i + 3]
            tf_msgs[i].transform.rotation.w = data[7 * i + 4]
            tf_msgs[i].transform.rotation.x = data[7 * i + 5]
            tf_msgs[i].transform.rotation.y = data[7 * i + 6]
            tf_msgs[i].transform.rotation.z = data[7 * i + 7]

        tf_broadcaster.sendTransform(tf_msgs)
        rate.sleep()

    multiverse_socket.disconnect()


def start_multiverse_publisher() -> None:
    rospy.init_node('multiverse_publisher')
    if not rospy.has_param('~multiverse/publish/tf'):
        return

    thread = threading.Thread(target=start_publish_tf)
    thread.start()
    thread.join()


if __name__ == "__main__":
    start_multiverse_publisher()
