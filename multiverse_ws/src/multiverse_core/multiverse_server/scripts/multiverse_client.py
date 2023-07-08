#!/usr/bin/env python3

import zmq
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Transform
import sys
from json import dumps
from struct import unpack, pack
import threading
from multiverse_msgs.srv import QueryData, QueryDataRequest, QueryDataResponse
from multiverse_msgs.msg import ObjectAttribute, ObjectData

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

host = "tcp://127.0.0.1"
port = "7300"

context = zmq.Context()


class MultiverseService:
    def __init__(self) -> None:
        self.host = host
        self.port = port
        self.socket_addr = self.host + ":" + self.port

    def send_meta_data(self, meta_data_json) -> dict:
        self.socket_client = context.socket(zmq.REQ)
        rospy.loginfo(f"Open the socket connection on {self.socket_addr}")
        self.socket_client.connect(self.socket_addr)

        # Send JSON string over ZMQ
        self.socket_client.send_string(dumps(meta_data_json), flags=0)

        # Receive buffer sizes over ZMQ
        try:
            receive_json = self.socket_client.recv_json()
        except zmq.error.ContextTerminated:
            receive_json = {}
        return receive_json

    def communicate(self) -> list:
        time_in_sec = rospy.Time().now().to_sec()
        self.socket_client.send(bytearray(pack('d', time_in_sec)), flags=0)

        try:
            receive_message = self.socket_client.recv()
            data = list(
                unpack('d' * (len(receive_message) // 8), receive_message))
        except zmq.error.ContextTerminated:
            data = []
        return data

    def deinit(self) -> None:
        rospy.loginfo(f"Closing the socket client on {self.socket_addr}")
        self.socket_client.send_string("{}")
        self.socket_client.disconnect(self.socket_addr)


def set_meta_data_json() -> dict:
    meta_data_json = {}
    meta_data_json["length_unit"] = rospy.get_param(
        "~length_unit") if rospy.has_param("~length_unit") else "m"
    meta_data_json["angle_unit"] = rospy.get_param(
        "~angle_unit") if rospy.has_param("~angle_unit") else "rad"
    meta_data_json["force_unit"] = rospy.get_param(
        "~force_unit") if rospy.has_param("~force_unit") else "N"
    meta_data_json["time_unit"] = rospy.get_param(
        "~time_unit") if rospy.has_param("~time_unit") else "s"
    meta_data_json["handedness"] = rospy.get_param(
        "~handedness") if rospy.has_param("~handedness") else "rhs"
    meta_data_json["receive"] = {}
    return meta_data_json


def start_publish_tf():
    meta_data_json = set_meta_data_json()
    meta_data_json["receive"][""] = ["position", "quaternion"]

    multiverse_client = MultiverseService()

    receive = multiverse_client.send_meta_data(meta_data_json)

    if receive.get("receive") is None:
        return
        
    object_name: str
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    tf_msgs = []
    root_frame_id = rospy.get_param('~root_frame_id') if rospy.has_param('~root_frame_id') else "map"
    object_names = receive["receive"].keys()
    for object_name in object_names:
        tf_msg = TransformStamped()
        tf_msg.header.frame_id = root_frame_id
        tf_msg.child_frame_id = object_name
        tf_msgs.append(tf_msg)

    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        data = multiverse_client.communicate()
        if len(data) == 0:
            break

        data.pop(0)
        for i, object_name in enumerate(object_names):
            tf_msgs[i].header.stamp = rospy.Time.now()
            tf_msgs[i].transform.translation.x = data.pop(0)
            tf_msgs[i].transform.translation.y = data.pop(0)
            tf_msgs[i].transform.translation.z = data.pop(0)
            tf_msgs[i].transform.rotation.w = data.pop(0)
            tf_msgs[i].transform.rotation.x = data.pop(0)
            tf_msgs[i].transform.rotation.y = data.pop(0)
            tf_msgs[i].transform.rotation.z = data.pop(0)

        tf_broadcaster.sendTransform(tf_msgs)
        rate.sleep()
        
    multiverse_client.deinit()


def start_multiverse_client() -> None:
    rospy.init_node('multiverse_client')
    threads = []
    if rospy.has_param('~publish/tf'):
        thread = threading.Thread(target=start_publish_tf)
        thread.start()
        threads.append(thread)

    for thread in threads:
        thread.join()


if __name__ == "__main__":
    if len(sys.argv) > 2 and sys.argv[1].isnumeric():
        port = sys.argv[1]
    start_multiverse_client()
