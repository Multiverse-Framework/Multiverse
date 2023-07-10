#!/usr/bin/env python3

import rospy
import sys
from multiverse_msgs.srv import Socket, SocketRequest, SocketResponse
from multiverse_msgs.msg import ObjectAttribute, ObjectData
import os
sys.path.append(os.path.dirname(os.path.dirname(sys.argv[0]))) 
from multiverse_socket import MultiverseSocket

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
port = 7400


def query_data_handle(req: SocketRequest):
    res = SocketResponse()

    multiverse_socket = MultiverseSocket()

    meta_data_json = {}
    meta_data_json["world"] = "world" if req.world == "" else req.world
    meta_data_json["length_unit"] = "m" if req.length_unit == "" else req.length_unit
    meta_data_json["angle_unit"] = "rad" if req.angle_unit == "" else req.angle_unit
    meta_data_json["force_unit"] = "N" if req.force_unit == "" else req.force_unit
    meta_data_json["time_unit"] = "s" if req.time_unit == "" else req.time_unit
    meta_data_json["handedness"] = "rhs" if req.handedness == "" else req.handedness

    meta_data_json["send"] = {}
    meta_data_json["receive"] = {}
    data: ObjectAttribute
    for data in req.receive:
        meta_data_json["receive"][data.object_name] = data.attribute_names

    multiverse_socket.init(host, port)
    multiverse_socket.set_meta_data(meta_data_json)
    multiverse_socket.connect()

    meta_data_response = multiverse_socket.get_meta_data_response()

    res.world = meta_data_response["world"]
    res.length_unit = meta_data_response["length_unit"]
    res.angle_unit = meta_data_response["angle_unit"]
    res.force_unit = meta_data_response["force_unit"]
    res.time_unit = meta_data_response["time_unit"]
    res.handedness = meta_data_response["handedness"]
    
    for object_name, object_data in meta_data_response["receive"].items():
        for attribute_name, attribute_data in object_data.items():
            res.receive.append(ObjectData(object_name, attribute_name, attribute_data))
    
    multiverse_socket.disconnect()

    return res


def start_multiverse_service() -> None:
    rospy.init_node('multiverse_service')
    rospy.Service('/multiverse/query_data', Socket, query_data_handle)
    rospy.loginfo("Start service /multiverse/query_data")
    rospy.spin()


if __name__ == "__main__":
    if len(sys.argv) > 2 and sys.argv[1].isnumeric():
        port = int(sys.argv[1])
    start_multiverse_service()
