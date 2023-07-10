#!/usr/bin/env python3

import zmq
import rospy
import sys
from json import dumps
from multiverse_msgs.srv import QueryData, QueryDataRequest, QueryDataResponse
from multiverse_msgs.msg import ObjectAttribute, ObjectData
import os
sys.path.append(os.path.dirname(os.path.dirname(sys.argv[0]))) 
from multiverse_client import MultiverseQueryData

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


class MultiverseService:
    def __init__(self) -> None:
        self.host = host
        self.port = port
        self.socket_addr = self.host + ":" + self.port

    def send_meta_data(self, meta_data: QueryDataRequest) -> dict:
        if not meta_data.receive:
            return

        meta_data_json = {}

        meta_data_json["length_unit"] = "m" if meta_data.length_unit == "" else meta_data.length_unit
        meta_data_json["angle_unit"] = "rad" if meta_data.angle_unit == "" else meta_data.angle_unit
        meta_data_json["force_unit"] = "N" if meta_data.force_unit == "" else meta_data.force_unit
        meta_data_json["time_unit"] = "s" if meta_data.time_unit == "" else meta_data.time_unit
        meta_data_json["handedness"] = "rhs" if meta_data.handedness == "" else meta_data.handedness

        meta_data_json["receive"] = {}
        data: ObjectAttribute
        for data in meta_data.receive:
            meta_data_json["receive"][data.object_name] = data.attribute_names

        rospy.loginfo(meta_data_json)

        self.context = zmq.Context()
        self.socket_client = self.context.socket(zmq.REQ)

        rospy.loginfo(f"Open the socket connection on {self.socket_addr}")
        self.socket_client.connect(self.socket_addr)

        # Send JSON string over ZMQ
        self.socket_client.send_string(dumps(meta_data_json), flags=0)

        # Receive buffer sizes over ZMQ
        return self.socket_client.recv_json()

    def deinit(self) -> None:
        rospy.loginfo(f"Closing the socket client on {self.socket_addr}")
        self.socket_client.send_string("\{\}")
        self.socket_client.disconnect(self.socket_addr)
        self.context.destroy()


def query_data_handle(req: QueryDataRequest):
    res = QueryDataResponse()

    multiverse_service = MultiverseQueryData()

    meta_data_json = {}
    meta_data_json["length_unit"] = "m" if req.length_unit == "" else req.length_unit
    meta_data_json["angle_unit"] = "rad" if req.angle_unit == "" else req.angle_unit
    meta_data_json["force_unit"] = "N" if req.force_unit == "" else req.force_unit
    meta_data_json["time_unit"] = "s" if req.time_unit == "" else req.time_unit
    meta_data_json["handedness"] = "rhs" if req.handedness == "" else req.handedness

    meta_data_json["receive"] = {}
    data: ObjectAttribute
    for data in req.receive:
        meta_data_json["receive"][data.object_name] = data.attribute_names

    multiverse_service.init(host, port)
    multiverse_service.set_meta_data(meta_data_json)
    multiverse_service.connect()
    
    multiverse_service.communicate()
    receive_data = multiverse_service.get_receive_data()
    multiverse_service.disconnect()

    return res


def start_multiverse_service() -> None:
    rospy.init_node('multiverse_service')
    rospy.Service('/multiverse/query_data', QueryData, query_data_handle)
    rospy.loginfo("Start service /multiverse/query_data")
    rospy.spin()


if __name__ == "__main__":
    if len(sys.argv) > 1:
        port = sys.argv[1]
    start_multiverse_service()
