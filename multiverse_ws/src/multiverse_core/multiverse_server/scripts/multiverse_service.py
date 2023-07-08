#!/usr/bin/env python3

import zmq
import rospy
import sys
from json import dumps
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
port = "7400"


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
        self.socket_client.send_string("{}")
        self.socket_client.disconnect(self.socket_addr)
        self.is_enabled = False
        self.context.destroy()


def query_data_handle(req: QueryDataRequest):
    res = QueryDataResponse()

    multiverse_service = MultiverseService()
    receive = multiverse_service.send_meta_data(req)
    multiverse_service.deinit()

    if receive.get("receive") is not None:
        object_name: str
        object_data: dict
        for object_name, object_data in receive["receive"].items():
            for attribute_name, data in object_data.items():
                res.receive.append(ObjectData(
                    object_name, attribute_name, data))

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
