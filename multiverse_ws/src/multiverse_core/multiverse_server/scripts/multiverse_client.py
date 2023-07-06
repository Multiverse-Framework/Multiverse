#!/usr/bin/env python3

import zmq
import rospy
import sys
from json import dumps
from struct import pack, unpack
import threading
from multiverse_msgs.srv import QueryData, QueryDataRequest, QueryDataResponse
from multiverse_msgs.msg import Attribute

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


class MultiverseClient:
    def __init__(self) -> None:
        self.host = host
        self.port = port
        self.socket_addr = self.host + ":" + self.port
        self.is_enabled = False

    def send_meta_data(self, meta_data: QueryDataRequest):
        if not meta_data.receive:
            return

        receive_buffer_size = 1

        meta_data_json = {}
        meta_data_json["simulator"] = meta_data.simulator
        meta_data_json["length_unit"] = meta_data.length_unit
        meta_data_json["angle_unit"] = meta_data.angle_unit
        meta_data_json["force_unit"] = meta_data.force_unit
        meta_data_json["time_unit"] = meta_data.time_unit
        meta_data_json["handedness"] = meta_data.handedness

        meta_data_json["receive"] = {}
        data: Attribute
        for data in meta_data.receive:
            meta_data_json["receive"][data.object_name] = data.attribute
            for attribute in data.attribute:
                receive_buffer_size += len(attribute_map[attribute])

        self.context = zmq.Context()
        self.socket_client = self.context.socket(zmq.REQ)

        rospy.loginfo(f"Open the socket connection on {self.socket_addr}")
        self.socket_client.connect(self.socket_addr)

        while not rospy.is_shutdown():
            # Send JSON string over ZMQ
            self.socket_client.send_string(dumps(meta_data_json), flags=0)

            # Receive buffer sizes over ZMQ
            message = self.socket_client.recv()
            buffer = list(unpack('d' * (len(message) // 8), message))
            if buffer[0] < 0:
                rospy.logwarn(
                    f"The socket server at {self.socket_addr} has been terminated, resend the message")
                self.socket_client.disconnect(self.socket_addr)
                self.socket_client.connect(self.socket_addr)
            else:
                break

        if int(buffer[1]) != receive_buffer_size:
            rospy.logerr(
                f"Failed to initialize the socket at {self.socket_addr}: receive_buffer_size({buffer[1]}, client = 1).")
            self.socket_client.disconnect(self.socket_addr)
        else:
            self.is_enabled = True

    def communicate(self) -> list:
        if self.is_enabled:
            time_in_sec = rospy.Time().now().to_sec()
            self.socket_client.send(bytearray(pack('d', time_in_sec)))

            message = self.socket_client.recv()
            return list(unpack('d' * (len(message) // 8), message))
        else:
            return []

    def deinit(self) -> None:
        rospy.loginfo(f"Closing the socket client on {self.socket_addr}")
        if self.is_enabled:
            self.socket_client.send_string("{}")
            self.socket_client.disconnect(self.socket_addr)
            self.is_enabled = False
            self.context.destroy()


def query_data_handle(req: QueryDataRequest):
    res = QueryDataResponse()

    multiverse_client = MultiverseClient()
    multiverse_client.send_meta_data(req)
    res.data = multiverse_client.communicate()
    multiverse_client.deinit()

    return res


def start_multiverse_client() -> None:
    rospy.init_node('multiverse_client')
    rospy.Service('/multiverse/query_data', QueryData, query_data_handle)
    rospy.loginfo("Start service /multiverse/query_data")
    rospy.spin()


if __name__ == "__main__":
    if len(sys.argv) > 1:
        port = sys.argv[1]
    start_multiverse_client()
