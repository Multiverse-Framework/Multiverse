#!/usr/bin/env python3

from multiverse_client.utils.multiverse_utils import set_send_meta_data_json
import rospy
import sys
from multiverse_msgs.srv import Socket, SocketRequest, SocketResponse
from multiverse_msgs.msg import ObjectAttribute, ObjectData
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

host = "tcp://127.0.0.1"
port = 7400

worlds = {}


def get_receive_meta_data(send_meta_data_dict: dict):
    multiverse_socket = MultiverseSocket(False)
    multiverse_socket.init(host, port)
    multiverse_socket.set_send_meta_data(send_meta_data_dict)
    multiverse_socket.connect()
    receive_meta_data_dict = multiverse_socket.get_receive_meta_data()
    multiverse_socket.disconnect()

    return receive_meta_data_dict


def update_world():
    send_meta_data_dict = set_send_meta_data_json()
    send_meta_data_dict["receive"][""] = [""]

    receive_meta_data_dict = get_receive_meta_data(send_meta_data_dict)

    world_name = receive_meta_data_dict["world"]
    worlds[world_name] = {}
    worlds[world_name][""] = [""]
    for object_name, object_data in receive_meta_data_dict["receive"].items():
        worlds[world_name][object_name] = []
        for attribute_name in object_data:
            worlds[world_name][""].append(attribute_name)
            worlds[world_name][object_name].append(attribute_name)
            worlds[world_name][object_name].append("")


def query_data_handle(req: SocketRequest):
    res = SocketResponse()

    send_meta_data_dict = {}
    world_name = "world" if req.world == "" else req.world

    send_meta_data_dict["world"] = world_name
    send_meta_data_dict["length_unit"] = "m" if req.length_unit == "" else req.length_unit
    send_meta_data_dict["angle_unit"] = "rad" if req.angle_unit == "" else req.angle_unit
    send_meta_data_dict["force_unit"] = "N" if req.force_unit == "" else req.force_unit
    send_meta_data_dict["time_unit"] = "s" if req.time_unit == "" else req.time_unit
    send_meta_data_dict["handedness"] = "rhs" if req.handedness == "" else req.handedness

    send_meta_data_dict["send"] = {}
    send_meta_data_dict["receive"] = {}

    world_need_update = False

    if worlds.get(world_name) is None:
        world_need_update = True
    else:
        data: ObjectAttribute
        for data in req.receive:
            if worlds[world_name].get(data.object_name) is None:
                world_need_update = True
                break
            send_meta_data_dict["receive"][data.object_name] = []
            for attribute_name in data.attribute_names:
                if worlds[world_name][data.object_name].count(attribute_name) == 0:
                    world_need_update = True
                    break

    if world_need_update:
        update_world()

    for data in req.receive:
        if worlds[world_name].get(data.object_name) is None:
            break
        send_meta_data_dict["receive"][data.object_name] = []
        for attribute_name in data.attribute_names:
            if worlds[world_name][data.object_name].count(attribute_name) == 0:
                break
            send_meta_data_dict["receive"][data.object_name].append(
                attribute_name)

    receive_meta_data_dict = get_receive_meta_data(send_meta_data_dict)

    res.world = receive_meta_data_dict["world"]
    res.length_unit = receive_meta_data_dict["length_unit"]
    res.angle_unit = receive_meta_data_dict["angle_unit"]
    res.force_unit = receive_meta_data_dict["force_unit"]
    res.time_unit = receive_meta_data_dict["time_unit"]
    res.handedness = receive_meta_data_dict["handedness"]

    for object_name, object_data in receive_meta_data_dict["receive"].items():
        for attribute_name, attribute_data in object_data.items():
            res.receive.append(ObjectData(
                object_name, attribute_name, attribute_data))

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
