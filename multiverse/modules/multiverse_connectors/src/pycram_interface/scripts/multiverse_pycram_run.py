#!/usr/bin/env python3

import argparse
import json
import os

import rospy
import yaml

from multiverse_pycram_socket.multiverse_pycram_interface import Multiverse
from multiverse_pycram_socket.multiverse_socket import SocketAddress
from pycram.enums import ObjectType
from pycram.urdf_interface import ObjectDescription
from pycram.world_object import Object

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Multiverse parser")
    parser.add_argument(
        "--multiverse_server",
        type=yaml.safe_load,
        required=False,
        help="Parameters for multiverse server",
    )
    args = parser.parse_args()

    multiverse_server = (
        json.loads(args.multiverse_server.replace("'", '"'))
        if isinstance(args.multiverse_server, str)
        else {"host": "tcp://127.0.0.1", "port": 7000}
    )

    SocketAddress.host = multiverse_server["host"]
    socket_addr = SocketAddress(port=str(multiverse_server["port"]))

    Multiverse._server_addr = socket_addr
    multiverse = Multiverse(client_addr=SocketAddress(port="5481"), is_prospection=True)
    if 'bin' in os.path.dirname(__file__):
        Multiverse.add_resource_path(os.path.join(os.path.dirname(__file__), '..',
                                                  "resources/objects/wooden_log/meshes/stl"))
    else:
        Multiverse.add_resource_path(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..',
                                                  "resources/objects/wooden_log/meshes/stl"))
    table = Object("wooden_log_1", ObjectType.GENERIC_OBJECT, "WoodenLog.stl", ObjectDescription)

    # while no keyboard interrupt
    while not rospy.is_shutdown():
        table_position = table.get_position_as_list()
        print(table_position)
        table_position[0] += 1
        table.set_position(table_position)
        rospy.sleep(1)
    multiverse.disconnect_from_physics_server()
