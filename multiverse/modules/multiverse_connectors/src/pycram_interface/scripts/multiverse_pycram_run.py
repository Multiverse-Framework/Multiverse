import argparse
import json

import yaml
from multiverse_ros_socket.multiverse_node.multiverse_node import SocketAddress

from multiverse_pycram_socket.multiverse_pycram_interface import Multiverse


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

    multiverse = Multiverse(client_addr=socket_addr)
    multiverse.run()
    multiverse.disconnect_from_physics_server()
