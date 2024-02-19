#!/usr/bin/env python3

import argparse
import json
import yaml

from multiverse_pycram_socket.multiverse_pycram import MultiversePycramInterface, SocketAddress


def main():
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

    MultiversePycramInterface._server_addr = socket_addr


if __name__ == "__main__":
    main()
