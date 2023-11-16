#!/usr/bin/env python3.10

import sys
import yaml
import argparse
from utils import run_subprocess


def main():
    parser = argparse.ArgumentParser(prog="multiverse_launch", description="Launch the multiverse framework")
    parser.add_argument(
        "--muv_file",
        type=str,
        required=True,
        help="Path to .muv file",
    )
    args = parser.parse_args()
    muv_file = args.muv_file

    try:
        with open(muv_file, "r") as file:
            data = yaml.safe_load(file)
    except Exception as e:
        print(f"Error reading MUV file: {e}")
        sys.exit(1)

    multiverse_server_dict = data.get("multiverse_server", {"host": "tcp://127.0.0.1", "port": "7000"})

    server_host = multiverse_server_dict.get("host", "tcp://127.0.0.1")
    server_port = multiverse_server_dict.get("port", "7000")
    process = run_subprocess(["multiverse_server", f"{server_host}:{server_port}"])


if __name__ == "__main__":
    main()
