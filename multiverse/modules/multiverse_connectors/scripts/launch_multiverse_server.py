#!/usr/bin/env python3

from multiverse_launch import MultiverseLaunch
from utils import run_subprocess


def main():
    multiverse_launch = MultiverseLaunch()

    server_port = multiverse_launch.multiverse_server["port"]
    run_subprocess(["multiverse_server", f"tcp://*:{server_port}"])


if __name__ == "__main__":
    main()
