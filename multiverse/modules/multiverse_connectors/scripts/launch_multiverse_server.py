#!/usr/bin/env python3.10

from multiverse_launch import MultiverseLaunch, run_subprocess


def main():
    multiverse_launch = MultiverseLaunch()

    server_host = multiverse_launch.multiverse_server["host"]
    server_port = multiverse_launch.multiverse_server["port"]
    run_subprocess(["multiverse_server", f"{server_host}:{server_port}"])


if __name__ == "__main__":
    main()
