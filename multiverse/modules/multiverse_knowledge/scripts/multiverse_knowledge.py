#!/usr/bin/env python3

import signal
import subprocess
import threading
from typing import List
from time import sleep, time

from multiverse_client_py import MultiverseClient, MultiverseMetaData, SocketAddress


class MultiverseKnowledge(MultiverseClient):
    def __init__(
        self, client_addr: SocketAddress, multiverse_meta_data: MultiverseMetaData
    ) -> None:
        super().__init__(client_addr, multiverse_meta_data)

    def loginfo(self, message: str) -> None:
        print(message)

    def logwarn(self, message: str) -> None:
        print(message)

    def _run(self) -> None:
        self._connect_and_start()

    def send_and_receive_meta_data(self):
        self._communicate(True)

    def send_and_receive_data(self):
        self._communicate(False)


data = None


def func_1(args: List[str]) -> List[str]:
    return args


def func_2(args: List[str]) -> List[str]:
    return [" ".join(args)]


def get_data(args: List[str]) -> List[str]:
    responses = []
    for arg in args:
        if arg == "time":
            responses.append(str(data[0]))
    return responses


if __name__ == "__main__":
    world_name = "world"
    port = "1234"
    rate = 10.0

    client_addr = SocketAddress(port=port)
    meta_data = MultiverseMetaData(
        length_unit="m",
        angle_unit="rad",
        mass_unit="kg",
        time_unit="s",
        handedness="rhs",
    )
    meta_data.world_name = world_name
    meta_data.simulation_name = "multiverse_knowledge"
    multiverse_client = MultiverseKnowledge(
        client_addr=client_addr, multiverse_meta_data=meta_data
    )
    multiverse_client.request_meta_data["receive"][""] = [""]
    multiverse_client.api_callbacks = {
        "func_1": func_1, 
        "func_2": func_2,
        "get_data": get_data,
    }
    multiverse_client.run()

    def signal_handler(sig, frame):
        multiverse_client.stop()
        exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    time_start = time()
    while True:
        # multiverse_client.send_and_receive_meta_data()
        multiverse_client.send_data = [time() - time_start]
        multiverse_client.send_and_receive_data()
        data = multiverse_client.receive_data
        sleep(1.0 / rate)
        print(data)
