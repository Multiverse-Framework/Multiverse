#!/usr/bin/env python3

from multiverse_client_py import MultiverseClient, MultiverseMetaData, SocketAddress
from time import time, sleep

class MultiverseClientTest(MultiverseClient):
    def __init__(self, client_addr: SocketAddress, multiverse_meta_data: MultiverseMetaData) -> None:
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


meta_data = MultiverseMetaData(
    world_name="world",
    length_unit="m",
    angle_unit="rad",
    mass_unit="kg",
    time_unit="s",
    handedness="rhs",
)

if __name__ == "__main__":
    meta_data_spawn = meta_data
    meta_data_spawn.simulation_name = "sim_test_spawn"
    multiverse_client_test_spawn = MultiverseClientTest(client_addr=SocketAddress(port="1236"),
                                                        multiverse_meta_data=meta_data_spawn)
    multiverse_client_test_spawn.run()

    multiverse_client_test_spawn.request_meta_data["meta_data"]["simulation_name"] = "sim_test_send"
    multiverse_client_test_spawn.request_meta_data["send"]["object_2"] = ["position", "quaternion"]
    multiverse_client_test_spawn.send_and_receive_meta_data()

    world_time_start = multiverse_client_test_spawn.response_meta_data["time"]
    time_start = time()
    world_time_now = time() - time_start + world_time_start

    multiverse_client_test_spawn.send_data = [world_time_now, 12, 11, 10, 0.0, 0.0, 0.0, 1.0]
    multiverse_client_test_spawn.send_and_receive_data()

    print(multiverse_client_test_spawn.receive_data)

    multiverse_client_test_spawn.stop()


    # i = 0
    # while i < 100:
    #     sleep(1)
    #     time_now = time() - time_start
    #     i += 1
    #
    #     multiverse_client_test_spawn.send_data = [time_now, i, i, i, 0.0, 0.0, 0.0, 1.0]
    #     multiverse_client_test_spawn.send_and_receive_data()
    #
    # multiverse_client_test_spawn.stop()
