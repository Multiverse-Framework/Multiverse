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
    time_start = time()
    meta_data_send = meta_data
    meta_data_send.simulation_name = "sim_test_send"
    multiverse_client_test_send = MultiverseClientTest(client_addr=SocketAddress(port="1235"),
                                                       multiverse_meta_data=meta_data_send)
    multiverse_client_test_send.request_meta_data["send"]["object_1"] = ["position", "quaternion"]
    multiverse_client_test_send.run()

    i = 0
    # while i < 100:
    #     sleep(1)
    #     time_now = time() - time_start
    #     i += 1
    #
    #     multiverse_client_test_send.send_data = [time_now, i, i, i, 0.0, 0.0, 0.0, 1.0]
    #     multiverse_client_test_send.send_and_receive_data()

    time_now = time() - time_start
    i += 1

    print("NOW")

    multiverse_client_test_send.send_data = [time_now, i, i, i, 0.0, 0.0, 0.0, 1.0]
    multiverse_client_test_send.send_and_receive_data()

    sleep(5)
    time_now = time() - time_start
    i += 1

    multiverse_client_test_send.send_data = [time_now, i, i, i, 0.0, 0.0, 0.0, 1.0]
    multiverse_client_test_send.send_and_receive_data()
    print(multiverse_client_test_send.request_meta_data)
    print(multiverse_client_test_send.response_meta_data)

    sleep(5)
    time_now = time() - time_start
    i += 1

    meta_data_receive = meta_data
    meta_data_receive.simulation_name = "sim_test_receive"
    multiverse_client_test_receive = MultiverseClientTest(client_addr=SocketAddress(port="1239"),
                                                          multiverse_meta_data=meta_data_receive)
    multiverse_client_test_receive.request_meta_data["receive"]["object_1"] = ["position", "quaternion"]
    multiverse_client_test_receive.request_meta_data["receive"]["object_2"] = ["position", "quaternion"]
    multiverse_client_test_receive.run()
    multiverse_client_test_receive.send_data = [time_now]
    multiverse_client_test_receive.send_and_receive_data()
    print(multiverse_client_test_receive.receive_data)

    multiverse_client_test_send.send_data = [time_now, i, i, i, 0.0, 0.0, 0.0, 1.0, i, i, i, 0.0, 0.0, 0.0, 1.0]
    multiverse_client_test_send.send_and_receive_data()

    sleep(1)
    time_now = time() - time_start
    i += 1

    multiverse_client_test_receive.send_data = [time_now]
    multiverse_client_test_receive.send_and_receive_data()
    print(multiverse_client_test_receive.receive_data)

    multiverse_client_test_send.send_data = [time_now, i, i, i, 0.0, 0.0, 0.0, 1.0, i, i, i, 0.0, 0.0, 0.0, 1.0]
    multiverse_client_test_send.send_and_receive_data()

    sleep(1)
    time_now = time() - time_start
    i += 1

    multiverse_client_test_receive.send_data = [time_now]
    multiverse_client_test_receive.send_and_receive_data()
    print(multiverse_client_test_receive.receive_data)

    multiverse_client_test_send.send_data = [time_now, i, i, i, 0.0, 0.0, 0.0, 1.0, i, i, i, 0.0, 0.0, 0.0, 1.0]
    multiverse_client_test_send.send_and_receive_data()

    multiverse_client_test_receive.send_data = [time_now]
    multiverse_client_test_receive.send_and_receive_data()
    print(multiverse_client_test_receive.receive_data)

    multiverse_client_test_send.stop()
