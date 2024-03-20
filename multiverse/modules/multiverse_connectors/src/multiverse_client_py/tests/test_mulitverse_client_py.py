import signal
import subprocess
import threading
import unittest
from time import sleep
from typing import Any

from multiverse_client_py import MultiverseClient, MultiverseMetaData, SocketAddress


def start_multiverse_server(server_port: str) -> subprocess.Popen:
    return subprocess.Popen(["multiverse_server", f"tcp://127.0.0.1:{server_port}"])


def kill_multiverse_server(process: subprocess.Popen):
    process.send_signal(signal.SIGINT)
    process.wait()


class MultiverseClientTest(MultiverseClient):
    def __init__(self, client_addr: SocketAddress, multiverse_meta_data: MultiverseMetaData) -> None:
        super().__init__(client_addr, multiverse_meta_data)

    def loginfo(self, message: str) -> None:
        print(message)

    def logwarn(self, message: str) -> None:
        print(message)

    def _run(self) -> None:
        self._connect_and_start()


class MultiverseClientCreationTestCase(unittest.TestCase):
    meta_data = MultiverseMetaData(
        world_name="world",
        simulation_name="sim_test",
        length_unit="m",
        angle_unit="rad",
        mass_unit="kg",
        time_unit="s",
        handedness="rhs",
    )

    def test_multiverse_client_creation(self):
        server_port = "6000"

        process = start_multiverse_server(server_port)
        MultiverseClientTest._server_addr.port = server_port
        multiverse_client_test = MultiverseClientTest(client_addr=SocketAddress(port="1234"),
                                                      multiverse_meta_data=self.meta_data)
        multiverse_client_test.run()
        sleep(1)
        multiverse_client_test.stop()
        kill_multiverse_server(process)


class MultiverseClientTestCase(unittest.TestCase):
    meta_data = MultiverseMetaData(
        world_name="world",
        length_unit="m",
        angle_unit="rad",
        mass_unit="kg",
        time_unit="s",
        handedness="rhs",
    )
    _server_port = "7000"
    _process = None

    @classmethod
    def setUpClass(cls) -> None:
        MultiverseClientTest._server_addr.port = cls._server_port
        cls._process = start_multiverse_server(cls._server_port)

    @classmethod
    def tearDownClass(cls) -> None:
        kill_multiverse_server(cls._process)

    def test_multiverse_client_send_creation_1(self):
        meta_data_send_1 = self.meta_data
        meta_data_send_1.simulation_name = "sim_test_send_1"
        multiverse_client_test_send_1 = MultiverseClientTest(client_addr=SocketAddress(port="1234"),
                                                             multiverse_meta_data=meta_data_send_1)
        multiverse_client_test_send_1.request_meta_data["send"]["object_1"] = ["position", "quaternion"]
        multiverse_client_test_send_1.run()
        self.assertDictEqual(multiverse_client_test_send_1.response_meta_data, {
            'meta_data': {'angle_unit': 'rad', 'handedness': 'rhs', 'length_unit': 'm', 'mass_unit': 'kg',
                          'simulation_name': 'sim_test_send_1', 'time_unit': 's', 'world_name': 'world'},
            'send': {'object_1': {'position': [None, None, None], 'quaternion': [None, None, None, None]}}, 'time': 0})

        multiverse_client_test_send_1.stop()

    def test_multiverse_client_send_and_receive_creation_1(self):
        meta_data_send_2 = self.meta_data
        meta_data_send_2.simulation_name = "sim_test_send_2"
        multiverse_client_test_send_2 = MultiverseClientTest(client_addr=SocketAddress(port="1235"),
                                                             multiverse_meta_data=meta_data_send_2)
        multiverse_client_test_send_2.request_meta_data["send"]["object_2"] = ["position", "quaternion"]
        multiverse_client_test_send_2.run()
        multiverse_client_test_send_2.stop()

        meta_data_receive_2 = self.meta_data
        meta_data_receive_2.simulation_name = "sim_test_receive_2"
        multiverse_client_test_receive_2 = MultiverseClientTest(client_addr=SocketAddress(port="1236"),
                                                                multiverse_meta_data=meta_data_receive_2)
        multiverse_client_test_receive_2.request_meta_data["receive"]["object_2"] = ["position", "quaternion"]
        multiverse_client_test_receive_2.run()
        multiverse_client_test_receive_2.stop()

    def test_multiverse_client_send_creation_2(self):
        meta_data_send_3 = self.meta_data
        meta_data_send_3.simulation_name = "sim_test_send_3"
        multiverse_client_test_send_1 = MultiverseClientTest(client_addr=SocketAddress(port="1237"),
                                                             multiverse_meta_data=meta_data_send_3)
        multiverse_client_test_send_1.request_meta_data["send"]["object_1"] = ["position", "quaternion"]
        multiverse_client_test_send_1.run()
        multiverse_client_test_send_1.stop()




if __name__ == "__main__":
    unittest.main()
