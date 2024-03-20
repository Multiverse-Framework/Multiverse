import signal
import subprocess
import threading
import unittest
from time import sleep, time
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

    def send_and_receive_meta_data(self):
        self._communicate(True)

    def send_and_receive_data(self):
        self._communicate(False)


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

    # @classmethod
    # def setUpClass(cls) -> None:
    #     MultiverseClientTest._server_addr.port = cls._server_port
    #     cls._process = start_multiverse_server(cls._server_port)
    #
    # @classmethod
    # def tearDownClass(cls) -> None:
    #     kill_multiverse_server(cls._process)

    def test_multiverse_client(self):
        time_start = time()
        meta_data_send = self.meta_data
        meta_data_send.simulation_name = "sim_test_send"
        multiverse_client_test_send = MultiverseClientTest(client_addr=SocketAddress(port="1235"),
                                                           multiverse_meta_data=meta_data_send)
        multiverse_client_test_send.request_meta_data["send"]["object_1"] = ["position", "quaternion"]
        multiverse_client_test_send.run()
        self.assertDictEqual(multiverse_client_test_send.response_meta_data, {
            'meta_data': {'angle_unit': 'rad', 'handedness': 'rhs', 'length_unit': 'm', 'mass_unit': 'kg',
                          'simulation_name': 'sim_test_send', 'time_unit': 's', 'world_name': 'world'},
            'send': {'object_1': {'position': [None, None, None], 'quaternion': [None, None, None, None]}}, 'time': 0})

        time_now = 0.0
        multiverse_client_test_send.send_data = [time_now, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]
        multiverse_client_test_send.send_and_receive_data()

        meta_data_receive = self.meta_data
        meta_data_receive.simulation_name = "sim_test_receive"
        multiverse_client_test_receive = MultiverseClientTest(client_addr=SocketAddress(port="1236"),
                                                              multiverse_meta_data=meta_data_receive)
        multiverse_client_test_receive.request_meta_data["receive"]["object_1"] = ["position", "quaternion"]
        multiverse_client_test_receive.run()
        self.assertDictEqual(multiverse_client_test_receive.response_meta_data, {
            'meta_data': {'angle_unit': 'rad', 'handedness': 'rhs', 'length_unit': 'm', 'mass_unit': 'kg',
                          'simulation_name': 'sim_test_receive', 'time_unit': 's', 'world_name': 'world'},
            'receive': {'object_1': {'position': [1.0, 2.0, 3.0], 'quaternion': [0.0, 0.0, 0.0, 1.0]}},
            'time': time_now})

        time_now = time() - time_start
        multiverse_client_test_send.send_data = [time_now, 3.0, 2.0, 1.0, 1.0, 0.0, 0.0, 0.0]
        multiverse_client_test_send.send_and_receive_data()
        self.assertEqual(multiverse_client_test_send.receive_data, [time_now])

        multiverse_client_test_receive.send_data = [time_now]
        multiverse_client_test_receive.send_and_receive_data()
        self.assertEqual(multiverse_client_test_receive.receive_data, [time_now, 3.0, 2.0, 1.0, 1.0, 0.0, 0.0, 0.0])

        meta_data_reset = self.meta_data
        meta_data_reset.simulation_name = "sim_test_reset"
        multiverse_client_test_reset = MultiverseClientTest(client_addr=SocketAddress(port="1237"),
                                                            multiverse_meta_data=meta_data_reset)
        multiverse_client_test_reset.run()
        multiverse_client_test_reset.send_data = [0.0]
        multiverse_client_test_reset.send_and_receive_data()
        self.assertEqual(multiverse_client_test_reset.receive_data, [0.0])

        time_now = time() - time_start
        multiverse_client_test_send.send_data = [time_now, 5.0, 2.0, 3.0, 0.0, 1.0, 0.0, 0.0]
        multiverse_client_test_send.send_and_receive_data()
        self.assertEqual(multiverse_client_test_send.receive_data, [0.0])
        multiverse_client_test_send.send_and_receive_data()
        self.assertEqual(multiverse_client_test_send.receive_data, [time_now])

        multiverse_client_test_receive.send_data = [time_now]
        multiverse_client_test_receive.send_and_receive_data()
        self.assertEqual(multiverse_client_test_receive.receive_data, [0.0, 5.0, 2.0, 3.0, 0.0, 1.0, 0.0, 0.0])
        multiverse_client_test_receive.send_and_receive_data()
        self.assertEqual(multiverse_client_test_receive.receive_data, [time_now, 5.0, 2.0, 3.0, 0.0, 1.0, 0.0, 0.0])

        meta_data_spawn = self.meta_data
        meta_data_spawn.simulation_name = "sim_test_spawn"
        multiverse_client_test_spawn = MultiverseClientTest(client_addr=SocketAddress(port="1238"),
                                                            multiverse_meta_data=meta_data_reset)
        multiverse_client_test_spawn.run()

        time_now = time() - time_start

        def send_and_receive_data():
            multiverse_client_test_spawn.request_meta_data["meta_data"]["simulation_name"] = "sim_test_send"
            multiverse_client_test_spawn.request_meta_data["send"]["object_2"] = ["position", "quaternion"]
            multiverse_client_test_spawn.send_and_receive_meta_data()
            multiverse_client_test_spawn.send_data = [time_now, 4.0, 3.0, 2.0, 0.0, 1.0, 0.0, 0.0]
            multiverse_client_test_spawn.send_and_receive_data()

        thread = threading.Thread(target=send_and_receive_data)
        thread.start()

        multiverse_client_test_send.send_and_receive_data()

        # multiverse_client_test_send.send_data = [time_now, 2.0, 5.0, 2.0, 0.0, 0.0, 1.0, 0.0]
        # multiverse_client_test_send.send_and_receive_data()

        multiverse_client_test_send.stop()
        multiverse_client_test_receive.stop()
        multiverse_client_test_reset.stop()
        multiverse_client_test_spawn.stop()

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
