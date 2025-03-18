#!/usr/bin/env python3

from multiverse_client_py import MultiverseClient, MultiverseMetaData

class MultiverseInitializer(MultiverseClient):
    def __init__(self, port: str, multiverse_meta_data: MultiverseMetaData) -> None:
        super().__init__(port, multiverse_meta_data)

    def loginfo(self, message: str) -> None:
        print(f"INFO: {message}")

    def logwarn(self, message: str) -> None:
        print(f"WARN: {message}")

    def _run(self) -> None:
        self.loginfo("Start logging.")
        self._connect_and_start()

    def send_and_receive_meta_data(self) -> None:
        self.loginfo("Sending request meta data: " + str(self.request_meta_data))
        self._communicate(True)
        self.loginfo("Received response meta data: " + str(self.response_meta_data))

    def send_and_receive_data(self) -> None:
        self._communicate(False)

import argparse
import time
import numpy
import yaml

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=f"Logging data from multiverse.")

    # Define arguments
    parser.add_argument("--world_name", type=str, required=False, default="world", help="Name of the world")
    parser.add_argument("--port", type=str, required=False, default="5432", help="Port number")
    parser.add_argument("--data_path", type=str, required=False, default="data.yaml", help="Path to load the data")

    # Parse arguments
    args = parser.parse_args()

    receive_data = []

    multiverse_meta_data = MultiverseMetaData(
        world_name=args.world_name,
        simulation_name="multiverse_initializer",
        length_unit="m",
        angle_unit="rad",
        mass_unit="kg",
        time_unit="s",
        handedness="rhs",
    )
    multiverse_initializer = MultiverseInitializer(port=args.port,
                                                   multiverse_meta_data=multiverse_meta_data)
    multiverse_initializer.run()

    data = yaml.load(open(args.data_path, 'r'), Loader=yaml.FullLoader)
    object_names = list(data.keys())

    multiverse_initializer.request_meta_data["send"] = {}
    multiverse_initializer.request_meta_data["receive"] = {}
    
    for object_name in object_names:
        multiverse_initializer.request_meta_data["send"][object_name] = []
        for attribute_name, attribute_data in data[object_name].items():
            multiverse_initializer.request_meta_data["send"][object_name].append(attribute_name)
    multiverse_initializer.send_and_receive_meta_data()
    
    response_meta_data = multiverse_initializer.response_meta_data
    send_data = []
    for object_name, attributes in response_meta_data["send"].items():
        for attribute_name in attributes.keys():
            send_data += data[object_name][attribute_name]

    multiverse_initializer.send_data = [0.0] + send_data
    multiverse_initializer.send_and_receive_data()
    multiverse_initializer.stop()
