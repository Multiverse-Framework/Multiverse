#!/usr/bin/env python3

from multiverse_client_py import MultiverseClient, MultiverseMetaData

class MultiverseLogger(MultiverseClient):
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

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=f"Logging data from multiverse.")

    # Define arguments
    parser.add_argument("--world_name", type=str, required=False, default="world", help="Name of the world")
    parser.add_argument("--port", type=str, required=False, default="5000", help="Port number")
    parser.add_argument("--save_path", type=str, required=False, default="data.csv", help="Path to save the data")
    parser.add_argument("--object_names", type=str, required=False, default="", help="Object names to log data from")
    parser.add_argument("--attribute_names", type=str, required=False, default="", help="Attribute names to log data from")
    parser.add_argument("--time", type=float, required=False, default=1.0, help="Time to log data for")
    parser.add_argument("--interpolation_factor", type=int, required=False, default=1, help="Interpolation factor")
    parser.add_argument("--time_step", type=float, required=False, default=0.01, help="Time step")

    # Parse arguments
    args = parser.parse_args()

    receive_data = []

    multiverse_meta_data = MultiverseMetaData(
        world_name=args.world_name,
        simulation_name="multiverse_logger",
        length_unit="m",
        angle_unit="rad",
        mass_unit="kg",
        time_unit="s",
        handedness="rhs",
    )
    multiverse_logger = MultiverseLogger(port=args.port,
                                         multiverse_meta_data=multiverse_meta_data)
    multiverse_logger.run()

    object_names = args.object_names.split(",")
    attribute_names = args.attribute_names.split(",")

    multiverse_logger.request_meta_data["send"] = {}
    multiverse_logger.request_meta_data["receive"] = {}
    for object_name in object_names:
        multiverse_logger.request_meta_data["receive"][object_name] = attribute_names
    multiverse_logger.send_and_receive_meta_data()

    multiverse_logger.send_data = [0.0]
    multiverse_logger.send_and_receive_data()

    data_size = len(multiverse_logger.receive_data)

    starting_time = multiverse_logger.sim_time
    current_time = starting_time
    start_record = False

    last_receive_data = multiverse_logger.receive_data
    N = args.interpolation_factor
    while current_time < starting_time + args.time:
        multiverse_logger.send_data = [current_time]
        multiverse_logger.send_and_receive_data()
        new_receive_data = [current_time] + multiverse_logger.receive_data[1:]
        if current_time >= starting_time + 0.1:
            for i in range(1, N+1):
                new_receive_data_each = []
                for j in range(len(new_receive_data)):
                    new_receive_data_each.append(new_receive_data[j] * i / N + ((N - i) / N) * last_receive_data[j])
                receive_data += new_receive_data_each
        last_receive_data = new_receive_data
        current_time = multiverse_logger.sim_time
        time.sleep(args.time_step)

    multiverse_logger.stop()

    data = {}
    data['time'] = receive_data[::data_size]
    data_adr = 1
    for object_name, object_data in multiverse_logger.response_meta_data["receive"].items():
        for attribute_name, attribute_values in object_data.items():
            if len(attribute_values) == 1:
                data_name = object_name + ":" + attribute_name
                data[data_name] = receive_data[data_adr::data_size]
            else:
                print(f"Attribute {attribute_name} has more than one value. Skipping.")
            data_adr += len(attribute_values)

    import pandas as pd

    # Create a DataFrame
    df = pd.DataFrame(data)

    # Writing to CSV, index=False to avoid writing row numbers
    df.to_csv('data.csv', index=False)
