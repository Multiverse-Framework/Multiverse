#!/usr/bin/env python3

from multiverse_client_py import MultiverseClient, MultiverseMetaData

class MultiverseReplay(MultiverseClient):
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

import pandas as pd
import time

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Replay a simulation log file')
    # Define arguments
    parser.add_argument("--world_name", type=str, required=False, default="world", help="Name of the world")
    parser.add_argument("--port", type=str, required=False, default="4000", help="Port number")
    parser.add_argument("--data_path", type=str, default="data.csv", required=False, help='Path to the CSV file containing the simulation log data')
    
    # Parse arguments
    args = parser.parse_args()

    # Load the CSV file to examine its contents
    data_path = args.data_path
    data = pd.read_csv(data_path)

    # Display the first few rows of the data to understand its structure
    data.head()

    multiverse_meta_data = MultiverseMetaData(
        world_name=args.world_name,
        simulation_name="multiverse_smoother",
        length_unit="m",
        angle_unit="rad",
        mass_unit="kg",
        time_unit="s",
        handedness="rhs",
    )
    multiverse_logger = MultiverseReplay(port=args.port, multiverse_meta_data=multiverse_meta_data)
    multiverse_logger.run()

    data_columns = [col for col in data.columns if 'time' not in col]
    data_values = data[data_columns]

    multiverse_logger.request_meta_data["send"] = {}
    multiverse_logger.request_meta_data["receive"] = {}
    for data_name in data_columns:
        object_name = data_name.split(':')[0]
        attribute_name = data_name.split(':')[1]
        multiverse_logger.request_meta_data["send"][object_name] = [f"{attribute_name}"]
    multiverse_logger.send_and_receive_meta_data()

    multiverse_logger.send_data = [0.0] + data_values.iloc[0].values.tolist()
    multiverse_logger.send_and_receive_data()

    data_time = data['time']
    start_time = data_time[0]
    for i in range(len(data_time)):
        time.sleep(data_time[i] - start_time)
        multiverse_logger.send_data = [data_time[i]] + data_values.iloc[i].values.tolist()
        multiverse_logger.send_and_receive_data()
        start_time = data_time[i]
        if i % 100 == 0:
            print(f"Processing {i*100/len(data)}% - Time: {data_time[i]}")
