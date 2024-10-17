from multiverse_client_py import MultiverseClient, MultiverseMetaData, SocketAddress

class IsaacSimConnector(MultiverseClient):
    def __init__(self, client_addr: SocketAddress, multiverse_meta_data: MultiverseMetaData) -> None:
        super().__init__(client_addr, multiverse_meta_data)

    def loginfo(self, message: str) -> None:
        print(f"INFO: {message}")

    def logwarn(self, message: str) -> None:
        print(f"WARN: {message}")

    def _run(self) -> None:
        self.loginfo("Start running the Isaac Sim Connector.")
        self._connect_and_start()

    def send_and_receive_meta_data(self) -> None:
        self.loginfo("Sending request meta data: " + str(self.request_meta_data))
        self._communicate(True)
        self.loginfo("Received response meta data: " + str(self.response_meta_data))

    def send_and_receive_data(self) -> None:
        self.loginfo("Sending data: " + str(self.send_data))
        self._communicate(False)
        self.loginfo("Received data: " + str(self.receive_data))

### Below is the code to run the Isaac Sim Connector

import argparse
import sys
from isaacsim import SimulationApp

if __name__ == "__main__":
    # This sample loads a usd stage and starts simulation
    CONFIG = {"width": 1280, "height": 720, "sync_loads": True, "headless": False, "renderer": "RayTracedLighting"}

    # Set up command line arguments
    parser = argparse.ArgumentParser("Multiverse Isaac Sim Connector")
    parser.add_argument(
        "--usd_path", type=str, help="Absolute path to usd file", required=True
    )
    parser.add_argument("--headless", default=False, action="store_true", help="Run stage headless")

    args, unknown = parser.parse_known_args()
    # Start the omniverse application
    CONFIG["headless"] = args.headless
    simulation_app = SimulationApp(CONFIG)

    import omni
    import carb
    from omni.isaac.nucleus import is_file
    import numpy as np

    usd_path = args.usd_path

    # make sure the file exists before we try to open it
    try:
        result = is_file(usd_path)
    except:
        result = False

    if result:
        omni.usd.get_context().open_stage(usd_path)
    else:
        carb.log_error(
            f"the usd path {usd_path} could not be opened, please make sure that {args.usd_path} is a valid usd file in {assets_root_path}"
        )
        simulation_app.close()
        sys.exit()

    # Wait two frames so that stage starts loading
    simulation_app.update()
    simulation_app.update()

    print("Loading stage...")
    from omni.isaac.core.utils.stage import is_stage_loading

    while is_stage_loading():
        simulation_app.update()
    print("Loading Complete")
    omni.timeline.get_timeline_interface().play()

    while simulation_app.is_running():
        

        # Run in realtime mode, we don't specify the step size
        simulation_app.update()

    simulation_app.close() # close Isaac Sim