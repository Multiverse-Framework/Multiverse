from semantic_digital_twin.adapters.mjcf import MJCFParser
from semantic_digital_twin.adapters.multi_sim import MujocoSim
import os
import time
import mujoco

if __name__ == "__main__":
    scene_path = os.path.join(
        os.path.dirname(__file__), "assets", "mjcf", "iai_tiago_position_in_apartment.xml"
    )
    image_dir = os.path.join(os.path.dirname(__file__), "..", "images")
    world = MJCFParser(scene_path).parse()
    headless = (
        os.environ.get("CI", "false").lower() == "true"
    )  # headless in CI environments
    multi_sim = MujocoSim(
        world=world,
        headless=headless,
        step_size=0.002,
        integrator=mujoco.mjtIntegrator.mjINT_EULER,
        cone=mujoco.mjtCone.mjCONE_ELLIPTIC,
    )
    multi_sim.start_simulation()

    print("Wait 1s...")
    time.sleep(1)
    print("Everything is ready")

    try:
        for i in range(600):
            # capture_rgb = multi_sim.simulator.capture_rgb(camera_name="head_camera")
            # rgb = capture_rgb.result
            # # Save as png
            # cv2.imwrite(os.path.join(image_dir, f"rgb_{i}.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stop simulation!")
    finally:
        multi_sim.stop_simulation()