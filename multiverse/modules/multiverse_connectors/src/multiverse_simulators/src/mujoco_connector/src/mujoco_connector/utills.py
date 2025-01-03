import os


def get_multiverse_connector_plugins():
    possible_mujoco_plugins_dir_1 = os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "..",
                                                 "build", "mujoco", "bin", "mujoco_plugin")
    possible_mujoco_plugins_dir_2 = os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "..",
                                                 "..", "..", "..", "build", "mujoco", "bin", "mujoco_plugin")
    if os.path.exists(possible_mujoco_plugins_dir_1):
        plugins_dir = possible_mujoco_plugins_dir_1
    elif os.path.exists(possible_mujoco_plugins_dir_2):
        plugins_dir = possible_mujoco_plugins_dir_2
    else:
        raise FileNotFoundError(f"Could not find MuJoCo plugins directory at {possible_mujoco_plugins_dir_1} "
                                f"or {possible_mujoco_plugins_dir_2}")
    return [os.path.join(plugins_dir, f) for f in os.listdir(plugins_dir) if os.path.isfile(os.path.join(plugins_dir, f))]
