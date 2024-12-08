import os


def get_multiverse_connector_plugin():
    possible_mujoco_plugins_path_1 = os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "..",
                                                 "build", "mujoco", "bin", "mujoco_plugin", "libmultiverse_connector.so")
    possible_mujoco_plugins_path_2 = os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "..",
                                                 "..", "..", "..", "build", "mujoco", "bin", "mujoco_plugin",  "libmultiverse_connector.so")
    if os.path.exists(possible_mujoco_plugins_path_1):
        return possible_mujoco_plugins_path_1
    elif os.path.exists(possible_mujoco_plugins_path_2):
        return possible_mujoco_plugins_path_2
    else:
        raise FileNotFoundError(f"Could not find MuJoCo plugins directory at {possible_mujoco_plugins_path_1} "
                                f"or {possible_mujoco_plugins_path_2}")
