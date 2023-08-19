import os
import importlib.util
import random, string

TMP = "tmp"
TMP_DIR = TMP + "/usd"
TMP_USD_FILE_DIR = os.path.join(
    os.path.dirname(importlib.util.find_spec("multiverse_parser").origin),
    ".cache",
    "".join(random.choices(string.ascii_letters + string.digits, k=10)),
)
TMP_USD_FILE_PATH = os.path.join(TMP_USD_FILE_DIR, TMP + ".usda")
TMP_USD_MESH_PATH = os.path.join(TMP_USD_FILE_DIR, TMP_DIR)

COLLISION_MESH_COLOR = [(1.0, 0.0, 0.0)]
COLLISION_MESH_OPACITY = [0.5]
