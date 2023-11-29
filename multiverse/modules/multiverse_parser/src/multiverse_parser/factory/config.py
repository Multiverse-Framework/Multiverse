from dataclasses import dataclass
import os
import random
import string


def get_random_string(length: int = 10) -> str:
    return "".join(random.choices(string.ascii_letters + string.digits, k=length))


def get_random_path() -> str:
    return os.path.join(
        "/tmp",
        "cache",
        get_random_string(length=10),
    )


@dataclass
class Cache:
    TMP = "tmp"
    TMP_DIR = f"{TMP}/usd"
    TMP_USD_FILE_DIR = get_random_path()
    TMP_USD_FILE_PATH = os.path.join(TMP_USD_FILE_DIR, TMP + ".usda")
    TMP_USD_MESH_PATH = os.path.join(TMP_USD_FILE_DIR, TMP_DIR)


def randomize_path() -> None:
    Cache.TMP_USD_FILE_DIR = get_random_path()
    Cache.TMP_USD_FILE_PATH = os.path.join(Cache.TMP_USD_FILE_DIR, Cache.TMP + ".usda")
    Cache.TMP_USD_MESH_PATH = os.path.join(Cache.TMP_USD_FILE_DIR, Cache.TMP_DIR)


@dataclass
class Configuration:
    model_name: str = ""
    with_physics: bool = True
    with_visual: bool = True
    with_collision: bool = True
    collision_rgba: tuple = (1.0, 0.0, 0.0, 1.0)
