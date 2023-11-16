#!/usr/bin/env python3.10

import os
import glob
from typing import List
import subprocess


def run_subprocess(cmd: List[str]):
    cmd_str = " ".join(cmd)
    print(f'Execute "{cmd_str}"')
    return subprocess.Popen(cmd)


def find_files(resources_paths: List[str], filename_pattern: str) -> str:
    matches = []
    for resources_path in resources_paths:
        search_pattern = os.path.join(resources_path, "**", filename_pattern)
        matches += [path for path in glob.iglob(search_pattern, recursive=True)]
    if len(matches) == 0:
        raise FileNotFoundError(f"{filename_pattern} not found in {resources_paths}")
    elif len(matches) >= 1:
        if len(matches) > 1:
            print(f"Multiple matches for '{filename_pattern}' detected: {matches}. Using the first one.")
        match = matches[0]
    return match
