from dataclasses import dataclass
from typing import Optional


@dataclass
class Configuration:
    model_name: str = ""
    with_physics: bool = True
    with_visual: bool = True
    with_collision: bool = True
    rgba: Optional[tuple] = None
