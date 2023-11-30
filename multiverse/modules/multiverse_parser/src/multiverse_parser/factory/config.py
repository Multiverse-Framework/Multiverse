from dataclasses import dataclass


@dataclass
class Configuration:
    model_name: str = ""
    with_physics: bool = True
    with_visual: bool = True
    with_collision: bool = True
    collision_rgba: tuple = (1.0, 0.0, 0.0, 1.0)
