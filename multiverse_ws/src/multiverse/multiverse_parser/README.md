# multiverse Parser

This package provides support for converting various scene descriptions to USD format and vice versa

## Usage

### From MJCF to USD

No physics, only visual

```bash
rosrun multiverse_parser mujoco_to_usd_no_physics.py <in_mjcf.xml> <out_usd.usda>
```

With physics

```bash
rosrun multiverse_parser mujoco_to_usd_with_physics.py <in_mjcf.xml> <out_usd.usda>
```

Examples

```bash
roslaunch multiverse_parser mjcf_to_usd.launch
```
