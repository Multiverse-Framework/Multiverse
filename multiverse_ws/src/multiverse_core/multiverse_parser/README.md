# Multiverse Parser

This package provides support for converting various scene descriptions to USD format and vice versa

## Usage

### From MJCF to USD

No physics, only visual

```bash
rosrun multiverse_parser mjcf_to_usd_no_physics.py <in_mjcf.xml> <out_usd.usda>
```

With physics

```bash
rosrun multiverse_parser mjcf_to_usd_with_physics.py <in_mjcf.xml> <out_usd.usda>
```

Examples

```bash
roslaunch multiverse_parser mjcf_to_usd.launch
```
### From USD to URDF

```bash
rosrun multiverse_parser usd_to_urdf.py <in_usd.usda> <out_urdf.urdf>
```

Examples

```bash
roslaunch multiverse_parser usd_to_urdf.launch
```
### From URDF to MJCF

```bash
rosrun multiverse_parser urdf_to_mjcf <in_urdf.urdf> <out_mjcf.xml> <disable_parent_child_collision_level>
```

Examples

```bash
roslaunch multiverse_parser urdf_to_mjcf.launch
```
