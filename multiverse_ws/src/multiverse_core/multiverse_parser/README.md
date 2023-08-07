# Multiverse Parser

This package provides support for converting various scene descriptions to USD format and vice versa

## Usage

### From MJCF to USD

```bash
source Multiverse/multiverse_ws/devel/setup.bash
python3.10 Multiverse/multiverse_ws/src/multiverse_parser/mjcf_to_usd/scripts/mjcf_to_usd_node.py --in_mjcf=</path/to/mjcf> --out_usd=</path/to/usd>
```

If you need supports, run

```bash
python Multiverse/multiverse_ws/src/multiverse_parser/mjcf_to_usd/scripts/mjcf_to_usd_node.py --help
```

### From USD to URDF

```bash
source Multiverse/multiverse_ws/devel/setup.bash
python3.10 Multiverse/multiverse_ws/src/multiverse_parser/usd_to_urdf/scripts/usd_to_urdf_node.py --in_usd=</path/to/usd> --out_urdf=</path/to/urdf>
```

If you need supports, run

```bash
python Multiverse/multiverse_ws/src/multiverse_parser/usd_to_urdf/scripts/usd_to_urdf_node.py --help
```

### From URDF to MJCF

```bash
source Multiverse/multiverse_ws/devel/setup.bash
rosrun multiverse_parser urdf_to_mjcf <in_urdf.urdf> <out_mjcf.xml> <disable_parent_child_collision_level>
```

### From URDF to FBX

```bash
source Multiverse/multiverse_ws/devel/setup.bash
python3.10 Multiverse/multiverse_ws/src/multiverse_parser/urdf_to_fbx/scripts/urdf_to_fbx_node.py --in_urdf=</path/to/urdf> --out_fbx=</path/to/fbx>
```

If you need supports, run

```bash
python Multiverse/multiverse_ws/src/multiverse_parser/urdf_to_fbx/scripts/urdf_to_fbx_node.py --help
```

### From WORLD to URDF

```bash
source Multiverse/multiverse_ws/devel/setup.bash
python3.10 Multiverse/multiverse_ws/src/multiverse_parser/world_to_urdf/scripts/world_to_urdf_node.py --in_world=</path/to/world> --out_urdf=</path/to/urdf>
```

If you need supports, run

```bash
python Multiverse/multiverse_ws/src/multiverse_parser/world_to_urdf/scripts/world_to_urdf_node.py --help
```
