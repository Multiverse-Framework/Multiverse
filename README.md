# Multiverse

Multiverse is a simulation framework designed to integrate multiple advanced physics engines such as [MuJoCo](https://mujoco.readthedocs.io/), [Project Chrono](https://projectchrono.org/), and [SOFA](https://www.sofa-framework.org/) along with various photo-realistic graphics engines like [Unreal Engine](https://www.unrealengine.com/) and [Omniverse](https://developer.nvidia.com/omniverse). Additionally, Multiverse provides the capability to generate knowledge graphs dynamically during runtime.

[Demo1](https://github.com/Universal-Simulation-Framework/multiverse/assets/64316740/19a3281f-ddd7-4430-b5ad-8219f9d17a92)

[Demo2](https://github.com/Multiverse-Framework/Multiverse/assets/64316740/e2509d42-39ad-4fa1-8224-2bcc55ef098f)

## Installation

### Linux

```bash
./install.sh                    # Install all prerequisites (sudo required)
./build_third_parties.sh        # Build the dependencies
./build_multiverse.sh           # Build the software
./build_multiverse_ws.sh        # Build the ROS workspace (only for Ubuntu 20.04)
./build_multiverse_ws2.sh       # Build the ROS2 workspace (for Ubuntu >= 20.04)
```

### Windows 11

```bash
install.bat                     # Install all prerequisites
build_multiverse.bat            # Build the software
```

## Test

```bash
multiverse_launch 
```
