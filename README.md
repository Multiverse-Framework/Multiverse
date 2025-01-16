# Multiverse

Multiverse is a simulation framework designed to integrate multiple advanced physics engines such as [MuJoCo](https://mujoco.readthedocs.io/), [Project Chrono](https://projectchrono.org/), and [SOFA](https://www.sofa-framework.org/) along with various photo-realistic graphics engines like [Unreal Engine](https://www.unrealengine.com/) and [Omniverse](https://developer.nvidia.com/omniverse). Additionally, Multiverse provides the capability to generate knowledge graphs dynamically during runtime.

[Demo1](https://github.com/Universal-Simulation-Framework/multiverse/assets/64316740/19a3281f-ddd7-4430-b5ad-8219f9d17a92)

[Demo2](https://github.com/Multiverse-Framework/Multiverse/assets/64316740/e2509d42-39ad-4fa1-8224-2bcc55ef098f)

## Documentation
https://multiverseframework.readthedocs.io/en/latest

## Installation

```bash
git clone https://github.com/Multiverse-Framework/Multiverse --depth 1
```

Install IsaacSim and IsaacLab according to this [documentation](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/binaries_installation.html#installation-using-isaac-sim-binaries). Make sure Isaac Sim is installed in the directory `${HOME}/.local/share/ov/pkg/isaac_sim-*`, with `*` corresponding to the Isaac Sim version.

### Linux

```bash
./install.sh                                                                    # Install all prerequisites
./build_third_parties.sh #--excludes blender usd mujoco pybind11                # Build the dependencies with optional exclusions
./build_multiverse.sh #--only-src / --only-modules connectors parser knowledge  # Build the software with optional inclusions
./build_multiverse_ws.sh                                                        # Build the ROS workspace (only for Ubuntu 20.04)
./build_multiverse_ws2.sh                                                       # Build the ROS2 workspace (for Ubuntu >= 20.04)
```

### Windows 11

Run Windows Powershell as Administrator and execute these scripts, restart the Powershell before running each script to refresh the environment paths

```bash
install.bat                     # Install all prerequisites (as Administrator)
build_third_parties.bat         # Build the dependencies
build_multiverse.bat            # Build the software
build_multiverse_ws2.bat        # Build the ROS2 workspace
```

## Test

```bash
multiverse_launch 
```

## Citation

If you find this software helpful, please cite it as follows:

```bibtex
@software{Multiverse,
  author = {Giang Nguyen, Michael Beetz},
  title = {Multiverse: Simulate Everything, Everywhere, All At Once},
  month = {December},
  year = {2024},
  url = {https://github.com/Multiverse-Framework/Multiverse},
  doi = {https://doi.org/10.5281/zenodo.14035537}
}
```
