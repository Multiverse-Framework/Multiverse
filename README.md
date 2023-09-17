# Multiverse

Multiverse is a simulation framework designed to integrate multiple advanced physics engines such as [MuJoCo](https://mujoco.readthedocs.io/), [Project Chrono](https://projectchrono.org/), and [SOFA](https://www.sofa-framework.org/) along with various photo-realistic graphics engines like [Unreal Engine](https://www.unrealengine.com/) and [Omniverse](https://developer.nvidia.com/omniverse). Additionally, Multiverse provides the capability to generate knowledge graphs dynamically during runtime.

[Demo1](https://github.com/Universal-Simulation-Framework/multiverse/assets/64316740/19a3281f-ddd7-4430-b5ad-8219f9d17a92)

[Demo2](https://github.com/Multiverse-Framework/Multiverse/assets/64316740/b7e1c84e-2d74-4290-ba07-c94143967c53)


# Installation

```
sudo ./install.sh      # Install all prerequisites
./build_third_parties.sh        # Build the dependencies
./build_multiverse.sh           # Build the software
./build_multiverse_ws.sh        # Build the ROS workspace
```
# Test

```
source multiverse_ws/devel/setup.bash                # Source the ROS workspace
multiverse_server                                    # Start the multiverse server
roslaunch mujoco_sim multi_mujoco_sim_ref.launch     # Start 4 instances of MuJoCo
```
