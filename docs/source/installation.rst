.. _installation:

Installation
============

Prerequisite
------------
To install Multiverse with all connectors, ensure you have at least 50 GB of free space and Git installed.

Start by cloning this `repository <https://github.com/Multiverse-Framework/Multiverse>`_ and navigating to the Multiverse folder.

.. code-block:: console

    git clone https://github.com/Multiverse-Framework/Multiverse --depth 1

Linux (Ubuntu >= 20.04)
-----------------------

Open a terminal and run the following commands:

.. code-block:: console

    ./install.sh                                                                    # Install all prerequisites
    ./build_third_parties.sh #--excludes cmake blender usd mujoco pybind11 isaaclab # Build the dependencies with optional exclusions
    ./build_multiverse.sh #--only-src / --only-modules connectors parser knowledge  # Build the software with optional inclusions
    ./build_multiverse_ws.sh                                                        # Build the ROS workspace (only for Ubuntu 20.04)
    ./build_multiverse_ws2.sh                                                       # Build the ROS2 workspace (for Ubuntu >= 20.04)

Windows 11
----------

Run Windows PowerShell as Administrator and execute the following scripts. Restart PowerShell before running each script to refresh the environment paths:

.. code-block:: console

    install.bat                     # Install all prerequisites (Administrator required)
    build_third_parties.bat         # Build the dependencies
    build_multiverse.bat            # Build the software
    build_multiverse_ws2.bat        # Build the ROS2 workspace (only for Windows 10)

Testing the Installation
------------------------

To test the installation, open a Terminal (in Ubuntu) or Windows PowerShell as Administrator (in Windows 11) in the Multiverse folder and execute the following command:

.. code-block:: console

    multiverse_launch multiverse/resources/muv/table_with_bowling.xml

This command will start an interactive simulation that combines four different instances of MuJoCo. You can control a cube using the middle mouse button and interact with other objects in the simulation.
