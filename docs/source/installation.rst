.. _installation:

Installation
============

Prerequisite
------------
To install Multiverse with all connectors, ensure you have at least 50 GB of free space and Git installed.

Linux (Ubuntu >= 20.04)
-----------------------

Open a terminal and run the following commands:

.. code-block:: console

    ./install.sh                    # Install all prerequisites (sudo required)
    ./build_third_parties.sh        # Build the dependencies
    ./build_multiverse.sh           # Build the software
    ./build_multiverse_ws.sh        # Build the ROS workspace (only for Ubuntu 20.04)
    ./build_multiverse_ws2.sh       # Build the ROS2 workspace (for Ubuntu >= 20.04)

Windows 11
----------

Run Windows PowerShell as Administrator and execute the following scripts. Restart PowerShell before running each script to refresh the environment paths:

.. code-block:: console

    install.bat                     # Install all prerequisites (Administrator required)
    build_third_parties.bat         # Build the dependencies
    build_multiverse.bat            # Build the software
    build_multiverse_ws2.bat        # Build the ROS2 workspace

Testing the Installation
------------------------

To test the installation, open a Terminal (in Ubuntu) or Windows PowerShell as Administrator (in Windows 11) in the Multiverse folder and execute the following command:

.. code-block:: console

    multiverse_launch multiverse/resources/muv/table_with_bowling.xml

This command will start an interactive simulation that combines four different instances of MuJoCo. You can control a cube using the middle mouse button and interact with other objects in the simulation.
