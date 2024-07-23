.. _tutorial_1:

Tutorial 1: Getting Started with Multiverse
===========================================

Introduction
------------

Welcome to the first tutorial on Multiverse! In this tutorial, we will cover the basics of getting started with Multiverse and creating your first project.

Getting Started
---------------

1. Create a new MUV file `my_project.muv` in `<path/to/Multiverse>/multiverse/resources/muv` and open it in your favorite text editor. This file will contain the configuration for your Multiverse project.

2. Add the following lines to the MUV file to specify the resources for your project:

    ```yaml
    resources: 
    - ../robots # Path to the robots directory, containing robot models in any formats
    - ../worlds # Path to the worlds directory, containing world models in any formats
    - ../objects # Path to the objects directory, containing object models in any formats
    ```

3. Add the following lines to the MUV file to define the worlds for your project:

    ```yaml
    worlds:
        my_world: # Name of the world
            rtf_desired: 1 # Real-time factor desired for the simulation
    ```

4. Add the following lines to the MUV file to define the simulation for your project:

    ```yaml
    simulations:
        my_simulation: # Name of the simulation
            simulator: mujoco # Simulator to use for the simulation
            world:
                name: my_world # Name of the world to use for the simulation defined in step 3
                path: floor.xml # Path to the world model file in the worlds directory defined in step 2
    ```

5. Save the MUV file, and you are ready to launch your first simulation using Multiverse.

    ```yaml
    resources:
    - ../robots
    - ../worlds
    - ../objects

    worlds:
        my_world:
            rtf_desired: 1

    simulations:
        my_simulation:
            simulator: mujoco
            world:
            name: my_world
    ```
    
    Run the Simulation

    ```bash
    multiverse_launch  `<path/to/Multiverse>/multiverse/resources/muv/my_project.muv`
    ```

The result should be a running simulation with the specified world and simulation parameters.

.. image:: _static/images/tutorials/tutorial_1_1.png
   :width: 1200

Conclusion
----------

Congratulations! You have successfully created your first project with Multiverse. In this tutorial, you learned how to define resources, worlds, and simulations in a MUV file and launch a simulation using the `multiverse_launch` command.

Next Steps
----------

- Run a simulation with different models and simulators.
