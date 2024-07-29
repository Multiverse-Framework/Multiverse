.. _tutorial_4:

Tutorial 4: Adding robots and objects to the simulation - Unreal Engine
=======================================================================

Introduction
------------

In this tutorial, we will cover how to add robots and objects to the Unreal Engine simulation in a world in Multiverse.

.. note::

   At the moment, this API in Unreal Engine simulator only supports parts of Multiverse functionalities. Support for the full Multiverse functionalities will be added in the future.

Getting Started
---------------

1. Create an Unreal Engine project and add to the `Plugins` folder the following plugins:

- `MultiverseConnector <https://github.com/Multiverse-Framework/Multiverse-UnrealEngine-Connector>`_
- `Meta XR Plugin v60 <https://developer.oculus.com/downloads/package/unreal-engine-5-integration/60.0>`_
- `Meta XR Platform v60 <https://developer.oculus.com/downloads/package/unreal-5-platform-sdk-plugin/60.0>`_

2. Build the Unreal Engine project and open it with Unreal Editor.

3. In the Unreal Editor, place the `MultiverseClientActor` actor in the scene.

Define Robots and Objects in the `MultiverseClientActor`
--------------------------------------------------------

4. In the `MultiverseClientActor` actor, add robots and objects to the simulation in the world.

5. Save the Unreal Engine project.

Running the Simulation
----------------------

.. note::

    Before running the simulation, make sure the Multiverse Server is running.

.. code-block:: bash

    multiverse_server

6. Run the Unreal Engine project.

At this point, you should see the robots and objects in the simulation in the world.

Conclusion
----------

Congratulations! You have successfully added robots and objects to the Unreal Engine simulation in Multiverse.

Next Steps
----------

- Deploy ROS as Multiverse Clients to interact with the simulation.