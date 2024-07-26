Welcome to Multiverse's documentation!
======================================

Multiverse is a simulation framework designed to integrate multiple advanced physics engines such as `MuJoCo <https://mujoco.readthedocs.io>`_, `Project Chrono <https://projectchrono.org>`_, and `SOFA <https://www.sofa-framework.org>`_ along with various photo-realistic graphics engines like `Unreal Engine <https://www.unrealengine.com>`_ and `Omniverse <https://developer.nvidia.com/omniverse>`_. Additionally, Multiverse provides the capability to generate knowledge graphs dynamically during runtime.

.. note::

   This project is under active development.

Introduction
------------

The Multiverse software is built with three main pillars: the Multiverse Server/Client, Multiverse Parser, and Multiverse Knowledge. Each pillar operates independently, making separate installation and operation possible. To simplify the launching and visualizing of clusters, Multiverse includes Multiverse Launch and Multiverse View. The software runs on both Linux and Windows, with seamless communication across systems via TCP socket. 

Multiverse Server-Client
~~~~~~~~~~~~~~~~~~~~~~~~

A distinctive feature of Multiverse is its capability to integrate various physics and graphics engines and operate them simultaneously. This capability is grounded in a star architecture, with the server at the core, coordinating data sharing among different client simulators. The framework treats each object cluster in a simulator as a Port-Hamiltonian system, facilitating input-output interactions with other simulators as effort/flow pairs. This ensures stability and feasibility through energy conservation. In the very first initial version of Multiverse, each client's data is stored as a map :math:`n \rightarrow v`, where :math:`n` denotes an attribute name in string format, and :math:`v` is the corresponding attribute value stored as an array of floats. 

.. image:: _static/images/MultiverseServerClient.png
   :width: 1200

In this framework, "simulator" encompasses a broad range of entities, including not only physics engines and graphics engines but also VR headsets that interpret objects geometry, position, and orientation to render them, and controllers that compute forces and torques based on physics data. It can also include Python scripts to manipulate and analyze this data. The possibilities are endless, making it adaptable to many different scenarios. The Multiverse Server-Client is the essential foundation that makes this versatility possible.

Please follow this `tutorial <https://multiverseframework.readthedocs.io/en/latest/tutorials.html>`_ to familiarize yourself with the Multiverse Server-Client.

Multiverse Parser
~~~~~~~~~~~~~~~~~

One might have concern "Oh but different simulators, each using its own scene description format, can't connect with each other if they aren't compatible, right?". The answer is...

.. raw:: html

    <img src="https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExOWtpdHUzM2R2NGU0dnZidjVhdnNmNjhodHEydTRheDI4eTE3MzF3cSZlcD12MV9naWZzX3NlYXJjaCZjdD1n/3oFzmkkwfOGlzZ0gxi/giphy.gif" alt="Animated GIF">

This is by far one of the most painful problems in the robotics community in particular and in society in general, everyone claims they are the best and compelling others to adopt their "almighty" solution. Although a solution may initially be considered the best, it's only a matter of time before it's surpassed by another. When this transition occurs, all software tools reliant on the outdated solution become obsolete and require significant effort to update. One good example is the URDF format, which is widely used to describe robots, yet lacks sufficient features to be useful beyond Gazebo.

Luckily, there's a solution to this problem. If there were a Nobel Peace Prize for Scene Description Formats, the Universal Scene Description Format (USD) would be the clear winner. USD, as its name implies, acts as a translation medium for different formats, much like the US Dollar in global trade (haha). The key to this versatility lies in USD's extensibility through custom schemas. A schema functions as a container, adding attributes to a USD prim, allowing any new format to be converted into USD with custom schemas, thus enabling bi-directional conversion.

And in the Multiverse Parser component of the Multiverse framework, USD serves as the translation medium, facilitating not only the seamless conversion of different scene descriptions, but also optimizing them by removing redundant data. This includes visual meshes that are irrelevant to physics engines, and collision meshes that are invisible to graphics engines. You have the meshes but you don't have the proper dynamic properties? Not a problem, Multiverse Parser can compute these properties based on the geometry.

So, let yourself be convinced by exploring this `tutorial <https://binder.intel4coro.de/v2/gh/Multiverse-Framework/Multiverse-Docker/main>`_ on the Multiverse Parser.

Multiverse Knowledge
~~~~~~~~~~~~~~~~~~~~

Running a physics simulation is fun and enjoyable for humans, particularly in video games, films, and education. If you think about it, the reason it's fun is because you can see what you imagine. State-of-the-art VR technologies also help immerse you in a world of imagination, where you can do whatever you want, even in unrealistic places such as the middle of the sun or in a fancy car surrounded by beautiful partners. Imagination is a powerful feature that has led us to where we are now, and in AI and robotics, we aim to enable this feature in robots.

The world is changing, moving from 2D to 3D data. While classification and semantic labeling of images and videos are commonplace, they remain in 2D. The transition to 3D data is not trivial. First, AI demands vast amounts of data, and 3D data is scarcer than 2D, with its capture being more challenging. While everyone has a smartphone and can take photos, how many people have a VR headset and want to scan an object in 3D? Second, 3D data offers layering, making the hidden parts of an object visible, increasing the data size compared to 2D's single layer from a camera capture. Third, data incompatibility further complicates the process.

The Multiverse Knowledge addresses this by taking the first step in understanding scene descriptions and translating them into a machine-readable scene graph. By converting different scene descriptions into USD using the Multiverse Parser, the Multiverse Knowledge transforms the USD scene into a semantic one, adapting custom schemas that support semantic labeling of each USD prim with ontological concepts from various ontologies. This process can be done manually or automatically using the Multiverse View. In summary, given a scene description in any format and some ontologies, the Multiverse Knowledge converts it into a knowledge graph that can be queried. Extending the example of imagination, you can now even interact with the 3D environment verbally.

Please try out some examples in this `tutorial <https://binder.intel4coro.de/v2/gh/Multiverse-Framework/Multiverse-Docker/main>`_ on the Multiverse Knowledge.

Contents
--------

.. toctree::
   :maxdepth: 1

   installation
   tutorials
   api
