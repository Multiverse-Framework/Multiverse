.. _tutorial_1:

Tutorial 1: Getting Started with Multiverse - A simple Multiverse Connector using Python
========================================================================================

Introduction
------------

Welcome to the first tutorial on Multiverse! In this tutorial, we will cover the basics of getting started with Multiverse and creating your first Multiverse Connector using a Python script.

.. important::

   **Zero-install experience:** You don’t need to install anything system-wide.
   Just **download the prebuilt package** and run it. No dependencies, no setup.

In this tutorial, you will:

- `Start the Multiverse Server tutorial_1.html#getting-started`
- Implement a simple Multiverse Connector in Python
- Run the Multiverse Connector and connect it to the Multiverse Server

Key Concepts
------------

.. image:: ../_static/images/MultiverseConnector.png
   :width: 1000

- **Multiverse Server**: The central hub that coordinates communication between simulators
  and controllers in real time.
- **Multiverse Client**: A base program (C++ with Python bindings) that connects to the
  Multiverse Server to send and receive data.
- **Multiverse Connector**: A specific implementation of a Multiverse Client that connects to a particular simulator or controller.

Getting Started
---------------

1. Clone **or download** the following repositories to your local machine:

   .. code-block:: bash

      git clone https://github.com/Multiverse-Framework/Multiverse-ServerClient.git
      git clone https://github.com/Multiverse-Framework/Multiverse-ClientPy.git

   .. note::

      **No Git?** Open each repository on GitHub, click **Code → Download ZIP**,
      and extract the archives locally.

Start the Multiverse Server
---------------------------

2. Start the Multiverse Server by running the following command in a terminal:

.. code-block:: bash

    cd <path/to/Multiverse-ServerClient>/bin
    ./multiverse_server

The Multiverse Server should now be running and waiting for Multiverse Clients to connect. The default port is `7000`.
The following output should appear in the terminal:

.. code-block:: text

    Start Multiverse Server...
    [Server] Create server socket tcp://*:7000
    [Server] Waiting for request...

Implement a simple Multiverse Connector in Python
-------------------------------------------------

3. Create a new Python script named `my_connector.py` in the `Multiverse-ClientPy` directory with the following content:

.. code-block:: python

    from multiverse_client_py import MultiverseClient, MultiverseMetaData

    class MyConnector(MultiverseClient):
        def __init__(self, port: str, multiverse_meta_data: MultiverseMetaData) -> None:
            super().__init__(port, multiverse_meta_data)

        def loginfo(self, message: str) -> None:
            print(f"INFO: {message}")

        def logwarn(self, message: str) -> None:
            print(f"WARN: {message}")

        def _run(self) -> None:
            self.loginfo("Start running the client.")
            self._connect_and_start()

        def send_and_receive_meta_data(self) -> None:
            self.loginfo("Sending request meta data: " + str(self.request_meta_data))
            self._communicate(True)
            self.loginfo("Received response meta data: " + str(self.response_meta_data))

        def send_and_receive_data(self) -> None:
            self.loginfo("Sending data: " + str(self.send_data))
            self._communicate(False)
            self.loginfo("Received data: " + str(self.receive_data))

    if __name__ == "__main__":
        multiverse_meta_data = MultiverseMetaData(
            world_name="my_world",
            simulation_name="my_simulation",
            length_unit="m",
            angle_unit="rad",
            mass_unit="kg",
            time_unit="s",
            handedness="rhs",
        )
        my_connector = MyConnector(port="5000", multiverse_meta_data=multiverse_meta_data)
        my_connector.run()
        my_connector.stop()

Save the Python file and you are ready to run your Multiverse Connector.

Run the Multiverse Connector and connect it to the Multiverse Server
--------------------------------------------------------------------

4. Run the Multiverse Connector by executing the following command in a new terminal:

.. code-block:: bash

    cd <path/to/Multiverse-ClientPy>
    python3 my_connector.py

You should see the following output in the terminal of Multiverse Server:

.. code-block:: text

    multiverse_server

    Start Multiverse Server...
    [Server] Create server socket tcp://*:7000
    [Server] Waiting for request...
    [Server] Received request to open socket tcp://127.0.0.1:5000.
    [Server] Sending response to open socket tcp://127.0.0.1:5000.
    [Server] Sent response to open socket tcp://127.0.0.1:5000.
    [Server] Waiting for request...
    [Server] Bind to socket tcp://127.0.0.1:5000.
    [Server] Received close signal at socket tcp://127.0.0.1:5000.

And the following output should appear in the terminal of your Multiverse Connector:

.. code-block:: text

    python3 my_connector.py

    INFO: [Client 5000] Start MyConnector5000.
    INFO: Start running the client.
    [Client 5000] Sending request tcp://127.0.0.1:5000 to tcp://127.0.0.1:7000.
    [Client 5000] Sent request tcp://127.0.0.1:5000 to tcp://127.0.0.1:7000.
    [Client 5000] Received response tcp://127.0.0.1:5000 from tcp://127.0.0.1:7000.
    [Client 5000] Opened the socket tcp://127.0.0.1:5000.
    [Client 5000] Start.
    [Client 5000] Closing the socket tcp://127.0.0.1:5000.

Sending Data and Receiving Data
-------------------------------

To successfully send data to the Multiverse Server, you need to define the ``request_meta_data`` and send it to the Multiverse Server. The Multiverse Server will respond with the ``response_meta_data``, indicating that the Multiverse Server understands the request and the connection can be established. Once the connection is established, you can send data to the server by populating the ``send_data`` in the order specified by the ``response_meta_data``.

1. Modify the code in the main part to send the ``request_meta_data`` to the server:

.. code-block:: python

    if __name__ == "__main__":
        multiverse_meta_data = MultiverseMetaData(
            world_name="my_world",
            simulation_name="my_simulation",
            length_unit="m",
            angle_unit="rad",
            mass_unit="kg",
            time_unit="s",
            handedness="rhs",
        )
        my_connector = MyConnector(port="5000", multiverse_meta_data=multiverse_meta_data)
        my_connector.run()

        my_connector.request_meta_data["send"] = {}
        my_connector.request_meta_data["send"]["my_object"] = [
            "quaternion",
            "position",
        ]
        my_connector.send_and_receive_meta_data()

        my_connector.stop()

Save the Python file and run the Multiverse Connector again. You should see the following output in the terminal of your Multiverse Connector:

.. code-block:: text

    python3 my_connector.py

    INFO: [Client 5000] Start MyConnector5000.
    INFO: Start running the client.
    [Client 5000] Sending request tcp://127.0.0.1:5000 to tcp://127.0.0.1:7000.
    [Client 5000] Sent request tcp://127.0.0.1:5000 to tcp://127.0.0.1:7000.
    [Client 5000] Received response tcp://127.0.0.1:5000 from tcp://127.0.0.1:7000.
    [Client 5000] Opened the socket tcp://127.0.0.1:5000.
    [Client 5000] Start.
    INFO: Sending request meta data: {'meta_data': {'angle_unit': 'rad', 'handedness': 'rhs', 'length_unit': 'm', 'mass_unit': 'kg', 'simulation_name': 'my_simulation', 'time_unit': 's', 'world_name': 'my_world'}, 'send': {'my_object': ['quaternion', 'position']}, 'receive': {}}
    INFO: Received response meta data: {'meta_data': {'angle_unit': 'rad', 'handedness': 'rhs', 'length_unit': 'm', 'mass_unit': 'kg', 'simulation_name': 'my_simulation', 'time_unit': 's', 'world_name': 'my_world'}, 'send': {'my_object': {'position': [None, None, None], 'quaternion': [None, None, None, None]}}, 'time': 0}
    [Client 5000] Closing the socket tcp://127.0.0.1:5000.

As you can see, the Multiverse Connector successfully sent the request meta data to the server and received the response meta data from the server. The None values in the response meta data indicate that the data is new and has not been sent yet. Now we can send data to the server by populating the ``send_data`` in the order specified by the ``response_meta_data``. The time field in the response meta data indicates the current time in the simulation. 

When you send data to the server, make sure to set the first value of the send_data to the current time (non-zero), if it’s zero, all simulations in the same world will be reset.

2. Modify the code in the main part to send the ``send_data`` to the server:

.. code-block:: python

    if __name__ == "__main__":
        multiverse_meta_data = MultiverseMetaData(
            world_name="my_world",
            simulation_name="my_simulation",
            length_unit="m",
            angle_unit="rad",
            mass_unit="kg",
            time_unit="s",
            handedness="rhs",
        )
        my_connector = MyConnector(port="5000", multiverse_meta_data=multiverse_meta_data)
        my_connector.run()

        my_connector.request_meta_data["send"] = {}
        my_connector.request_meta_data["send"]["my_object"] = [
            "quaternion",
            "position",
        ]
        my_connector.send_and_receive_meta_data()

        sim_time = my_connector.sim_time # The current simulation time
        my_object_pos = [1.0, 2.0, 3.0]
        my_object_quat = [0.0, 0.0, 0.0, 1.0]

        my_connector.send_data = [sim_time] + my_object_pos + my_object_quat # The send_data to the order specified by the response_meta_data
        my_connector.send_and_receive_data()

        my_connector.stop()

Save the Python file and run the Multiverse Connector again. You should see the following output in the terminal of your Multiverse Connector:

.. code-block:: text

    python3 my_connector.py

    INFO: [Client 5000] Start MyConnector5000.
    INFO: Start running the client.
    [Client 5000] Sending request tcp://127.0.0.1:5000 to tcp://127.0.0.1:7000.
    [Client 5000] Sent request tcp://127.0.0.1:5000 to tcp://127.0.0.1:7000.
    [Client 5000] Received response tcp://127.0.0.1:5000 from tcp://127.0.0.1:7000.
    [Client 5000] Opened the socket tcp://127.0.0.1:5000.
    [Client 5000] Start.
    INFO: Sending request meta data: {'meta_data': {'angle_unit': 'rad', 'handedness': 'rhs', 'length_unit': 'm', 'mass_unit': 'kg', 'simulation_name': 'my_simulation', 'time_unit': 's', 'world_name': 'my_world'}, 'send': {'my_object': ['quaternion', 'position']}, 'receive': {}}
    INFO: Received response meta data: {'meta_data': {'angle_unit': 'rad', 'handedness': 'rhs', 'length_unit': 'm', 'mass_unit': 'kg', 'simulation_name': 'my_simulation', 'time_unit': 's', 'world_name': 'my_world'}, 'send': {'my_object': {'position': [None, None, None], 'quaternion': [None, None, None, None]}}, 'time': 0}
    INFO: Sending data: [0.010332822799682617, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]
    [Client 5000] Starting the communication (send: [7 - 0 - 0], receive: [0 - 0 - 0]).
    INFO: Received data: [0.010332822799682617]
    [Client 5000] Closing the socket tcp://127.0.0.1:5000.

As you can see, the Multiverse Connector successfully sent the data to the server and received the data as the current world time from the server. 

- ``[Client 5000] Starting the communication (send: [7 - 0 - 0], receive: [0 - 0 - 0]).``  
  Using the ``[float - uint8 - uint16]`` layout, the client sends **1 float** (current
  simulation time) plus **7 floats** (``3`` position components and ``4`` quaternion
  components). It expects a reply of **1 float** (the current world time). No ``uint8`` or
  ``uint16`` fields are used in either direction.

To successfully receive data from the Multiverse Server, same as sending data, you need to define the receive field ``request_meta_data`` and send it to the server. 

If the Multiverse Server understands the request and the data is available, the ultiverse Server will respond with the ``response_meta_data``. 

If the data is unavailable, the Multiverse Server will wait for the data to be available and the Multiverse Client will be blocked until the data is sent. 

So to make sure the ultiverse Client is not blocked, you need to send the data to the ultiverse Server first. Therefore we will continue to receive data after sending data.

3. Modify the code in the main part to receive data from the ultiverse Server:

.. code-block:: python

    if __name__ == "__main__":
        multiverse_meta_data = MultiverseMetaData(
            world_name="my_world",
            simulation_name="my_simulation",
            length_unit="m",
            angle_unit="rad",
            mass_unit="kg",
            time_unit="s",
            handedness="rhs",
        )
        my_connector = MyConnector(port="5000", multiverse_meta_data=multiverse_meta_data)
        my_connector.run()

        my_connector.request_meta_data["send"] = {}
        my_connector.request_meta_data["send"]["my_object"] = [
            "quaternion",
            "position",
        ]
        my_connector.send_and_receive_meta_data()

        sim_time = my_connector.sim_time # The current simulation time
        my_object_pos = [1.0, 2.0, 3.0]
        my_object_quat = [0.0, 0.0, 0.0, 1.0]

        my_connector.send_data = [sim_time] + my_object_pos + my_object_quat # The send_data to the correct order
        my_connector.send_and_receive_data()

        # Change the request meta data to receive the position and quaternion of my_object

        my_connector.request_meta_data["send"] = {}
        my_connector.request_meta_data["receive"] = {}
        my_connector.request_meta_data["receive"]["my_object"] = [
            "position",
            "quaternion"
        ]
        my_connector.send_and_receive_meta_data()

        sim_time = my_connector.sim_time # The current simulation time
        my_connector.send_data = [sim_time]
        my_connector.send_and_receive_data()

        my_connector.stop()

Save the Python file and run the Multiverse Connector again. You should see the following output in the terminal of your Multiverse Connector:

.. code-block:: text

    python my_connector.py

    ...
    INFO: Sending data: [0.016848087310791016]
    [Client 5000] Starting the communication (send: [0 - 0 - 0], receive: [7 - 0 - 0]).
    INFO: Received data: [0.016848087310791016, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]
    [Client 5000] Closing the socket tcp://127.0.0.1:5000.

As you can see, the Multiverse Connector successfully received the data from the Multiverse Server.

**Tip**: If you don’t know about the objects and object attributes in the world, send an empty string in the receive field of ``request_meta_data`` to the Multiverse Server and the Multiverse Server will respond with the available objects and their attributes. For example:

.. code-block:: python

    # To get the all available objects and their attributes
    my_connector.request_meta_data["receive"][""] = [""]

    # To get the available attributes of the object my_object
    my_connector.request_meta_data["receive"]["my_object"] = [""]

    # To get the position of all available objects
    my_connector.request_meta_data["receive"][""] = ["position"]

Conclusion
----------

Congratulations! You have successfully written your own Multiverse Connector in Python. In this tutorial, you learned how to define the Multiverse Connector class, send and receive meta data, and send and receive data to and from the Multiverse Server. You also learned how to run the Multiverse Connector and interact with the Multiverse Server.

Next Steps
----------

- Use Multiverse Connector in MuJoCo as a plugin: :ref:`tutorial_2`
