.. _tutorial_6:

Tutorial 6: Write your own connector - Part 1
=============================================

Introduction
------------

In this tutorial, we will cover how to write your own Multiverse Connector to interact with the Multverse Server.
According to the Multiverse architecture, the Multiverse Connector is responsible for handling the communication between the Multiverse Server and the simulators.
It is implemented as a Python class that inherits from the Python binding of the Multiverse Client, which is written in C++.
In the following steps, you will write a simple Multiverse Connector in Python.

Getting Started
---------------

1. Create a new Python file `my_connector.py` anywhere in your system and open it in your favorite text editor. This file will contain the implementation of your Multiverse Connector.

2. Make sure the path `<path/to/Multiverse>/multiverse/lib/dist-packages` is in your `PYTHONPATH`. This path contains the Python binding of the Multiverse Client.

Write the Multiverse Connector
------------------------------

3. Add the following lines to the Python file to import the necessary modules:

.. code-block:: python

    from multiverse_client_py import MultiverseClient, MultiverseMetaData, SocketAddress

4. Add the following lines to the Python file to define the Multiverse Connector class:

.. code-block:: python

    class MyConnector(MultiverseClient):
        def __init__(self, client_addr: SocketAddress, multiverse_meta_data: MultiverseMetaData) -> None:
            super().__init__(client_addr, multiverse_meta_data)

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

5. Create an instance of the Multiverse Connector and run it:

.. code-block:: python

    if __name__ == "__main__":
        multiverse_meta_data = MultiverseMetaData(
            world_name="my_world",
            length_unit="m",
            angle_unit="rad",
            mass_unit="kg",
            time_unit="s",
            handedness="rhs",
        )
        client_addr = SocketAddress(port="5000")
        my_connector = MyConnector(client_addr=client_addr,
                                   multiverse_meta_data=multiverse_meta_data)
        my_connector.run()
        my_connector.stop()

6. Save the Python file and you are ready to run your Multiverse Connector.

Running the Multiverse Connector
--------------------------------

7. Open a terminal and start the Multiverse Server by running the following command:

.. code-block:: console

    multiverse_server

8. Open another terminal and start the Multiverse Connector by running the following command:

.. code-block:: console

    python my_connector.py

9. The Multiverse Connector will connect to the Multiverse Server and terminate. 
You will see the following output in the terminal:

.. code-block:: console

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

.. code-block:: console

    python my_connector.py 

    INFO: [Client 5000] Start MyConnector5000.
    INFO: Start running the client.
    [Client 5000] Sending request tcp://127.0.0.1:5000 to tcp://127.0.0.1:7000.
    [Client 5000] Sent request tcp://127.0.0.1:5000 to tcp://127.0.0.1:7000.
    [Client 5000] Received response tcp://127.0.0.1:5000 from tcp://127.0.0.1:7000.
    [Client 5000] Opened the socket tcp://127.0.0.1:5000.
    [Client 5000] Start.
    [Client 5000] Closing the socket tcp://127.0.0.1:5000.

Sending Data to the Multiverse Server
-------------------------------------

To successfully send data to the Multiverse Server, you need to define the `request_meta_data` and send it to the server.
The server will respond with the `response_meta_data`, indicating that the server understands the request and the connection can be established.
Once the connection is established, you can send data to the server by populating the `send_data` in the order specified by the `response_meta_data`.

10. Modify the code in the main part to send the request meta data to the server:

.. code-block:: python

    multiverse_meta_data = MultiverseMetaData(
        world_name="my_world",
        simulation_name="my_simulation",
        length_unit="m",
        angle_unit="rad",
        mass_unit="kg",
        time_unit="s",
        handedness="rhs",
    )
    client_addr = SocketAddress(port="5000")
    my_connector = MyConnector(client_addr=client_addr,
                               multiverse_meta_data=multiverse_meta_data)
    my_connector.run()

    my_connector.request_meta_data["send"] = {}
    my_connector.request_meta_data["send"]["my_object"] = [
        "position",
        "quaternion"
    ]
    my_connector.send_and_receive_meta_data()

    my_connector.stop()

11. Save the Python file and run the step 8 again. You will see the following output in the terminal:

.. code-block:: console

    python my_connector.py 

    INFO: [Client 5000] Start MyConnector5000.
    INFO: Start running the client.
    [Client 5000] Sending request tcp://127.0.0.1:5000 to tcp://127.0.0.1:7000.
    [Client 5000] Sent request tcp://127.0.0.1:5000 to tcp://127.0.0.1:7000.
    [Client 5000] Received response tcp://127.0.0.1:5000 from tcp://127.0.0.1:7000.
    [Client 5000] Opened the socket tcp://127.0.0.1:5000.
    [Client 5000] Start.
    INFO: Sending request meta data: {'meta_data': {'angle_unit': 'rad', 'handedness': 'rhs', 'length_unit': 'm', 'mass_unit': 'kg', 'simulation_name': 'my_simulation', 'time_unit': 's', 'world_name': 'my_world'}, 'send': {'my_object': ['position', 'quaternion']}, 'receive': {}}
    INFO: Received response meta data: {'meta_data': {'angle_unit': 'rad', 'handedness': 'rhs', 'length_unit': 'm', 'mass_unit': 'kg', 'simulation_name': 'my_simulation', 'time_unit': 's', 'world_name': 'my_world'}, 'send': {'my_object': {'position': [None, None, None], 'quaternion': [None, None, None, None]}}, 'time': 0}
    [Client 5000] Closing the socket tcp://127.0.0.1:5000.

As you can see, the Multiverse Connector successfully sent the request meta data to the server and received the response meta data from the server.
The `None` values in the response meta data indicate that the data is new and has not been sent yet.
Now we can send data to the server by populating the `send_data` in the order specified by the `response_meta_data`.
The `time` field in the response meta data indicates the current time in the simulation.
When you send data to the server, make sure to set the first value of the `send_data` to the current time (non-zero), if it's zero, all simulations in the same world will be reset.

12. Modify the code in the main part to send data to the server:

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
        client_addr = SocketAddress(port="5000")
        my_connector = MyConnector(client_addr=client_addr,
                                multiverse_meta_data=multiverse_meta_data)
        my_connector.run()

        my_connector.request_meta_data["send"] = {}
        my_connector.request_meta_data["send"]["my_object"] = [
            "position",
            "quaternion"
        ]
        my_connector.send_and_receive_meta_data()

        sim_time = my_connector.sim_time # The current simulation time
        my_object_pos = [1.0, 2.0, 3.0]
        my_object_quat = [0.0, 0.0, 0.0, 1.0]

        my_connector.send_data = [sim_time] + my_object_pos + my_object_quat # The send_data to the correct order
        my_connector.send_and_receive_data()

        my_connector.stop()

13. Save the Python file and run the step 8 again. You will see the following output in the terminal:

.. code-block:: console

    python my_connector.py

    INFO: [Client 5000] Start MyConnector5000.
    INFO: Start running the client.
    [Client 5000] Sending request tcp://127.0.0.1:5000 to tcp://127.0.0.1:7000.
    [Client 5000] Sent request tcp://127.0.0.1:5000 to tcp://127.0.0.1:7000.
    [Client 5000] Received response tcp://127.0.0.1:5000 from tcp://127.0.0.1:7000.
    [Client 5000] Opened the socket tcp://127.0.0.1:5000.
    [Client 5000] Start.
    INFO: Sending request meta data: {'meta_data': {'angle_unit': 'rad', 'handedness': 'rhs', 'length_unit': 'm', 'mass_unit': 'kg', 'simulation_name': 'my_simulation', 'time_unit': 's', 'world_name': 'my_world'}, 'send': {'my_object': ['position', 'quaternion']}, 'receive': {}}
    INFO: Received response meta data: {'meta_data': {'angle_unit': 'rad', 'handedness': 'rhs', 'length_unit': 'm', 'mass_unit': 'kg', 'simulation_name': 'my_simulation', 'time_unit': 's', 'world_name': 'my_world'}, 'send': {'my_object': {'position': [None, None, None], 'quaternion': [None, None, None, None]}}, 'time': 0}
    INFO: Sending data: [0.010332822799682617, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]
    [Client 5000] Starting the communication (send: [7 - 0 - 0], receive: [0 - 0 - 0]).
    INFO: Received data: [0.010332822799682617]
    [Client 5000] Closing the socket tcp://127.0.0.1:5000.

As you can see, the Multiverse Connector successfully sent the data to the server and received the data as the current world time from the server.
The line `[Client 5000] Starting the communication (send: [7 - 0 - 0], receive: [0 - 0 - 0])` indicates that the size of the data from the server and the client is correct (in this case, the client want to send 7 double, 0 uint8 and 0 uint16 and receive 0 double, 0 uint8 and 0 uint16 excluding time).

Receiving Data from the Multiverse Server
-----------------------------------------

To successfully receive data from the Multiverse Server, same as sending data, you need to define the `receive` field `request_meta_data` and send it to the server.
If the server understands the request and the data is available, the server will respond with the `response_meta_data`.
If the data is unavailable, the server will wait for the data to be available and the client will be blocked until the data is sent.
So to make sure the client is not blocked, you need to send the data to the server first.
Therefore we will continue from the step 12.

14. Modify the code in the main part to receive data from the server:

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
        client_addr = SocketAddress(port="5000")
        my_connector = MyConnector(client_addr=client_addr,
                                multiverse_meta_data=multiverse_meta_data)
        my_connector.run()

        my_connector.request_meta_data["send"] = {}
        my_connector.request_meta_data["send"]["my_object"] = [
            "position",
            "quaternion"
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

15. Save the Python file and run the step 8 again. You will see the following output in the terminal:

.. code-block:: console

    python my_connector.py

    ...
    INFO: Sending data: [0.016848087310791016]
    [Client 5000] Starting the communication (send: [0 - 0 - 0], receive: [7 - 0 - 0]).
    INFO: Received data: [0.016848087310791016, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]
    [Client 5000] Closing the socket tcp://127.0.0.1:5000.

As you can see, the Multiverse Connector successfully received the data from the server.

**Tip:** If you don't know about the objects and object attributes in the world, send an empty string in the `receive` field of `request_meta_data` to the server and the server will respond with the available objects and their attributes.
For example:

.. code-block:: python

    # To get the all available objects and their attributes
    my_connector.request_meta_data["receive"][""] = [""] 

    # To get the available attributes of the object my_object
    my_connector.request_meta_data["receive"]["my_object"] = [""]

    # To get the position of all available objects
    my_connector.request_meta_data["receive"][""] = ["position"]

Conclusion
----------

Congratulations! You have successfully written your own Multiverse Connector in Python. 
In this tutorial, you learned how to define the Multiverse Connector class, send and receive meta data, and send and receive data to and from the Multiverse Server. 
You also learned how to run the Multiverse Connector and interact with the Multiverse Server.

Next Steps
----------

- Extend the Multiverse Connector to interact with other Multiverse Clients through the Multiverse Server.