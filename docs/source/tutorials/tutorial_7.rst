.. _tutorial_7:

Tutorial 7: Write your own connector - Part 2
=============================================

Introduction
------------

In this tutorial, we will cover how to extend the Multiverse connector to interact with other Multiverse Clients via the Multiverse Server,
such as Spawning, Destroying objects, Changing the properties of objects and Calling API.
As such, we will start with a simple scenario in MuJoCo.

Getting Started
---------------

1. Open a terminal and run the scenario:

.. code-block:: console

    multiverse_launch <path/to/Multiverse>/multiverse/resources/muv/empty.muv

2. Continue the step 5 from the previous :ref:`tutorial_6` to create a new connector.

Spawn an object
---------------

3. Add the following lines to the connector to spawn an object:

.. code-block:: python

    if __name__ == "__main__":
        multiverse_meta_data = MultiverseMetaData(
            world_name="world",
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

        # Spawn an object called milk_box in the simulation called empty_simulation

        object_name = "milk_box"
        object_pos = [0.0, 0.0, 5.0]
        object_quat = [0.707, 0.0, 0.707, 0.0]

        my_connector.request_meta_data["meta_data"]["simulation_name"] = "empty_simulation"
        my_connector.request_meta_data["send"] = {}
        my_connector.request_meta_data["send"]["milk_box"] = ["position", "quaternion"]
        my_connector.send_and_receive_meta_data()
        my_connector.send_data = [my_connector.sim_time] + object_pos + object_quat
        my_connector.send_and_receive_data()

        my_connector.stop()

4. Save the connector and run it:

.. code-block:: console

    python my_connector.py

The object milk_box will be spawned in the MuJoCo simulation empty_simulation. 
The path of the object is found in the resources defined in the MUV file.
If the object is not found in the resources, the object will not be spawned.
After spawning the object, the Multiverse Client running the MuJoCo simulation will send the position and quaternion of the new object `milk_box` to the Multiverse Server,
as specified in the request_meta_data of the requested connector.

Destroy an object
-----------------

5. Add the following lines to the connector to destroy the object after 5 seconds:

.. code-block:: python

    multiverse_meta_data = MultiverseMetaData(
            world_name="world",
            simulation_name="my_simulation",
            length_unit="m",
            angle_unit="rad",
            mass_unit="kg",
            time_unit="s",
            handedness="rhs",
        )
        my_connector = MyConnector(port="5000",
                                   multiverse_meta_data=multiverse_meta_data)
        my_connector.run()

        # Spawn an object called milk_box in the simulation called empty_simulation

        object_name = "milk_box"
        object_pos = [0.0, 0.0, 5.0]
        object_quat = [0.707, 0.0, 0.707, 0.0]

        my_connector.request_meta_data["meta_data"]["simulation_name"] = "empty_simulation"
        my_connector.request_meta_data["send"] = {}
        my_connector.request_meta_data["send"]["milk_box"] = ["position", "quaternion"]
        my_connector.send_and_receive_meta_data()
        my_connector.send_data = [my_connector.sim_time] + object_pos + object_quat
        my_connector.send_and_receive_data()

        # Destroy the object after 5 seconds

        import time
        time.sleep(5)

        my_connector.request_meta_data["meta_data"]["simulation_name"] = "empty_simulation"
        my_connector.request_meta_data["send"] = {}
        my_connector.request_meta_data["send"]["milk_box"] = []
        my_connector.request_meta_data["receive"]["milk_box"] = []
        my_connector.send_and_receive_meta_data()
        my_connector.send_data = [my_connector.sim_time]
        my_connector.send_and_receive_data()

        my_connector.stop()

6. Save the connector and run it:

.. code-block:: console

    python my_connector.py

The object milk_box will be destroyed in the MuJoCo simulation empty_simulation after 5 seconds.
After destroying the object, the Multiverse Client running the MuJoCo simulation will stop sending the position and quaternion of the object `milk_box` to the Multiverse Server,
as specified in the request_meta_data of the requested connector.

Change the properties of an object
----------------------------------

The modification of the properties of an object is similar to the spawning of an object.
The only difference is that the object already exists in the simulation.
If the object is not found in the simulation, the object will be spawned.
You can test it by excuting the step 3 two times, and the object will be updated with the new position and quaternion.

Call API
--------

7. After spawning an object, add the following lines to the connector to call the API:

.. code-block:: python

    if __name__ == "__main__":
        multiverse_meta_data = MultiverseMetaData(
            world_name="world",
            simulation_name="my_simulation",
            length_unit="m",
            angle_unit="rad",
            mass_unit="kg",
            time_unit="s",
            handedness="rhs",
        )
        my_connector = MyConnector(port="5000",
                                   multiverse_meta_data=multiverse_meta_data)
        my_connector.run()

        # Spawn an object called milk_box in the simulation called empty_simulation

        object_name = "milk_box"
        object_pos = [0.0, 0.0, 5.0]
        object_quat = [0.707, 0.0, 0.707, 0.0]

        my_connector.request_meta_data["meta_data"]["simulation_name"] = "empty_simulation"
        my_connector.request_meta_data["send"] = {}
        my_connector.request_meta_data["send"]["milk_box"] = ["position", "quaternion"]
        my_connector.send_and_receive_meta_data()
        my_connector.send_data = [my_connector.sim_time] + object_pos + object_quat
        my_connector.send_and_receive_data()

        # Call four APIs after spawning the object

        my_connector.request_meta_data["meta_data"]["simulation_name"] = "my_simulation"
        my_connector.request_meta_data["send"] = {}         # Clear the send data
        my_connector.request_meta_data["receive"] = {}      # Clear the receive data
        my_connector.request_meta_data["api_callbacks"] = {
            "empty_simulation": [
                {"exist": ["milk_box"]},                    # Check if the object exists
                {"get_rays": ["0 0 1", "0 0 5"]},           # Get the objects that hit by the ray from [0 0 1] to [0 0 5]
                {"is_mujoco": []},                          # Check if the simulator is MuJoCo
                {"something_else": ["param1", "param2"]},   # Call another API
            ]
        }
        my_connector.send_and_receive_meta_data()

        my_connector.stop()

8. Save the connector and run it:

.. code-block:: console

    python my_connector.py

You will see the results of the four APIs as follows:

.. code-block:: console

    INFO: [Client 5000] Start MyConnector5000.
    INFO: Start running the client.
    [Client 5000] Sending request tcp://127.0.0.1:5000 to tcp://127.0.0.1:7000.
    [Client 5000] Sent request tcp://127.0.0.1:5000 to tcp://127.0.0.1:7000.
    [Client 5000] Received response tcp://127.0.0.1:5000 from tcp://127.0.0.1:7000.
    [Client 5000] Opened the socket tcp://127.0.0.1:5000.
    [Client 5000] Start.
    INFO: Sending request meta data: {'meta_data': {'angle_unit': 'rad', 'handedness': 'rhs', 'length_unit': 'm', 'mass_unit': 'kg', 'simulation_name': 'empty_simulation', 'time_unit': 's', 'world_name': 'world'}, 'send': {'milk_box': ['position', 'quaternion']}, 'receive': {}}
    INFO: Received response meta data: {'meta_data': {'angle_unit': 'rad', 'handedness': 'rhs', 'length_unit': 'm', 'mass_unit': 'kg', 'simulation_name': 'empty_simulation', 'time_unit': 's', 'world_name': 'world'}, 'send': {'milk_box': {'position': [5.115300022826556e-18, 3.703495798793672e-18, 0.02989224457978379], 'quaternion': [0.7071067811865475, 4.130162651383842e-17, 0.7071067811865475, -2.5531090745860042e-17]}}, 'time': 1538.8849999705408}
    INFO: Sending data: [0.004271030426025391, 0.0, 0.0, 5.0, 0.707, 0.0, 0.707, 0.0]
    [Client 5000] Starting the communication (send: [7 - 0 - 0], receive: [0 - 0 - 0]).
    INFO: Received data: [1538.8999999705406]
    INFO: Sending request meta data: {'meta_data': {'angle_unit': 'rad', 'handedness': 'rhs', 'length_unit': 'm', 'mass_unit': 'kg', 'simulation_name': 'my_simulation', 'time_unit': 's', 'world_name': 'world'}, 'send': {}, 'receive': {}, 'api_callbacks': {'empty_simulation': [{'exist': ['milk_box']}, {'get_rays': ['0 0 1', '0 0 5']}, {'is_mujoco': []}, {'something_else': ['param1', 'param2']}]}}
    INFO: Received response meta data: {'api_callbacks_response': {'empty_simulation': [{'exist': ['yes']}, {'get_rays': ['milk_box 3.969843']}, {'is_mujoco': ['true']}, {'something_else': ['not implemented']}]}, 'meta_data': {'angle_unit': 'rad', 'handedness': 'rhs', 'length_unit': 'm', 'mass_unit': 'kg', 'simulation_name': 'my_simulation', 'time_unit': 's', 'world_name': 'world'}, 'time': 1538.9039999705406}

The connector will call the four APIs after spawning the object `milk_box` in the MuJoCo simulation empty_simulation.
The Multiverse Client running the MuJoCo simulation will send the results of the APIs to the Multiverse Server,
as specified in the request_meta_data of your requested connector.
Then the Multiverse Server will send the results to your requested connector.
The first API checks if the object `milk_box` exists in the simulation, the answer is `yes`.
The second API gets the objects that are hit by the ray from [0 0 1] to [0 0 5], the result is `milk_box 3.969843`, indicates that the object `milk_box` is hit by the ray at 3.969843 meters from the origin.
The third API checks if the simulator is MuJoCo, the answer is `true`.
The fourth API is not implemented, so the result is `not implemented`.

Conclusion
----------

Congratulations! You have successfully extended the Multiverse connector to interact with other Multiverse Clients via the Multiverse Server.
You have learned how to spawn an object, destroy an object, change the properties of an object, and call the API.