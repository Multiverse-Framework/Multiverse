.. _tutorial_5:

Tutorial 5: Connect with ROS
============================

Introduction
------------

In this tutorial, we will cover how to connect Multiverse with ROS (Robot Operating System) to interact with robots and objects in the simulation. 
For Ubuntu 20.04, both ROS and ROS2 are available.
For Ubuntu >20.04, only ROS2 is available.

Getting Started
---------------

1. Create a new MUV file `my_project.muv` in `<path/to/Multiverse>/multiverse/resources/muv` and open it in your favorite text editor.

Define resources, worlds, and simulations
-----------------------------------------

In this example, we will use only one robot and one object in the simulation.

.. code-block:: yaml

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
                path: floor.xml
            robots:
                tiago_dual:
                    path: tiago_dual.xml
                    apply:
                        body:
                            tiago_dual:
                                pos: [-1, -2, 0]
                    disable_self_collision: auto

            objects:
                milk_box:
                    path: milk_box.xml
                    apply:
                        body:
                            milk_box:
                                pos: [0.0, 0.0, 5.0]
                                quat: [0, 0.707, 0, 0.707]


Define ROS Configuration in the MUV file
----------------------------------------

2. Add the following lines to the MUV file to define the Multiverse Server:

.. code-block:: yaml

    multiverse_server:
        host: "tcp://127.0.0.1"
        port: 7000

3. Add the following lines to the MUV file to define the Multiverse Clients:

.. code-block:: yaml
    
    multiverse_clients:
        my_simulation:
            port: 7501
            send:
                body: ["position", "quaternion"]
                joint: ["joint_rvalue", "joint_tvalue", "joint_angular_velocity", "joint_linear_velocity", "joint_torque", "joint_force"]
            receive:
                tiago_dual: ["odometric_velocity"]
                torso_lift_actuator: ["cmd_joint_tvalue"]
                head_1_actuator: ["cmd_joint_rvalue"]
                head_2_actuator: ["cmd_joint_rvalue"]
                arm_left_1_actuator: ["cmd_joint_rvalue"]
                arm_left_2_actuator: ["cmd_joint_rvalue"]
                arm_left_3_actuator: ["cmd_joint_rvalue"]
                arm_left_4_actuator: ["cmd_joint_rvalue"]
                arm_left_5_actuator: ["cmd_joint_rvalue"]
                arm_left_6_actuator: ["cmd_joint_rvalue"]
                arm_left_7_actuator: ["cmd_joint_rvalue"]
                arm_right_1_actuator: ["cmd_joint_rvalue"]
                arm_right_2_actuator: ["cmd_joint_rvalue"]
                arm_right_3_actuator: ["cmd_joint_rvalue"]
                arm_right_4_actuator: ["cmd_joint_rvalue"]
                arm_right_5_actuator: ["cmd_joint_rvalue"]
                arm_right_6_actuator: ["cmd_joint_rvalue"]
                arm_right_7_actuator: ["cmd_joint_rvalue"]

                
        ros: # For ROS configuration, for ROS2 configuration use ros2
            ros_nodes:
                services:
                    socket:
                      - port: 7400

                publishers:
                    tf:
                      - meta_data:
                            world_name: my_world
                            length_unit: m
                            angle_unit: rad
                            mass_unit: kg
                            time_unit: s
                            handedness: rhs
                        port: 7301
                        topic: /tf
                        rate: 60
                        root_frame_id: map
                    odom:
                      - meta_data:
                            world_name: my_world
                            length_unit: m
                            angle_unit: rad
                            mass_unit: kg
                            time_unit: s
                            handedness: rhs
                        port: 7302
                        odom_topic: /odom
                        tf_topic: /tf
                        rate: 60
                        body: tiago_dual # The body to attach the odometry to
                        frame_id: map

                subscribers:
                    cmd_vel:
                      - meta_data:
                            world_name: my_world
                            length_unit: m
                            angle_unit: rad
                            mass_unit: kg
                            time_unit: s
                            handedness: rhs
                        port: 7203
                        topic: /cmd_vel
                        body: tiago_dual # The body to attach the velocity command to

            ros_control: # Only available for ROS, not yet for ROS2
            - meta_data:
                    world_name: my_world
                    length_unit: m
                    angle_unit: rad
                    mass_unit: kg
                    time_unit: s
                    handedness: rhs
                port: 7600
                controller_manager:
                    robot: tiago_dual
                    robot_description: /robot_description
                    urdf: tiago_dual/urdf/tiago_dual.urdf
                    config: tiago_dual/config/ros_control.yaml
                    actuators:
                        torso_lift_actuator: torso_lift_joint
                        head_1_actuator: head_1_joint
                        head_2_actuator: head_2_joint
                        arm_left_1_actuator: arm_left_1_joint
                        arm_left_2_actuator: arm_left_2_joint
                        arm_left_3_actuator: arm_left_3_joint
                        arm_left_4_actuator: arm_left_4_joint
                        arm_left_5_actuator: arm_left_5_joint
                        arm_left_6_actuator: arm_left_6_joint
                        arm_left_7_actuator: arm_left_7_joint
                        arm_right_1_actuator: arm_right_1_joint
                        arm_right_2_actuator: arm_right_2_joint
                        arm_right_3_actuator: arm_right_3_joint
                        arm_right_4_actuator: arm_right_4_joint
                        arm_right_5_actuator: arm_right_5_joint
                        arm_right_6_actuator: arm_right_6_joint
                        arm_right_7_actuator: arm_right_7_joint
                        gripper_left_left_finger_actuator: gripper_left_left_finger_joint
                        gripper_left_right_finger_actuator: gripper_left_right_finger_joint
                        gripper_right_left_finger_actuator: gripper_right_left_finger_joint
                        gripper_right_right_finger_actuator: gripper_right_right_finger_joint
                    controllers:
                        spawn:
                        - joint_state_controller
                            torso_controller
                            head_controller
                            arm_left_controller
                            arm_right_controller
                            gripper_left_left_finger_effort_controller
                            gripper_left_right_finger_effort_controller
                            gripper_right_left_finger_effort_controller
                            gripper_right_right_finger_effort_controller

1. Save the MUV file, and you are ready to connect Multiverse with ROS.

Running the Simulation and Testing ROS Connection
-------------------------------------------------

5. Launch the simulation using the following command:

.. code-block:: bash

    multiverse_launch  <path/to/Multiverse>/multiverse/resources/muv/my_project.muv

6. Open a new terminal and source the ROS workspace:

For ROS:
~~~~~~~~

.. code-block:: bash

    source <path/to/Multiverse>/multiverse_ws/devel/setup.bash
    rostopic list
    rosservice list

Here is the list of topics and services that you can see in ROS:

.. code-block:: bash

    rostopic list

    /cmd_vel
    /my_world/tiago_dual/arm_left_controller/command
    /my_world/tiago_dual/arm_left_controller/follow_joint_trajectory/cancel
    /my_world/tiago_dual/arm_left_controller/follow_joint_trajectory/feedback
    /my_world/tiago_dual/arm_left_controller/follow_joint_trajectory/goal
    /my_world/tiago_dual/arm_left_controller/follow_joint_trajectory/result
    /my_world/tiago_dual/arm_left_controller/follow_joint_trajectory/status
    /my_world/tiago_dual/arm_left_controller/state
    /my_world/tiago_dual/arm_right_controller/command
    /my_world/tiago_dual/arm_right_controller/follow_joint_trajectory/cancel
    /my_world/tiago_dual/arm_right_controller/follow_joint_trajectory/feedback
    /my_world/tiago_dual/arm_right_controller/follow_joint_trajectory/goal
    /my_world/tiago_dual/arm_right_controller/follow_joint_trajectory/result
    /my_world/tiago_dual/arm_right_controller/follow_joint_trajectory/status
    /my_world/tiago_dual/arm_right_controller/state
    /my_world/tiago_dual/gripper_left_left_finger_effort_controller/command
    /my_world/tiago_dual/gripper_left_right_finger_effort_controller/command
    /my_world/tiago_dual/gripper_right_left_finger_effort_controller/command
    /my_world/tiago_dual/gripper_right_right_finger_effort_controller/command
    /my_world/tiago_dual/head_controller/command
    /my_world/tiago_dual/head_controller/follow_joint_trajectory/cancel
    /my_world/tiago_dual/head_controller/follow_joint_trajectory/feedback
    /my_world/tiago_dual/head_controller/follow_joint_trajectory/goal
    /my_world/tiago_dual/head_controller/follow_joint_trajectory/result
    /my_world/tiago_dual/head_controller/follow_joint_trajectory/status
    /my_world/tiago_dual/head_controller/state
    /my_world/tiago_dual/joint_states
    /my_world/tiago_dual/torso_controller/command
    /my_world/tiago_dual/torso_controller/follow_joint_trajectory/cancel
    /my_world/tiago_dual/torso_controller/follow_joint_trajectory/feedback
    /my_world/tiago_dual/torso_controller/follow_joint_trajectory/goal
    /my_world/tiago_dual/torso_controller/follow_joint_trajectory/result
    /my_world/tiago_dual/torso_controller/follow_joint_trajectory/status
    /my_world/tiago_dual/torso_controller/state
    /odom
    /rosout
    /rosout_agg
    /tf

    rosservice list

    /multiverse/socket
    /multiverse_control_1721760267969714959/get_loggers
    /multiverse_control_1721760267969714959/set_logger_level
    /multiverse_ros_socket/get_loggers
    /multiverse_ros_socket/set_logger_level
    /my_world/tiago_dual/arm_left_controller/query_state
    /my_world/tiago_dual/arm_right_controller/query_state
    /my_world/tiago_dual/controller_manager/list_controller_types
    /my_world/tiago_dual/controller_manager/list_controllers
    /my_world/tiago_dual/controller_manager/load_controller
    /my_world/tiago_dual/controller_manager/reload_controller_libraries
    /my_world/tiago_dual/controller_manager/switch_controller
    /my_world/tiago_dual/controller_manager/unload_controller
    /my_world/tiago_dual/head_controller/query_state
    /my_world/tiago_dual/torso_controller/query_state
    /rosout/get_loggers
    /rosout/set_logger_level

To test the connection, you can publish a message to the `/cmd_vel` topic to control the base of the robot,
or you can control the joints of the robot using the `rqt_joint_trajectory_controller` tool.

You can also query the state of the simulations by calling the ROS service `/multiverse/socket`. For example:

.. code-block:: bash

    rosservice call /multiverse/socket "meta_data: {world_name: 'my_world', simulation_name: '', length_unit: 'm', angle_unit: 'rad',
  mass_unit: 'm', time_unit: 's', handedness: 'rhs'}
    send:
    -   object_name: ''
        attribute_name: ''
        data: [0]
    receive:
    -   object_name: 'tiago_dual'
        attribute_names: ['position']"

The above command will return the position of the robot in the simulation in meters in the right-handed coordinate system.
Changing the attributes in the meta data will return the data in the desired units.

.. code-block:: bash

    meta_data: 
        world_name: "my_world"
        simulation_name: "ros"
        length_unit: "m"
        angle_unit: "rad"
        mass_unit: "m"
        time_unit: "s"
        handedness: "rhs"
    send: []
    receive: 
    - 
        object_name: "tiago_dual"
        attribute_name: "position"
        data: [-2.0007730929407312, -3.9984003943955817, 0.0]

For ROS2:
~~~~~~~~~

.. code-block:: bash

    source <path/to/Multiverse>/multiverse_ws2/install/setup.bash
    ros2 topic list
    ros2 service list

Here is the list of topics and services that you can see in ROS2:

.. code-block:: bash

    ros2 topic list

    /cmd_vel
    /odom
    /parameter_events
    /rosout
    /tf

    ros2 service list

    /CmdVelSubscriber7203/describe_parameters
    /CmdVelSubscriber7203/get_parameter_types
    /CmdVelSubscriber7203/get_parameters
    /CmdVelSubscriber7203/list_parameters
    /CmdVelSubscriber7203/set_parameters
    /CmdVelSubscriber7203/set_parameters_atomically
    /OdomPublisher7302/describe_parameters
    /OdomPublisher7302/get_parameter_types
    /OdomPublisher7302/get_parameters
    /OdomPublisher7302/list_parameters
    /OdomPublisher7302/set_parameters
    /OdomPublisher7302/set_parameters_atomically
    /SocketService7400/describe_parameters
    /SocketService7400/get_parameter_types
    /SocketService7400/get_parameters
    /SocketService7400/list_parameters
    /SocketService7400/set_parameters
    /SocketService7400/set_parameters_atomically
    /TfPublisher7301/describe_parameters
    /TfPublisher7301/get_parameter_types
    /TfPublisher7301/get_parameters
    /TfPublisher7301/list_parameters
    /TfPublisher7301/set_parameters
    /TfPublisher7301/set_parameters_atomically
    /multiverse/socket

Conclusion
----------

Congratulations! You have successfully connected Multiverse with ROS. 
Until now, you have learned how to define resources, worlds, simulations, and communication between simulators in a MUV file, launch the simulations using the `multiverse_launch` command, and connect Multiverse with ROS to interact with robots and objects in the simulation.

Next Steps
----------

- Write your own Multiverse Connectors to interact with the Multiverse Server.