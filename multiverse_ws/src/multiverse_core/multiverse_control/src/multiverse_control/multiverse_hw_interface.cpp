// Copyright (c) 2023, Giang Hoang Nguyen - Institute for Artificial Intelligence, University Bremen

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "multiverse_hw_interface.h"
#include <urdf/model.h>

MultiverseHWInterface::MultiverseHWInterface(const std::map<std::string, std::string> &multiverse_params)
{
    meta_data["world"] = multiverse_params.at("world");
    meta_data["length_unit"] = multiverse_params.at("length_unit");
    meta_data["angle_unit"] = multiverse_params.at("angle_unit");
    meta_data["mass_unit"] = multiverse_params.at("mass_unit");
    meta_data["time_unit"] = multiverse_params.at("time_unit");
    meta_data["handedness"] = multiverse_params.at("handedness");
    
    urdf::Model urdf_model;
    const std::string robot_description = multiverse_params.at("robot_description");
    if (!urdf_model.initParamWithNodeHandle(robot_description, n))
    {
        ROS_WARN("Failed to load robot from %s\n", robot_description.c_str());
        return;
    }

    server_socket_addr = multiverse_params.at("server_host") + ":" + multiverse_params.at("server_port");
    
    host = multiverse_params.at("client_host");
	port = multiverse_params.at("client_port");
    
    for (const std::pair<std::string, urdf::JointSharedPtr> &robot_joint : urdf_model.joints_)
    {
        if (robot_joint.second->type == urdf::Joint::PRISMATIC || robot_joint.second->type == urdf::Joint::REVOLUTE)
        {
            if (robot_joint.second->type == urdf::Joint::PRISMATIC)
            {
                receive_objects[robot_joint.first] = {"joint_tvalue", "joint_linear_velocity", "joint_force"};
                send_objects[robot_joint.first] = {"cmd_joint_tvalue", "cmd_joint_linear_velocity", "cmd_joint_force"};
            }
            else if (robot_joint.second->type == urdf::Joint::REVOLUTE)
            {
                receive_objects[robot_joint.first] = {"joint_rvalue", "joint_angular_velocity", "joint_torque"};
                send_objects[robot_joint.first] = {"cmd_joint_rvalue", "cmd_joint_angular_velocity", "cmd_joint_torque"};
            }

            joint_names.push_back(robot_joint.first);
            joint_states[robot_joint.first] = (double *)calloc(3, sizeof(double));
            joint_commands[robot_joint.first] = (double *)calloc(3, sizeof(double));
        }
    }
    
    for (const std::string &joint_name : joint_names)
    {
        hardware_interface::JointStateHandle joint_state_handle(joint_name, &joint_states[joint_name][0], &joint_states[joint_name][1], &joint_states[joint_name][2]);
        joint_state_interface.registerHandle(joint_state_handle);

        hardware_interface::JointHandle joint_handle_position(joint_state_interface.getHandle(joint_name), &joint_commands[joint_name][0]);
        position_joint_interface.registerHandle(joint_handle_position);

        hardware_interface::JointHandle joint_handle_velocity(joint_state_interface.getHandle(joint_name), &joint_commands[joint_name][1]);
        velocity_joint_interface.registerHandle(joint_handle_velocity);

        hardware_interface::JointHandle joint_handle_effort(joint_state_interface.getHandle(joint_name), &joint_commands[joint_name][2]);
        effort_joint_interface.registerHandle(joint_handle_effort);
    }
    registerInterface(&joint_state_interface);
    registerInterface(&position_joint_interface);
    registerInterface(&velocity_joint_interface);
    registerInterface(&effort_joint_interface);

    connect();
}

MultiverseHWInterface::~MultiverseHWInterface()
{
    disconnect();
}

bool MultiverseHWInterface::init_objects()
{
    return send_objects.size() > 0 || receive_objects.size() > 0;
}

void MultiverseHWInterface::start_connect_to_server_thread()
{
    connect_to_server();
}

void MultiverseHWInterface::wait_for_connect_to_server_thread_finish()
{
}

void MultiverseHWInterface::start_meta_data_thread()
{
    send_and_receive_meta_data();
}

void MultiverseHWInterface::wait_for_meta_data_thread_finish()
{
}

void MultiverseHWInterface::bind_request_meta_data()
{
    // Create JSON object and populate it
    request_meta_data_json.clear();
    request_meta_data_json["world"] = meta_data["world"];
    request_meta_data_json["length_unit"] = meta_data["length_unit"];
    request_meta_data_json["angle_unit"] = meta_data["angle_unit"];
    request_meta_data_json["mass_unit"] = meta_data["mass_unit"];
    request_meta_data_json["time_unit"] = meta_data["time_unit"];
    request_meta_data_json["handedness"] = meta_data["handedness"];

    for (const std::pair<std::string, std::set<std::string>> &send_object : send_objects)
    {
        for (const std::string &attribute_name : send_object.second)
        {
            request_meta_data_json["send"][send_object.first].append(attribute_name);
        }
    }

    for (const std::pair<std::string, std::set<std::string>> &receive_object : receive_objects)
    {
        for (const std::string &attribute_name : receive_object.second)
        {
            request_meta_data_json["receive"][receive_object.first].append(attribute_name);
        }
    }

    request_meta_data_str = request_meta_data_json.toStyledString();
}

void MultiverseHWInterface::bind_response_meta_data()
{
}

void MultiverseHWInterface::init_send_and_receive_data()
{
    
}

void MultiverseHWInterface::bind_send_data()
{
    double *send_buffer_addr = send_buffer + 1;

    for (const std::string &joint_name : joint_names)
    {
        *send_buffer_addr++ = joint_commands[joint_name][0];
        *send_buffer_addr++ = joint_commands[joint_name][1];
        *send_buffer_addr++ = joint_commands[joint_name][2];
    }
}

void MultiverseHWInterface::bind_receive_data()
{
    double *receive_buffer_addr = receive_buffer + 1;

    for (const std::string &joint_name : joint_names)
    {
        joint_states[joint_name][0] = *receive_buffer_addr++;
        joint_states[joint_name][1] = *receive_buffer_addr++;
        joint_states[joint_name][2] = *receive_buffer_addr++;
    }
}

void MultiverseHWInterface::clean_up()
{

}

void MultiverseHWInterface::communicate(const bool resend_meta_data)
{
    MultiverseClient::communicate(resend_meta_data);
    ROS_WARN("%f - %f - %f\n", joint_commands["shoulder_pan_joint"][0], joint_commands["shoulder_pan_joint"][1], joint_commands["shoulder_pan_joint"][2]);
}

void MultiverseHWInterface::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                                     const std::list<hardware_interface::ControllerInfo> &stop_list)
{
    for (const hardware_interface::ControllerInfo &stop_controller : stop_list)
    {
        for (const hardware_interface::InterfaceResources &interface_resource : stop_controller.claimed_resources)
        {
            for (const std::string &joint_name : interface_resource.resources)
            {
                if (std::find(joint_names.begin(), joint_names.end(), joint_name) != joint_names.end())
                {
                    joint_commands[joint_name][1] = 0.0;
                    joint_commands[joint_name][2] = 0.0;
                }
            }
        }
    }
}