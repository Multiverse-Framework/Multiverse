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

#include "multiverse_system_hardware.h"
#include <cstring>
#include <cmath>
#include <limits>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

MultiverseSystemHardware::MultiverseSystemHardware()
{
}

MultiverseSystemHardware::~MultiverseSystemHardware()
{
    disconnect();
}

void MultiverseSystemHardware::init(const std::map<std::string, std::string> &multiverse_params, const std::map<std::string, std::string> &in_joint_actuator_map)
{
    joint_actuator_map = in_joint_actuator_map;

    meta_data["world_name"] = multiverse_params.at("world_name");
    meta_data["simulation_name"] = multiverse_params.at("robot") + "_controller";
    meta_data["length_unit"] = multiverse_params.at("length_unit");
    meta_data["angle_unit"] = multiverse_params.at("angle_unit");
    meta_data["mass_unit"] = multiverse_params.at("mass_unit");
    meta_data["time_unit"] = multiverse_params.at("time_unit");
    meta_data["handedness"] = multiverse_params.at("handedness");

    server_socket_addr = multiverse_params.at("server_host") + ":" + multiverse_params.at("server_port");

    host = multiverse_params.at("client_host");
    port = multiverse_params.at("client_port");
}

double MultiverseSystemHardware::get_world_time(const double offset) const
{
    return world_time + offset;
}

hardware_interface::return_type MultiverseSystemHardware::configure(const hardware_interface::HardwareInfo &system_info)
{
    if (configure_default(system_info) != hardware_interface::return_type::OK)
    {
        return hardware_interface::return_type::ERROR;
    }

    for (const hardware_interface::ComponentInfo &robot_joint : system_info.joints)
    {
        if (joint_actuator_map.count(robot_joint.name) == 0)
        {
            continue;
        }
        actuator_joint_map[joint_actuator_map.at(robot_joint.name)] = robot_joint.name;

        if (strcmp(robot_joint.parameters.at("type").c_str(), "prismatic") == 0 ||
            strcmp(robot_joint.parameters.at("type").c_str(), "revolute") == 0)
        {
            if (strcmp(robot_joint.parameters.at("type").c_str(), "prismatic") == 0)
            {
                receive_objects[robot_joint.name] = {"joint_tvalue", "joint_linear_velocity", "joint_force"};
                send_objects[joint_actuator_map.at(robot_joint.name)] = {"cmd_joint_tvalue", "cmd_joint_linear_velocity", "cmd_joint_force"};
            }
            else if (strcmp(robot_joint.parameters.at("type").c_str(), "revolute") == 0)
            {
                receive_objects[robot_joint.name] = {"joint_rvalue", "joint_angular_velocity", "joint_torque"};
                send_objects[joint_actuator_map.at(robot_joint.name)] = {"cmd_joint_rvalue", "cmd_joint_angular_velocity", "cmd_joint_torque"};
            }

            joint_names.push_back(robot_joint.name);
            joint_states[robot_joint.name] = (double *)calloc(3, sizeof(double));
            joint_commands[robot_joint.name] = (double *)calloc(3, sizeof(double));
        }
    }

    connect();

    return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> MultiverseSystemHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (const std::string &joint_name : joint_names)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_name, hardware_interface::HW_IF_POSITION, &joint_states[joint_name][0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_name, hardware_interface::HW_IF_VELOCITY, &joint_states[joint_name][1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_name, hardware_interface::HW_IF_EFFORT, &joint_states[joint_name][2]));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MultiverseSystemHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (const std::string &joint_name : joint_names)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_name, hardware_interface::HW_IF_POSITION, &joint_commands[joint_name][0]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_name, hardware_interface::HW_IF_VELOCITY, &joint_commands[joint_name][1]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_name, hardware_interface::HW_IF_EFFORT, &joint_commands[joint_name][2]));
    }

    return command_interfaces;
}

hardware_interface::return_type MultiverseSystemHardware::start()
{
    for (const std::string &joint_name : joint_names)
    {
        for (size_t i = 0; i < 3; i++)
        {
            joint_commands[joint_name][i] = 0.0;
        }
    }

    status_ = hardware_interface::status::STARTED;

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MultiverseSystemHardware::stop()
{
    for (const std::string &joint_name : joint_names)
    {
        for (size_t i = 1; i < 3; i++)
        {
            joint_commands[joint_name][i] = 0.0;
        }
    }

    status_ = hardware_interface::status::STOPPED;

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MultiverseSystemHardware::read()
{
    for (const std::string &joint_name : joint_names)
    {
        for (size_t i = 1; i < 3; i++)
        {
            if (std::isnan(joint_states[joint_name][i]))
            {
                return hardware_interface::return_type::ERROR;
            }
        }
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MultiverseSystemHardware::write()
{
    for (const std::string &joint_name : joint_names)
    {
        for (size_t i = 1; i < 3; i++)
        {
            if (std::isnan(joint_commands[joint_name][i]))
            {
                return hardware_interface::return_type::ERROR;
            }
        }
    }
    return hardware_interface::return_type::OK;
}

bool MultiverseSystemHardware::init_objects(bool)
{
    return send_objects.size() > 0 || receive_objects.size() > 0;
}

void MultiverseSystemHardware::start_connect_to_server_thread()
{
    connect_to_server();
}

void MultiverseSystemHardware::wait_for_connect_to_server_thread_finish()
{
}

void MultiverseSystemHardware::start_meta_data_thread()
{
    send_and_receive_meta_data();
}

void MultiverseSystemHardware::wait_for_meta_data_thread_finish()
{
}

void MultiverseSystemHardware::bind_request_meta_data()
{
    // Create JSON object and populate it
    request_meta_data_json.clear();
    request_meta_data_json["meta_data"]["world_name"] = meta_data["world_name"];
    request_meta_data_json["meta_data"]["simulation_name"] = meta_data["simulation_name"];
    request_meta_data_json["meta_data"]["length_unit"] = meta_data["length_unit"];
    request_meta_data_json["meta_data"]["angle_unit"] = meta_data["angle_unit"];
    request_meta_data_json["meta_data"]["mass_unit"] = meta_data["mass_unit"];
    request_meta_data_json["meta_data"]["time_unit"] = meta_data["time_unit"];
    request_meta_data_json["meta_data"]["handedness"] = meta_data["handedness"];

    for (const std::pair<const std::string, std::set<std::string>> &send_object : send_objects)
    {
        for (const std::string &attribute_name : send_object.second)
        {
            request_meta_data_json["send"][send_object.first].append(attribute_name);
        }
    }

    for (const std::pair<const std::string, std::set<std::string>> &receive_object : receive_objects)
    {
        for (const std::string &attribute_name : receive_object.second)
        {
            request_meta_data_json["receive"][receive_object.first].append(attribute_name);
        }
    }

    request_meta_data_str = request_meta_data_json.toStyledString();
}

void MultiverseSystemHardware::bind_response_meta_data()
{
    
}

void MultiverseSystemHardware::init_send_and_receive_data()
{
    send_data_vec.emplace_back(nullptr);

    for (const std::pair<const std::string, std::set<std::string>> &send_object : send_objects)
    {
        for (const std::string &attribute_name : send_object.second)
        {
            if (strcmp(attribute_name.c_str(), "cmd_joint_tvalue") == 0 || strcmp(attribute_name.c_str(), "cmd_joint_rvalue") == 0)
            {
                send_data_vec.emplace_back(&joint_commands.at(actuator_joint_map.at(send_object.first))[0]);
            }
            else if (strcmp(attribute_name.c_str(), "cmd_joint_linear_velocity") == 0 || strcmp(attribute_name.c_str(), "cmd_joint_angular_velocity") == 0)
            {
                send_data_vec.emplace_back(&joint_commands.at(actuator_joint_map.at(send_object.first))[1]);
            }
            else if (strcmp(attribute_name.c_str(), "cmd_joint_force") == 0 || strcmp(attribute_name.c_str(), "cmd_joint_torque") == 0)
            {
                send_data_vec.emplace_back(&joint_commands.at(actuator_joint_map.at(send_object.first))[2]);
            }
        }
    }

    receive_data_vec.emplace_back(&world_time);
    for (const std::pair<const std::string, std::set<std::string>> &receive_object : receive_objects)
    {
        for (const std::string &attribute_name : receive_object.second)
        {
            if (strcmp(attribute_name.c_str(), "joint_tvalue") == 0 || strcmp(attribute_name.c_str(), "joint_rvalue") == 0)
            {
                receive_data_vec.emplace_back(&joint_states[receive_object.first][0]);
            }
            else if (strcmp(attribute_name.c_str(), "joint_linear_velocity") == 0 || strcmp(attribute_name.c_str(), "joint_angular_velocity") == 0)
            {
                receive_data_vec.emplace_back(&joint_states[receive_object.first][1]);
            }
            else if (strcmp(attribute_name.c_str(), "joint_force") == 0 || strcmp(attribute_name.c_str(), "joint_torque") == 0)
            {
                receive_data_vec.emplace_back(&joint_states[receive_object.first][2]);
            }
        }
    }
}

void MultiverseSystemHardware::bind_send_data()
{
    send_buffer[0] = std::numeric_limits<double>::quiet_NaN();
    for (size_t i = 1; i < send_buffer_size; i++)
    {
        send_buffer[i] = *send_data_vec[i];
    }
}

void MultiverseSystemHardware::bind_receive_data()
{
    for (size_t i = 0; i < receive_buffer_size; i++)
    {
        *receive_data_vec[i] = receive_buffer[i];
    }
}

void MultiverseSystemHardware::clean_up()
{
    send_data_vec.clear();
    receive_data_vec.clear();
}

void MultiverseSystemHardware::communicate(const bool resend_meta_data)
{
    MultiverseClient::communicate(resend_meta_data);
}

void MultiverseSystemHardware::reset()
{
    for (const std::string &joint_name : joint_names)
    {
        joint_commands[joint_name][0] = 0.0;
        joint_commands[joint_name][1] = 0.0;
        joint_commands[joint_name][2] = 0.0;
    }
}