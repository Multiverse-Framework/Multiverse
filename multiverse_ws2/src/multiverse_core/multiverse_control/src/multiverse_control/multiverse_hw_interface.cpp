// Copyright (c) 2024, Giang Hoang Nguyen - Institute for Artificial Intelligence, University Bremen

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

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MultiverseHWInterface::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    meta_data["world_name"] = info_.hardware_parameters["world_name"];
    meta_data["simulation_name"] = info_.hardware_parameters["robot"] + "_controller";
    meta_data["length_unit"] = info_.hardware_parameters["length_unit"];
    meta_data["angle_unit"] = info_.hardware_parameters["angle_unit"];
    meta_data["mass_unit"] = info_.hardware_parameters["mass_unit"];
    meta_data["time_unit"] = info_.hardware_parameters["time_unit"];
    meta_data["handedness"] = info_.hardware_parameters["handedness"];

    host = info_.hardware_parameters["host"];
    server_port = info_.hardware_parameters["server_port"];
    client_port = info_.hardware_parameters["client_port"];

    urdf::Model urdf_model;
    const std::string robot_description = info_.original_xml;
    if (!urdf_model.initString(robot_description))
    {
        RCLCPP_ERROR(rclcpp::get_logger("multiverse_hw_interface"), "Failed to load robot_description");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
        receive_objects[joint.name] = {};

        auto joint_type = urdf_model.getJoint(joint.name)->type;
        if (joint_type == urdf::Joint::PRISMATIC || joint_type == urdf::Joint::REVOLUTE || joint_type == urdf::Joint::CONTINUOUS)
        {
            if (joint_type == urdf::Joint::PRISMATIC)
            {
                receive_objects[joint.name] = {"joint_tvalue", "joint_linear_velocity"};
            }
            else if (joint_type == urdf::Joint::REVOLUTE || joint_type == urdf::Joint::CONTINUOUS)
            {
                receive_objects[joint.name] = {"joint_rvalue", "joint_angular_velocity"};
            }

            joint_names.push_back(joint.name);
            joint_states[joint.name] = (double *)calloc(2, sizeof(double));
            joint_commands[joint.name] = (double *)calloc(3, sizeof(double));

            init_joint_positions[joint.name] = 0.0;
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("multiverse_hw_interface"), "Joint %s is not prismatic, revolute or continuous, will be ignored", joint.name.c_str());
        }

        for (const hardware_interface::InterfaceInfo &state_interface : joint.state_interfaces)
        {
            if (strcmp(state_interface.name.c_str(), "position") == 0)
            {
                if (state_interface.initial_value.empty())
                {
                    RCLCPP_WARN(rclcpp::get_logger("multiverse_hw_interface"), "initial_value not found for joint %s, set default position as 0.0", joint.name.c_str());
                }
                else
                {
                    init_joint_positions[joint.name] = std::stod(state_interface.initial_value);
                }
                joint_states[joint.name][0] = init_joint_positions[joint.name];
                joint_commands[joint.name][0] = init_joint_positions[joint.name];
            }
            joint_interfaces[state_interface.name].push_back(joint.name);
        }

        if (joint.parameters.count("actuator") == 0)
        {
            continue;
        }

        std::string actuator_name = joint.parameters.at("actuator");
        actuators[actuator_name] = joint.name;
        send_objects[actuator_name] = {};
        for (const hardware_interface::InterfaceInfo &command_interface : joint.command_interfaces)
        {
            actuator_interfaces[command_interface.name].push_back(joint.name);
            if (joint_type == urdf::Joint::PRISMATIC)
            {
                if (strcmp(command_interface.name.c_str(), "position") == 0)
                {
                    send_objects[actuator_name].insert("cmd_joint_tvalue");
                }
                else if (strcmp(command_interface.name.c_str(), "velocity") == 0)
                {
                    send_objects[actuator_name].insert("cmd_joint_linear_velocity");
                }
                else if (strcmp(command_interface.name.c_str(), "effort") == 0)
                {
                    send_objects[actuator_name].insert("cmd_joint_force");
                }
                else
                {
                    RCLCPP_WARN(rclcpp::get_logger("multiverse_hw_interface"), "Command interface %s is not supported for joint %s", command_interface.name.c_str(), joint.name.c_str());
                }
            }
            else if (joint_type == urdf::Joint::REVOLUTE || joint_type == urdf::Joint::CONTINUOUS)
            {
                if (strcmp(command_interface.name.c_str(), "position") == 0)
                {
                    send_objects[actuator_name].insert("cmd_joint_rvalue");
                }
                else if (strcmp(command_interface.name.c_str(), "velocity") == 0)
                {
                    send_objects[actuator_name].insert("cmd_joint_angular_velocity");
                }
                else if (strcmp(command_interface.name.c_str(), "effort") == 0)
                {
                    send_objects[actuator_name].insert("cmd_joint_torque");
                }
                else
                {
                    RCLCPP_WARN(rclcpp::get_logger("multiverse_hw_interface"), "Command interface %s is not supported for joint %s", command_interface.name.c_str(), joint.name.c_str());
                }
            }
        }
    }

    connect();

    *world_time = 0.0;

    sim_start_time = get_time_now();

    commnunicate_thread = std::thread(
        [this]() {
            while (rclcpp::ok())
            {
                communicate();
            }
            disconnect();
        });

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MultiverseHWInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (const std::string &joint_name : joint_interfaces["position"])
    {
        state_interfaces.emplace_back(joint_name, "position", &joint_states[joint_name][0]);
    }

    for (const std::string &joint_name : joint_interfaces["velocity"])
    {
        state_interfaces.emplace_back(joint_name, "velocity", &joint_states[joint_name][1]);
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MultiverseHWInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (const std::string &joint_name : actuator_interfaces["position"])
    {
        command_interfaces.emplace_back(joint_name, "position", &joint_commands[joint_name][0]);
    }

    for (const std::string &joint_name : actuator_interfaces["velocity"])
    {
        command_interfaces.emplace_back(joint_name, "velocity", &joint_commands[joint_name][1]);
    }

    for (const std::string &joint_name : actuator_interfaces["effort"])
    {
        command_interfaces.emplace_back(joint_name, "effort", &joint_commands[joint_name][2]);
    }

    return command_interfaces;
}

hardware_interface::return_type MultiverseHWInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MultiverseHWInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    return hardware_interface::return_type::OK;
}

MultiverseHWInterface::~MultiverseHWInterface()
{
    commnunicate_thread.join();
}

bool MultiverseHWInterface::init_objects(bool)
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

void MultiverseHWInterface::bind_response_meta_data()
{
}

void MultiverseHWInterface::bind_api_callbacks()
{
}

void MultiverseHWInterface::bind_api_callbacks_response()
{
}

void MultiverseHWInterface::init_send_and_receive_data()
{
    for (const std::pair<const std::string, std::set<std::string>> &send_object : send_objects)
    {
        for (const std::string &attribute_name : send_object.second)
        {
            if (strcmp(attribute_name.c_str(), "cmd_joint_tvalue") == 0 || strcmp(attribute_name.c_str(), "cmd_joint_rvalue") == 0)
            {
                send_data_vec.emplace_back(&joint_commands.at(actuators.at(send_object.first))[0]);
            }
            else if (strcmp(attribute_name.c_str(), "cmd_joint_linear_velocity") == 0 || strcmp(attribute_name.c_str(), "cmd_joint_angular_velocity") == 0)
            {
                send_data_vec.emplace_back(&joint_commands.at(actuators.at(send_object.first))[1]);
            }
            else if (strcmp(attribute_name.c_str(), "cmd_joint_force") == 0 || strcmp(attribute_name.c_str(), "cmd_joint_torque") == 0)
            {
                send_data_vec.emplace_back(&joint_commands.at(actuators.at(send_object.first))[2]);
            }
        }
    }

    for (const std::pair<const std::string, std::set<std::string>> &receive_object : receive_objects)
    {
        for (const std::string &attribute_name : receive_object.second)
        {
            if (strcmp(attribute_name.c_str(), "joint_tvalue") == 0 || strcmp(attribute_name.c_str(), "joint_rvalue") == 0)
            {
                receive_data_vec.emplace_back(&joint_states.at(receive_object.first)[0]);
            }
            else if (strcmp(attribute_name.c_str(), "joint_linear_velocity") == 0 || strcmp(attribute_name.c_str(), "joint_angular_velocity") == 0)
            {
                receive_data_vec.emplace_back(&joint_states.at(receive_object.first)[1]);
            }
        }
    }
}

void MultiverseHWInterface::bind_send_data()
{
    *world_time = get_time_now() - sim_start_time;
    for (size_t i = 0; i < send_buffer.buffer_double.size; i++)
    {
        send_buffer.buffer_double.data[i] = *send_data_vec[i];
    }
}

void MultiverseHWInterface::bind_receive_data()
{
    for (size_t i = 0; i < receive_buffer.buffer_double.size; i++)
    {
        *receive_data_vec[i] = receive_buffer.buffer_double.data[i];
    }
}

void MultiverseHWInterface::clean_up()
{
    send_data_vec.clear();
    receive_data_vec.clear();
}

void MultiverseHWInterface::reset()
{
    sim_start_time = get_time_now();
    for (const std::string &joint_name : joint_names)
    {
        joint_commands[joint_name][0] = init_joint_positions[joint_name];
        joint_commands[joint_name][1] = 0.0;
        joint_commands[joint_name][2] = 0.0;
    }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(MultiverseHWInterface, hardware_interface::SystemInterface)
