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

#pragma once

#include "multiverse_client_json.h"

#include "string"
#include "unordered_map"
#include "vector"
#include "set"
#include "thread"

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

class MultiverseHWInterface : public MultiverseClientJson, public hardware_interface::SystemInterface
{
public:
    ~MultiverseHWInterface();

public:
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    void start_connect_to_server_thread() override;

    void wait_for_connect_to_server_thread_finish() override;

    void start_meta_data_thread() override;

    void wait_for_meta_data_thread_finish() override;

    bool init_objects(bool from_request_meta_data = false) override;

    void bind_request_meta_data() override;

    void bind_response_meta_data() override;

    void bind_api_callbacks() override;

    void bind_api_callbacks_response() override;

    void init_send_and_receive_data() override;

    void bind_send_data() override;

    void bind_receive_data() override;

    void clean_up() override;

    void reset() override;

private:
    std::map<std::string, std::string> meta_data;

    std::map<std::string, std::set<std::string>> send_objects;

    std::map<std::string, std::set<std::string>> receive_objects;

    std::vector<double *> send_data_vec;

    std::vector<double *> receive_data_vec;

    std::vector<std::string> joint_names;

    std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = {
        {"position", {}}, {"velocity", {}}, {"acceleration", {}}};

    std::map<std::string, std::string> actuators;

    std::unordered_map<std::string, std::vector<std::string>> actuator_interfaces = {
        {"position", {}}, {"velocity", {}}, {"effort", {}}};

    std::map<std::string, double> init_joint_positions;

    std::map<std::string, double *> joint_states;

    std::map<std::string, double *> joint_commands;

    double sim_start_time;

    std::thread commnunicate_thread;
};