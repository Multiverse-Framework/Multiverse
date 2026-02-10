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

#pragma once

#include "multiverse_client_json.h"

// ROS Controls
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class MultiverseHWInterface : public MultiverseClientJson, public hardware_interface::RobotHW
{
public:
    MultiverseHWInterface(const std::map<std::string, std::string> &multiverse_params,
                          const std::map<std::string, std::string> &joint_actuators,
                          const std::map<std::string, double> &init_joint_states);

    ~MultiverseHWInterface();

public:
    ros::Time get_world_time(const double offset = 0.0) const;

public:
    bool communicate(const bool resend_meta_data = false) override;

private:
    std::map<std::string, std::string> meta_data;

    std::map<std::string, std::set<std::string>> send_objects;

    std::map<std::string, std::set<std::string>> receive_objects;

    std::vector<double *> send_data_vec;

    std::vector<double *> receive_data_vec;

    std::map<std::string, std::string> actuators;

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

public:
    void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                  const std::list<hardware_interface::ControllerInfo> &stop_list);

private:
    ros::NodeHandle n;

    // Hardware interfaces
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::PositionJointInterface position_joint_interface;
    hardware_interface::VelocityJointInterface velocity_joint_interface;
    hardware_interface::EffortJointInterface effort_joint_interface;

private:
    std::vector<std::string> joint_names;

    // Initial states
    std::map<std::string, double> init_joint_states;

    // States
    std::map<std::string, double *> joint_states;

    // Commands
    std::map<std::string, double *> joint_commands;

    double sim_start_time;
};