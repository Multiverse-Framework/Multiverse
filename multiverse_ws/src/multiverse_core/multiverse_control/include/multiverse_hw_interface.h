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

class MultiverseHWInterface : public hardware_interface::RobotHW
{
public:
    MultiverseHWInterface(const std::map<std::string, double> &joint_states);

    ~MultiverseHWInterface();

public:
    void read();

    void write();

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

    // States
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
    std::vector<double> joint_efforts;

    // Commands
    std::vector<double> joint_positions_command;
    std::vector<double> joint_velocities_command;
    std::vector<double> joint_efforts_command;
};