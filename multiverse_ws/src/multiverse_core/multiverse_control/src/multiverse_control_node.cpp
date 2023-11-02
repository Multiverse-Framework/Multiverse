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
#include <controller_manager/controller_manager.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multiverse_control", ros::init_options::AnonymousName);

    if (argc < 2)
    {
        return 0;
    }
    
    const std::string multiverse_params_str = argv[1];
    Json::Value multiverse_params_json;
    Json::Reader reader;
    if (!reader.parse(multiverse_params_str, multiverse_params_json) || multiverse_params_json.empty())
    {
        ROS_WARN("%s\n", multiverse_params_str.c_str());
        return 0;
    }

    const Json::Value multiverse_server_params_json = multiverse_params_json["multiverse_server"];
    std::string server_host = multiverse_server_params_json["host"].toStyledString();
    server_host.erase(std::remove(server_host.begin(), server_host.end(), '"'), server_host.end());
    server_host.erase(std::remove(server_host.begin(), server_host.end(), '\n'), server_host.end());
    std::string server_port = multiverse_server_params_json["port"].toStyledString();
    server_port.erase(std::remove(server_port.begin(), server_port.end(), '"'), server_port.end());
    server_port.erase(std::remove(server_port.begin(), server_port.end(), '\n'), server_port.end());

    const Json::Value multiverse_client_params_json = multiverse_params_json["multiverse_client"];
    std::string client_port = multiverse_client_params_json["port"].toStyledString();
    client_port.erase(std::remove(client_port.begin(), client_port.end(), '"'), client_port.end());
    client_port.erase(std::remove(client_port.begin(), client_port.end(), '\n'), client_port.end());
    std::string world = multiverse_client_params_json["world"].toStyledString();
    world.erase(std::remove(world.begin(), world.end(), '"'), world.end());
    world.erase(std::remove(world.begin(), world.end(), '\n'), world.end());
    
    const Json::Value robot_joints_json = multiverse_params_json["robots"];
    std::map<std::string, std::map<std::string, std::string>> robot_joints;
    for (std::string robot_name : robot_joints_json.getMemberNames())
	{
        const Json::Value robot_json = robot_joints_json[robot_name];
        for (std::string joint_name : robot_json.getMemberNames())
        {
            std::string joint_type = robot_json[joint_name].toStyledString();
            joint_name.erase(std::remove(joint_name.begin(), joint_name.end(), '"'), joint_name.end());
            joint_name.erase(std::remove(joint_name.begin(), joint_name.end(), '\n'), joint_name.end());
            joint_type.erase(std::remove(joint_type.begin(), joint_type.end(), '"'), joint_type.end());
            joint_type.erase(std::remove(joint_type.begin(), joint_type.end(), '\n'), joint_type.end());
            robot_joints[robot_name][joint_name] = joint_type;
        }
    }

    std::vector<MultiverseHWInterface *> multiverse_hw_interfaces;
    std::vector<controller_manager::ControllerManager *> controller_managers;
    for (const std::string &robot_name : robot_joints_json.getMemberNames())
    {
        ROS_WARN("%s\n", robot_name.c_str());
        multiverse_hw_interfaces.push_back(new MultiverseHWInterface(server_host, server_port, client_port, robot_joints[robot_name], world));
        controller_managers.push_back(new controller_manager::ControllerManager(multiverse_hw_interfaces.back(), ros::NodeHandle(robot_name)));
    }

    ros::AsyncSpinner spinner(3);
    spinner.start();
    
    ros::Time time_last = ros::Time::now();
    ros::Time time_now = ros::Time::now();

    while (ros::ok())
    {
        time_now = ros::Time::now();
        ros::Duration duration = time_now - time_last;
        for (controller_manager::ControllerManager *controller_manager : controller_managers)
        {
            controller_manager->update(time_now, duration);
        }
        time_last = time_now;
    }

    // // mj_multiverse_client.init(server_host, server_port, client_port, multiverse_client_params_json["send"], multiverse_client_params_json["receive"], world);

    return 0;
}