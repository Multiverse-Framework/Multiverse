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

    std::map<std::string, std::string> multiverse_params;

    const Json::Value multiverse_server_params_json = multiverse_params_json["multiverse_server"];
    multiverse_params["server_host"] = multiverse_server_params_json["host"].toStyledString();
    multiverse_params["server_port"] = multiverse_server_params_json["port"].toStyledString();

    const Json::Value multiverse_client_params_json = multiverse_params_json["multiverse_client"];
    multiverse_params["client_host"] = multiverse_client_params_json["host"].toStyledString();
    multiverse_params["client_port"] = multiverse_client_params_json["port"].toStyledString();

    const Json::Value multiverse_client_meta_data_json = multiverse_client_params_json["meta_data"];
    multiverse_params["world"] = multiverse_client_meta_data_json["world"].toStyledString();
    multiverse_params["length_unit"] = multiverse_client_meta_data_json["length_unit"].toStyledString();
    multiverse_params["angle_unit"] = multiverse_client_meta_data_json["angle_unit"].toStyledString();
    multiverse_params["mass_unit"] = multiverse_client_meta_data_json["mass_unit"].toStyledString();
    multiverse_params["time_unit"] = multiverse_client_meta_data_json["time_unit"].toStyledString();
    multiverse_params["handedness"] = multiverse_client_meta_data_json["handedness"].toStyledString();

    const Json::Value controller_manager_params_json = multiverse_params_json["controller_manager"];
    multiverse_params["robot"] = controller_manager_params_json["robot"].toStyledString();
    multiverse_params["robot_description"] = controller_manager_params_json["robot_description"].toStyledString();

    for (std::pair<const std::string, std::string> &multiverse_param : multiverse_params)
    {
        multiverse_param.second.erase(std::remove(multiverse_param.second.begin(), multiverse_param.second.end(), '"'), multiverse_param.second.end());
        multiverse_param.second.erase(std::remove(multiverse_param.second.begin(), multiverse_param.second.end(), '\n'), multiverse_param.second.end());
    }

    MultiverseHWInterface multiverse_hw_interface(multiverse_params);
    controller_manager::ControllerManager controller_manager(&multiverse_hw_interface, ros::NodeHandle(multiverse_params["robot"]));

    ros::AsyncSpinner spinner(3);
    spinner.start();
    
    double ros_time_start = ros::Time::now().toSec();
    ros::Time world_time_now = multiverse_hw_interface.get_world_time(ros_time_start);
    ros::Time world_time_last = world_time_now;
    ros::Duration duration;
    while (ros::ok())
    {
        do
        {
            multiverse_hw_interface.communicate();
            world_time_now = multiverse_hw_interface.get_world_time(ros_time_start);
            duration = world_time_now - world_time_last;
            world_time_last = world_time_now;
        } while (duration.toSec() <= 0.0 && ros::ok());
        
        controller_manager.update(ros::Time::now(), duration);
    }

    return 0;
}