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
#include <controller_manager/controller_manager.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

	if (argc < 3)
	{
		return 0;
	}

	const std::string &multiverse_params_str = argv[1];
	const std::string &robot_urdf_str = argv[2];
	Json::Value multiverse_params_json;
	Json::Reader reader;
	if (!reader.parse(multiverse_params_str, multiverse_params_json) || multiverse_params_json.empty())
	{
		printf("%s\n", multiverse_params_str.c_str());
		return 0;
	}

	std::map<std::string, std::string> multiverse_params;
	std::map<std::string, std::string> joint_actuator_map;

	const Json::Value &multiverse_server_params_json = multiverse_params_json["multiverse_server"];
	multiverse_params["server_host"] = multiverse_server_params_json["host"].asString();
	multiverse_params["server_port"] = multiverse_server_params_json["port"].asString();

	const Json::Value &multiverse_client_params_json = multiverse_params_json["multiverse_client"];
	multiverse_params["client_host"] = multiverse_client_params_json["host"].asString();
	multiverse_params["client_port"] = multiverse_client_params_json["port"].asString();

	const Json::Value &multiverse_client_meta_data_json = multiverse_client_params_json["meta_data"];
	multiverse_params["world_name"] = multiverse_client_meta_data_json["world_name"].asString();
	multiverse_params["length_unit"] = multiverse_client_meta_data_json["length_unit"].asString();
	multiverse_params["angle_unit"] = multiverse_client_meta_data_json["angle_unit"].asString();
	multiverse_params["mass_unit"] = multiverse_client_meta_data_json["mass_unit"].asString();
	multiverse_params["time_unit"] = multiverse_client_meta_data_json["time_unit"].asString();
	multiverse_params["handedness"] = multiverse_client_meta_data_json["handedness"].asString();

	const Json::Value &controller_manager_params_json = multiverse_params_json["controller_manager"];
	multiverse_params["robot"] = controller_manager_params_json["robot"].asString();

	const Json::Value &actuators_json = controller_manager_params_json["actuators"];
	for (const std::string &actuator_name : actuators_json.getMemberNames())
	{
		const std::string &joint_name = actuators_json[actuator_name].asString();
		joint_actuator_map[joint_name] = actuator_name;
	}

	MultiverseSystemHardware multiverse_system_hardware;
	multiverse_system_hardware.init(multiverse_params, joint_actuator_map);
	std::shared_ptr<rclcpp::Executor> executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
	controller_manager::ControllerManager controller_manager(std::make_unique<hardware_interface::ResourceManager>(robot_urdf_str), executor, "controller_manager", multiverse_params["robot"]);

	double ros_time_start = controller_manager.now().seconds();
	double world_time_now = multiverse_system_hardware.get_world_time(0);
	double world_time_last = world_time_now;
	double duration;
	while (rclcpp::ok())
	{
		do
		{
			multiverse_system_hardware.communicate();
			world_time_now = multiverse_system_hardware.get_world_time(ros_time_start);
			duration = world_time_now - world_time_last;
			world_time_last = world_time_now;
		} while (duration <= 0.0 && rclcpp::ok());

		controller_manager.read();
		controller_manager.update();
		controller_manager.write();
	}

	return 0;
}