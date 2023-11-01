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

// void simulate()
// {
//     std::vector<MjHWInterface *> mj_hw_interfaces;
//     std::vector<controller_manager::ControllerManager *> controller_managers;
//     for (const std::string &robot_name : MjSim::robot_names)
//     {
//         mj_hw_interfaces.push_back(new MjHWInterface(robot_name));
//         if (MjSim::robot_names.size() < 2)
//         {
//             controller_managers.push_back(new controller_manager::ControllerManager(mj_hw_interfaces.back()));
//         }
//         else
//         {
//             controller_managers.push_back(new controller_manager::ControllerManager(mj_hw_interfaces.back(), ros::NodeHandle(robot_name)));
//         }
//     }

//     ros::AsyncSpinner spinner(3);
//     spinner.start();
//     ros::Time last_sim_time = MjRos::ros_start;
//     double time_step = m->opt.timestep;
//     while (ros::ok())
//     {
//         mj_multiverse_client.communicate();
//         {
//             ros::Time sim_time = (ros::Time)(MjRos::ros_start.toSec() + d->time);
//             ros::Duration sim_period = sim_time - last_sim_time;

//             mtx.lock();
//             mj_step1(m, d);
//             // check if we should update the controllers
//             if (sim_period.toSec() >= 1 / 10000.) // Controller with 10kHz
//             {
//                 // store simulation time
//                 last_sim_time = sim_time;

//                 // update the robot simulation with the state of the mujoco model
//                 for (MjHWInterface *mj_hw_interface : mj_hw_interfaces)
//                 {
//                     mj_hw_interface->read();
//                 }

//                 // compute the controller commands
//                 for (controller_manager::ControllerManager *controller_manager : controller_managers)
//                 {
//                     controller_manager->update(sim_time, sim_period);
//                 }
//             }
//             // update the mujoco model with the result of the controller
//             for (MjHWInterface *mj_hw_interface : mj_hw_interfaces)
//             {
//                 mj_hw_interface->write();
//             }

//             mj_step2(m, d);

//             mj_sim.set_odom_vels();

//             mtx.unlock();
//         }

//         // Calculate real time factor
//         int num_step = mju_ceil(1 / m->opt.timestep);
//         static std::deque<double> last_sim_time;
//         static std::deque<double> last_ros_time;
//         double error_time;
//         double ros_time;
//         double sim_time = d->time - MjSim::sim_start;
//         if (i == 0)
//         {
//             last_sim_time.clear();
//             last_ros_time.clear();
//         }
//         do
//         {
//             ros_time = (ros::Time::now() - MjRos::ros_start).toSec();
//             error_time = ros_time - sim_time;
//         } while (error_time < -1E-6 && i != 0);

//         sim_time = d->time - MjSim::sim_start;
//         last_ros_time.push_front(ros_time);
//         last_sim_time.push_front(sim_time);
//         if (i == num_step)
//         {
//             last_ros_time.pop_back();
//             last_sim_time.pop_back();
//         }
//         else
//         {
//             i++;
//         }
//         rtf = (ros_time - last_ros_time.back()) / (sim_time - last_sim_time.back());

//         // Change timestep when out of sync
//         if (error_time > 1E-3)
//         {
//             if (m->opt.timestep < MjSim::max_time_step)
//             {
//                 m->opt.timestep *= 2;
//             }
//         }
//         else
//         {
//             if (m->opt.timestep > time_step)
//             {
//                 m->opt.timestep /= 2;
//             }
//         }
//     }
// }

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
    server_host.erase(std::remove(server_host.begin(), server_host.end(), '\"'), server_host.end());
    server_host.erase(std::remove(server_host.begin(), server_host.end(), '\n'), server_host.end());
    std::string server_port = multiverse_server_params_json["port"].toStyledString();
    server_port.erase(std::remove(server_port.begin(), server_port.end(), '\"'), server_port.end());
    server_port.erase(std::remove(server_port.begin(), server_port.end(), '\n'), server_port.end());

    const Json::Value multiverse_client_params_json = multiverse_params_json["multiverse_client"];
    std::string client_port = multiverse_client_params_json["port"].toStyledString();
    client_port.erase(std::remove(client_port.begin(), client_port.end(), '\"'), client_port.end());
    client_port.erase(std::remove(client_port.begin(), client_port.end(), '\n'), client_port.end());
    std::string world = multiverse_client_params_json["world"].toStyledString();
    world.erase(std::remove(world.begin(), world.end(), '\"'), world.end());
    world.erase(std::remove(world.begin(), world.end(), '\n'), world.end());
    
    const Json::Value robot_joints_json = multiverse_params_json["robot_joints"];
    std::map<std::string, std::string> robot_joints;
    for (std::string joint_name : robot_joints_json.getMemberNames())
	{
        std::string joint_type = robot_joints_json[joint_name].toStyledString();
        joint_name.erase(std::remove(world.begin(), world.end(), '\"'), world.end());
        joint_name.erase(std::remove(world.begin(), world.end(), '\n'), world.end());
        joint_type.erase(std::remove(world.begin(), world.end(), '\"'), world.end());
        joint_type.erase(std::remove(world.begin(), world.end(), '\n'), world.end());
        robot_joints[joint_name] = joint_type;
    }

    MultiverseHWInterface multiverse_hw_interface(server_host, server_port, client_port, robot_joints, world);

    // // mj_multiverse_client.init(server_host, server_port, client_port, multiverse_client_params_json["send"], multiverse_client_params_json["receive"], world);

    return 0;
}