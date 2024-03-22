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

#define _USE_MATH_DEFINES
#include <set>
#include <chrono>
#include <csignal>
#include <iostream>
#include <mutex>
#include <thread>

#include "multiverse_server.h"

#define STRING_SIZE 2000

using namespace std::chrono_literals;

bool should_shut_down = false;
std::map<std::string, bool> sockets_need_clean_up;
zmq::context_t server_context{1};

std::set<std::string> cumulative_attribute_names = {"force", "torque"};

std::map<std::string, std::pair<EAttribute, std::vector<double>>> attribute_map =
    {
        {"time", {EAttribute::Time, {0.0}}},
        {"position", {EAttribute::Position, {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()}}},
        {"quaternion", {EAttribute::Quaternion, {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()}}},
        {"relative_velocity", {EAttribute::RelativeVelocity, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}},
        {"odometric_velocity", {EAttribute::OdometricVelocity, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}},
        {"joint_rvalue", {EAttribute::JointRvalue, {std::numeric_limits<double>::quiet_NaN()}}},
        {"joint_tvalue", {EAttribute::JointTvalue, {std::numeric_limits<double>::quiet_NaN()}}},
        {"joint_linear_velocity", {EAttribute::JointLinearVelocity, {std::numeric_limits<double>::quiet_NaN()}}},
        {"joint_angular_velocity", {EAttribute::JointAngularVelocity, {std::numeric_limits<double>::quiet_NaN()}}},
        {"joint_force", {EAttribute::JointForce, {std::numeric_limits<double>::quiet_NaN()}}},
        {"joint_torque", {EAttribute::JointTorque, {std::numeric_limits<double>::quiet_NaN()}}},
        {"cmd_joint_rvalue", {EAttribute::CmdJointRvalue, {std::numeric_limits<double>::quiet_NaN()}}},
        {"cmd_joint_tvalue", {EAttribute::CmdJointTvalue, {std::numeric_limits<double>::quiet_NaN()}}},
        {"cmd_joint_linear_velocity", {EAttribute::CmdJointLinearVelocity, {std::numeric_limits<double>::quiet_NaN()}}},
        {"cmd_joint_angular_velocity", {EAttribute::CmdJointAngularVelocity, {std::numeric_limits<double>::quiet_NaN()}}},
        {"cmd_joint_force", {EAttribute::CmdJointForce, {std::numeric_limits<double>::quiet_NaN()}}},
        {"cmd_joint_torque", {EAttribute::CmdJointTorque, {std::numeric_limits<double>::quiet_NaN()}}},
        {"joint_position", {EAttribute::JointPosition, {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()}}},
        {"joint_quaternion", {EAttribute::JointQuaternion, {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()}}},
        {"force", {EAttribute::Force, {0.0, 0.0, 0.0}}},
        {"torque", {EAttribute::Torque, {0.0, 0.0, 0.0}}}};

std::map<std::string, double> unit_scale =
    {
        {"s", 1.0},
        {"ms", 0.001},
        {"us", 0.00001},
        {"m", 1.0},
        {"cm", 0.01},
        {"rad", 1.0},
        {"deg", M_PI / 180.0},
        {"mg", 0.00001},
        {"g", 0.001},
        {"kg", 1.0}};

std::map<EAttribute, std::map<std::string, std::vector<double>>> handedness_scale =
    {
        {EAttribute::Time,
         {{"rhs", {1.0}},
          {"lhs", {1.0}}}},
        {EAttribute::Position,
         {{"rhs", {1.0, 1.0, 1.0}},
          {"lhs", {1.0, -1.0, 1.0}}}},
        {EAttribute::Quaternion,
         {{"rhs", {1.0, 1.0, 1.0, 1.0}},
          {"lhs", {-1.0, 1.0, -1.0, 1.0}}}},
        {EAttribute::RelativeVelocity,
         {{"rhs", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}},
          {"lhs", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}}}},
        {EAttribute::OdometricVelocity,
         {{"rhs", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}},
          {"lhs", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}}}},
        {EAttribute::JointRvalue,
         {{"rhs", {1.0}},
          {"lhs", {-1.0}}}},
        {EAttribute::JointTvalue,
         {{"rhs", {1.0}},
          {"lhs", {-1.0}}}},
        {EAttribute::JointLinearVelocity,
         {{"rhs", {1.0}},
          {"lhs", {1.0}}}},
        {EAttribute::JointAngularVelocity,
         {{"rhs", {1.0}},
          {"lhs", {1.0}}}},
        {EAttribute::JointForce,
         {{"rhs", {1.0}},
          {"lhs", {1.0}}}},
        {EAttribute::JointTorque,
         {{"rhs", {1.0}},
          {"lhs", {1.0}}}},
        {EAttribute::CmdJointRvalue,
         {{"rhs", {1.0}},
          {"lhs", {-1.0}}}},
        {EAttribute::CmdJointTvalue,
         {{"rhs", {1.0}},
          {"lhs", {-1.0}}}},
        {EAttribute::CmdJointLinearVelocity,
         {{"rhs", {1.0}},
          {"lhs", {1.0}}}},
        {EAttribute::CmdJointAngularVelocity,
         {{"rhs", {1.0}},
          {"lhs", {1.0}}}},
        {EAttribute::CmdJointForce,
         {{"rhs", {1.0}},
          {"lhs", {1.0}}}},
        {EAttribute::CmdJointTorque,
         {{"rhs", {1.0}},
          {"lhs", {1.0}}}},
        {EAttribute::JointPosition,
         {{"rhs", {1.0, 1.0, 1.0}},
          {"lhs", {1.0, -1.0, 1.0}}}},
        {EAttribute::JointQuaternion,
         {{"rhs", {1.0, 1.0, 1.0, 1.0}},
          {"lhs", {1.0, 1.0, -1.0, 1.0}}}},
        {EAttribute::Force,
         {{"rhs", {1.0, 1.0, 1.0}},
          {"lhs", {1.0, -1.0, 1.0}}}},
        {EAttribute::Torque,
         {{"rhs", {1.0, 1.0, 1.0}},
          {"lhs", {1.0, -1.0, 1.0}}}}};

enum class EMetaDataState : unsigned char
{
    Normal,
    Reset,
    WaitAfterSendReceiveData,
    WaitAfterOtherBindSendData,
    WaitAfterOtherSendRequestMetaData
};

std::mutex mtx;

struct Attribute
{
    std::vector<double> data;
    std::map<std::string, std::vector<double>> simulation_data;
    bool is_sent = false;
};

struct Object
{
    std::map<std::string, Attribute> attributes;
};

struct Simulation
{
    std::map<std::string, Object *> objects;
    Json::Value request_meta_data_json;
    EMetaDataState meta_data_state;
};

struct World
{
    std::map<std::string, Object> objects;
    std::map<std::string, Simulation> simulations;
    double time = 0.0;
};

std::map<std::string, World> worlds;

static double get_time_now()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000000.0;
}

MultiverseServer::MultiverseServer(const std::string &in_socket_addr)
{
    socket = zmq::socket_t(server_context, zmq::socket_type::rep);
    socket_addr = in_socket_addr;
    socket.bind(socket_addr);
    sockets_need_clean_up[socket_addr] = false;
    printf("[Server] Bind to socket %s.\n", socket_addr.c_str());
}

MultiverseServer::~MultiverseServer()
{
    printf("[Server] Close socket %s.\n", socket_addr.c_str());

    if (send_buffer != nullptr)
    {
        free(send_buffer);
    }

    if (receive_buffer != nullptr)
    {
        free(receive_buffer);
    }

    sockets_need_clean_up[socket_addr] = false;
}

void MultiverseServer::start()
{
    while (!should_shut_down)
    {
        switch (flag)
        {
        case EMultiverseServerState::ReceiveRequestMetaData:
        {
            receive_request_meta_data();
            if (message.to_string()[0] == '{')
            {
                const std::string &message_str = message.to_string();
                if (message_str[1] == '}' && message_str.size() == 2)
                {
                    printf("[Server] Received close signal %s from socket %s.\n", message_str.c_str(), socket_addr.c_str());
                    send_data_vec = {{&worlds[world_name].time, conversion_map[attribute_map["time"].first][0]}};
                    receive_data_vec = {{&worlds[world_name].time, conversion_map[attribute_map["time"].first][0]}};
                    flag = EMultiverseServerState::SendResponseMetaData;
                }
                else if (reader.parse(message_str, request_meta_data_json) && !request_meta_data_json.empty())
                {
                    send_data_vec.clear();
                    receive_data_vec.clear();
                    flag = EMultiverseServerState::BindObjects;
                }
                else if (std::isnan(send_buffer[0]))
                {
                    printf("[Server] Received [%s] from socket %s.\n", message_str.c_str(), socket_addr.c_str());
                    flag = EMultiverseServerState::BindReceiveData;
                }
                else
                {
                    flag = EMultiverseServerState::BindSendData;
                }
            }
            else
            {
                flag = EMultiverseServerState::BindSendData;
            }

            break;
        }

        case EMultiverseServerState::BindObjects:
        {
            // printf("[Server] Received meta data from socket %s %s:\n%s", simulation_name.c_str(), socket_addr.c_str(), request_meta_data_json.toStyledString().c_str());
            bind_meta_data();

            mtx.lock();
            bind_send_objects();
            validate_meta_data();
            mtx.unlock();

            wait_for_objects();

            mtx.lock();
            bind_receive_objects();
            mtx.unlock();

            flag = EMultiverseServerState::SendResponseMetaData;
        }

        case EMultiverseServerState::SendResponseMetaData:
        {
            send_response_meta_data();
            // printf("[Server] Sent meta data to socket %s:\n%s", socket_addr.c_str(), response_meta_data_json.toStyledString().c_str());

            flag = EMultiverseServerState::ReceiveSendData;
            break;
        }

        case EMultiverseServerState::ReceiveSendData:
        {
            receive_send_data();

            if (message.to_string()[0] == '{')
            {
                const std::string &message_str = message.to_string();
                if (message_str[1] == '}' && message_str.size() == 2)
                {
                    printf("[Server] Received close signal %s from socket %s.\n", message_str.c_str(), socket_addr.c_str());
                    flag = EMultiverseServerState::SendResponseMetaData;
                }
                else if (reader.parse(message_str, request_meta_data_json) && !request_meta_data_json.empty())
                {
                    send_data_vec.clear();
                    receive_data_vec.clear();
                    flag = EMultiverseServerState::BindObjects;
                }
                else if (std::isnan(send_buffer[0]))
                {
                    printf("[Server] Received [%s] from socket %s.\n", message_str.c_str(), socket_addr.c_str());
                    flag = EMultiverseServerState::BindReceiveData;
                }
                else
                {
                    flag = EMultiverseServerState::BindSendData;
                }
            }
            else
            {
                flag = EMultiverseServerState::BindSendData;
            }

            break;
        }

        case EMultiverseServerState::BindSendData:
        {
            mtx.lock();
            if (send_buffer[0] >= 0.0)
            {
                *send_data_vec[0].first = send_buffer[0] * send_data_vec[0].second;
            }
            mtx.unlock();

            if (worlds[world_name].time == 0.0)
            {
                printf("[Server] Reset all simulations in world %s.\n", world_name.c_str());
                for (std::pair<const std::string, Simulation> &simulation : worlds[world_name].simulations)
                {
                    printf("[Server] Reset simulation %s.\n", simulation.first.c_str());
                    simulation.second.meta_data_state = EMetaDataState::Reset;
                }
            }

            if ((strcmp(request_world_name.c_str(), world_name.c_str()) != 0 || strcmp(request_simulation_name.c_str(), simulation_name.c_str()) != 0) && worlds[request_world_name].simulations.count(request_simulation_name) > 0)
            {
                double start = get_time_now();
                double now = get_time_now();
                EMetaDataState &request_meta_data_state = worlds[request_world_name].simulations[request_simulation_name].meta_data_state;
                while (!should_shut_down)
                {
                    if (request_meta_data_state == EMetaDataState::WaitAfterOtherBindSendData || request_meta_data_state == EMetaDataState::Normal)
                    {
                        break;
                    }
                    now = get_time_now();
                    if (now - start > 1)
                    {
                        printf("[Server] Socket %s is waiting for %s to send data.\n", socket_addr.c_str(), request_simulation_name.c_str());
                        start = now;
                    }
                }
                request_meta_data_state = EMetaDataState::WaitAfterOtherSendRequestMetaData;
            }

            mtx.lock();
            for (size_t i = 1; i < send_buffer_size; i++)
            {
                *send_data_vec[i].first = send_buffer[i] * send_data_vec[i].second;
            }
            mtx.unlock();

            flag = EMultiverseServerState::BindReceiveData;
            break;
        }

        case EMultiverseServerState::BindReceiveData:
        {
            wait_for_receive_data();

            mtx.lock();
            compute_cumulative_data();
            mtx.unlock();

            for (size_t i = 0; i < receive_buffer_size; i++)
            {
                receive_buffer[i] = *receive_data_vec[i].first * receive_data_vec[i].second;
            }
            flag = EMultiverseServerState::SendReceiveData;
            break;
        }

        case EMultiverseServerState::SendReceiveData:
        {
            send_receive_data();
            Simulation &simulation = worlds[world_name].simulations[simulation_name];
            if (simulation.meta_data_state == EMetaDataState::WaitAfterSendReceiveData)
            {
                printf("[Server] Socket %s has received new request meta data.\n", socket_addr.c_str());

                simulation.meta_data_state = EMetaDataState::WaitAfterOtherBindSendData;

                double start = get_time_now();
                double now = get_time_now();
                while (!should_shut_down)
                {
                    if (simulation.meta_data_state == EMetaDataState::WaitAfterOtherSendRequestMetaData)
                    {
                        break;
                    }

                    now = get_time_now();
                    if (now - start > 1)
                    {
                        printf("[Server] Socket %s is waiting for send data to be sent.\n", socket_addr.c_str());
                        start = now;
                    }
                }

                zmq::recv_result_t recv_result_t = socket.recv(message, zmq::recv_flags::none); // TODO: Make use of the message
                request_meta_data_json = simulation.request_meta_data_json;
                simulation.meta_data_state = EMetaDataState::Normal;

                send_data_vec.clear();
                receive_data_vec.clear();
                flag = EMultiverseServerState::BindObjects;
            }
            else
            {
                flag = EMultiverseServerState::ReceiveSendData;
            }

            break;
        }

        default:
            break;
        }
    }

    if (sockets_need_clean_up[socket_addr])
    {
        if (flag != EMultiverseServerState::ReceiveSendData && flag != EMultiverseServerState::ReceiveRequestMetaData)
        {
            send_receive_data();
        }

        printf("[Server] Unbind from socket %s.\n", socket_addr.c_str());
        try
        {
            socket.unbind(socket_addr);
        }
        catch (const zmq::error_t &e)
        {
            printf("[Server] %s, socket %s can not be unbinded.\n", e.what(), socket_addr.c_str());
        }
    }
}

void MultiverseServer::receive_request_meta_data()
{
    send_buffer_size = 1;
    receive_buffer_size = 1;

    send_buffer = (double *)calloc(send_buffer_size, sizeof(double));
    receive_buffer = (double *)calloc(receive_buffer_size, sizeof(double));

    is_receive_data_sent = false;

    // Receive JSON string over ZMQ
    try
    {
        sockets_need_clean_up[socket_addr] = false;
        printf("[Server] Socket %s is waiting for meta data.\n", socket_addr.c_str());
        zmq::recv_result_t recv_result_t = socket.recv(message, zmq::recv_flags::none);
        printf("[Server] Socket %s received meta data.\n", socket_addr.c_str());
        sockets_need_clean_up[socket_addr] = true;
    }
    catch (const zmq::error_t &e)
    {
        should_shut_down = true;
        printf("[Server] %s, socket %s prepares to close.\n", e.what(), socket_addr.c_str());
    }
}

void MultiverseServer::bind_meta_data()
{
    if (!request_meta_data_json.isMember("meta_data") || request_meta_data_json["meta_data"].empty())
    {
        throw std::invalid_argument("[Server] Request meta data from socket " + socket_addr + " doesn't have meta data.");
    }

    Json::Value &meta_data = request_meta_data_json["meta_data"];
    if (!meta_data.isMember("world_name") || meta_data["world_name"].asString().empty())
    {
        throw std::invalid_argument("[Server] Request meta data from socket " + socket_addr + " doesn't have a world name.");
    }
    request_world_name = meta_data["world_name"].asString();

    if (!meta_data.isMember("simulation_name") || meta_data["simulation_name"].asString().empty())
    {
        throw std::invalid_argument("[Server] Request meta data from socket " + socket_addr + " doesn't have a simulation name.");
    }
    request_simulation_name = meta_data["simulation_name"].asString();

    if (!simulation_name.empty() && worlds[request_world_name].simulations.count(request_simulation_name) == 0)
    {
        printf("[Server] Socket %s requests a non-existing simulation (%s).\n", socket_addr.c_str(), request_simulation_name.c_str());
    }

    if (request_simulation_name != simulation_name && !simulation_name.empty() && worlds[request_world_name].simulations.count(request_simulation_name) > 0)
    {
        printf("[Server] Socket %s (%s) requests a different simulation (%s).\n", socket_addr.c_str(), simulation_name.c_str(), request_simulation_name.c_str());
        Simulation &request_simulation = worlds[request_world_name].simulations[request_simulation_name];
        for (const std::string &type_str : {"send", "receive"})
        {
            for (const std::string &object_name : request_meta_data_json[type_str].getMemberNames())
            {
                if (object_name.empty())
                {
                    break;
                }

                for (const Json::Value &attribute : request_meta_data_json[type_str][object_name])
                {
                    if (attribute.asString().empty())
                    {
                        break;
                    }

                    Json::Value &attributes = request_simulation.request_meta_data_json[type_str][object_name];
                    if (std::find(attributes.begin(), attributes.end(), attribute) == attributes.end())
                    {
                        attributes.append(attribute);
                    }
                }
            }
        }

        request_simulation.meta_data_state = EMetaDataState::WaitAfterSendReceiveData;
        world_name = request_world_name;
    }
    else
    {
        world_name = request_world_name;
        simulation_name = request_simulation_name;
    }

    worlds[world_name].simulations[simulation_name].request_meta_data_json = request_meta_data_json;
    worlds[world_name].simulations[simulation_name].meta_data_state = EMetaDataState::Normal;

    const std::string length_unit = meta_data.isMember("length_unit") ? meta_data["length_unit"].asString() : "m";
    const std::string angle_unit = meta_data.isMember("angle_unit") ? meta_data["angle_unit"].asString() : "rad";
    const std::string handedness = meta_data.isMember("handedness") ? meta_data["handedness"].asString() : "rhs";
    const std::string mass_unit = meta_data.isMember("mass_unit") ? meta_data["mass_unit"].asString() : "kg";
    const std::string time_unit = meta_data.isMember("time_unit") ? meta_data["time_unit"].asString() : "s";

    for (const std::pair<const std::string, std::pair<EAttribute, std::vector<double>>> &attribute : attribute_map)
    {
        conversion_map.emplace(attribute.second);
    }

    std::for_each(conversion_map[EAttribute::Time].begin(), conversion_map[EAttribute::Time].end(),
                  [time_unit](double &time)
                  { time = unit_scale[time_unit]; });

    std::for_each(conversion_map[EAttribute::Position].begin(), conversion_map[EAttribute::Position].end(),
                  [length_unit](double &position)
                  { position = unit_scale[length_unit]; });

    std::for_each(conversion_map[EAttribute::Quaternion].begin(), conversion_map[EAttribute::Quaternion].end(),
                  [](double &quaternion)
                  { quaternion = 1.0; });

    std::for_each(conversion_map[EAttribute::JointRvalue].begin(), conversion_map[EAttribute::JointRvalue].end(),
                  [angle_unit](double &joint_rvalue)
                  { joint_rvalue = unit_scale[angle_unit]; });

    std::for_each(conversion_map[EAttribute::JointTvalue].begin(), conversion_map[EAttribute::JointTvalue].end(),
                  [length_unit](double &joint_tvalue)
                  { joint_tvalue = unit_scale[length_unit]; });

    std::for_each(conversion_map[EAttribute::JointLinearVelocity].begin(), conversion_map[EAttribute::JointLinearVelocity].end(),
                  [length_unit, time_unit](double &joint_linear_velocity)
                  { joint_linear_velocity = unit_scale[length_unit] / unit_scale[time_unit]; });

    std::for_each(conversion_map[EAttribute::JointAngularVelocity].begin(), conversion_map[EAttribute::JointAngularVelocity].end(),
                  [angle_unit, time_unit](double &joint_angular_velocity)
                  { joint_angular_velocity = unit_scale[angle_unit] / unit_scale[time_unit]; });

    std::for_each(conversion_map[EAttribute::JointForce].begin(), conversion_map[EAttribute::JointForce].end(),
                  [mass_unit, length_unit, time_unit](double &force)
                  { force = unit_scale[mass_unit] * unit_scale[length_unit] / (unit_scale[time_unit] * unit_scale[time_unit]); });

    std::for_each(conversion_map[EAttribute::JointTorque].begin(), conversion_map[EAttribute::JointTorque].end(),
                  [mass_unit, length_unit, time_unit](double &torque)
                  { torque = unit_scale[mass_unit] * unit_scale[length_unit] * unit_scale[length_unit] / (unit_scale[time_unit] * unit_scale[time_unit]); });

    std::for_each(conversion_map[EAttribute::JointPosition].begin(), conversion_map[EAttribute::JointPosition].end(),
                  [length_unit](double &joint_position)
                  { joint_position = unit_scale[length_unit]; });

    std::for_each(conversion_map[EAttribute::JointQuaternion].begin(), conversion_map[EAttribute::JointQuaternion].end(),
                  [](double &joint_quaternion)
                  { joint_quaternion = 1.0; });

    std::for_each(conversion_map[EAttribute::Force].begin(), conversion_map[EAttribute::Force].end(),
                  [mass_unit, length_unit, time_unit](double &force)
                  { force = unit_scale[mass_unit] * unit_scale[length_unit] / (unit_scale[time_unit] * unit_scale[time_unit]); });

    std::for_each(conversion_map[EAttribute::Torque].begin(), conversion_map[EAttribute::Torque].end(),
                  [mass_unit, length_unit, time_unit](double &torque)
                  { torque = unit_scale[mass_unit] * unit_scale[length_unit] * unit_scale[length_unit] / (unit_scale[time_unit] * unit_scale[time_unit]); });

    for (size_t i = 0; i < 3; i++)
    {
        conversion_map[EAttribute::RelativeVelocity][i] = unit_scale[length_unit] / unit_scale[time_unit];
    }
    for (size_t i = 3; i < 6; i++)
    {
        conversion_map[EAttribute::RelativeVelocity][i] = unit_scale[angle_unit] / unit_scale[time_unit];
    }

    conversion_map[EAttribute::CmdJointRvalue] = conversion_map[EAttribute::JointRvalue];

    conversion_map[EAttribute::CmdJointTvalue] = conversion_map[EAttribute::JointTvalue];

    conversion_map[EAttribute::CmdJointLinearVelocity] = conversion_map[EAttribute::JointLinearVelocity];

    conversion_map[EAttribute::CmdJointAngularVelocity] = conversion_map[EAttribute::JointAngularVelocity];

    conversion_map[EAttribute::CmdJointForce] = conversion_map[EAttribute::Force];

    conversion_map[EAttribute::CmdJointTorque] = conversion_map[EAttribute::Torque];

    conversion_map[EAttribute::OdometricVelocity] = conversion_map[EAttribute::RelativeVelocity];

    for (std::pair<const EAttribute, std::vector<double>> &conversion_scale : conversion_map)
    {
        std::vector<double>::iterator conversion_scale_it = conversion_scale.second.begin();
        std::vector<double>::iterator handedness_scale_it = handedness_scale[conversion_scale.first][handedness].begin();
        for (size_t i = 0; i < conversion_scale.second.size(); i++)
        {
            *(conversion_scale_it++) *= *(handedness_scale_it++);
        }
    }

    response_meta_data_json.clear();
    response_meta_data_json["meta_data"] = meta_data;
    response_meta_data_json["time"] = worlds[world_name].time * unit_scale[time_unit];
}

void MultiverseServer::bind_send_objects()
{
    send_objects_json = request_meta_data_json["send"];
    std::map<std::string, Object> &objects = worlds[world_name].objects;
    Simulation &simulation = worlds[world_name].simulations[simulation_name];
    send_data_vec.emplace_back(&worlds[world_name].time, conversion_map[attribute_map["time"].first][0]);

    for (const std::string &object_name : send_objects_json.getMemberNames())
    {
        Object &object = objects[object_name];
        simulation.objects[object_name] = &object;
        for (const Json::Value &attribute_json : send_objects_json[object_name])
        {
            const std::string &attribute_name = attribute_json.asString();
            Attribute &attribute = object.attributes[attribute_name];
            if (attribute.data.size() == 0 && cumulative_attribute_names.count(attribute_name) == 0)
            {
                attribute.data = attribute_map[attribute_name].second;
                for (size_t i = 0; i < attribute.data.size(); i++)
                {
                    double *data = &attribute.data[i];
                    const double conversion = conversion_map[attribute_map[attribute_name].first][i];
                    send_data_vec.emplace_back(data, conversion);
                    response_meta_data_json["send"][object_name][attribute_name].append(attribute_map[attribute_name].second[i]);
                }
            }
            else if (cumulative_attribute_names.count(attribute_name) > 0)
            {
                std::vector<double> &simulation_data = attribute.simulation_data[simulation_name];
                if (simulation_data.size() == 0)
                {
                    simulation_data = attribute_map[attribute_name].second;
                }

                for (size_t i = 0; i < simulation_data.size(); i++)
                {
                    double *data = &simulation_data[i];
                    const double conversion = conversion_map[attribute_map[attribute_name].first][i];
                    send_data_vec.emplace_back(data, conversion);
                    response_meta_data_json["send"][object_name][attribute_name].append(*data * conversion);
                }
            }
            else
            {
                printf("[Server] Continue state [%s - %s] on socket %s\n", object_name.c_str(), attribute_name.c_str(), socket_addr.c_str());
                continue_state = true;
                attribute.is_sent = true;

                for (size_t i = 0; i < attribute.data.size(); i++)
                {
                    double *data = &attribute.data[i];
                    const double conversion = conversion_map[attribute_map[attribute_name].first][i];
                    send_data_vec.emplace_back(data, conversion);
                    response_meta_data_json["send"][object_name][attribute_name].append(*data * conversion);
                }
            }
        }
    }
}

void MultiverseServer::validate_meta_data()
{
    receive_objects_json = request_meta_data_json["receive"];

    if (receive_objects_json.isMember("") &&
        std::find(receive_objects_json[""].begin(), receive_objects_json[""].end(), "") != receive_objects_json[""].end())
    {
        receive_objects_json = {};
        for (const std::pair<std::string, Object> &object : worlds[world_name].objects)
        {
            for (const std::pair<std::string, Attribute> &attribute_pair : object.second.attributes)
            {
                receive_objects_json[object.first].append(attribute_pair.first);
            }
        }
        return;
    }

    for (const std::string &object_name : request_meta_data_json["receive"].getMemberNames())
    {
        if (!object_name.empty())
        {
            for (const Json::Value &attribute_json : request_meta_data_json["receive"][object_name])
            {
                const std::string &attribute_name = attribute_json.asString();
                if (!attribute_name.empty())
                {
                    continue;
                }

                receive_objects_json[object_name] = {};
                for (const std::pair<std::string, Attribute> &attribute_pair : worlds[world_name].objects[object_name].attributes)
                {
                    receive_objects_json[object_name].append(attribute_pair.first);
                }
                break;
            }
        }
        else
        {
            for (const Json::Value &attribute_json : request_meta_data_json["receive"][object_name])
            {
                const std::string &attribute_name = attribute_json.asString();
                for (const std::pair<std::string, Object> &object_pair : worlds[world_name].objects)
                {
                    if (object_pair.second.attributes.count(attribute_name) > 0)
                    {
                        receive_objects_json[object_pair.first].append(attribute_name);
                    }
                }
            }
            receive_objects_json.removeMember(object_name);
            break;
        }
    }
}

void MultiverseServer::wait_for_objects()
{
    double start = get_time_now();
    double now = get_time_now();
    bool found_all_objects = true;
    do
    {
        found_all_objects = true;
        now = get_time_now();
        for (const std::string &object_name : receive_objects_json.getMemberNames())
        {
            for (const Json::Value &attribute_json : receive_objects_json[object_name])
            {
                const std::string &attribute_name = attribute_json.asString();
                if ((worlds[world_name].objects.count(object_name) == 0 || worlds[world_name].objects[object_name].attributes.count(attribute_name) == 0))
                {
                    found_all_objects = false;
                    if (now - start > 1)
                    {
                        printf("[Server] Socket %s is waiting for [%s][%s][%s] to be declared.\n", socket_addr.c_str(), world_name.c_str(), object_name.c_str(), attribute_name.c_str());
                    }
                }
            }
        }
        if (now - start > 1)
        {
            start = now;
        }
    } while (!should_shut_down && !found_all_objects);
}

void MultiverseServer::bind_receive_objects()
{
    receive_data_vec.emplace_back(&worlds[world_name].time, conversion_map[attribute_map["time"].first][0]);
    for (const std::string &object_name : receive_objects_json.getMemberNames())
    {
        for (const Json::Value &attribute_json : receive_objects_json[object_name])
        {
            const std::string attribute_name = attribute_json.asString();
            Attribute &attribute = worlds[world_name].objects[object_name].attributes[attribute_name];
            if (cumulative_attribute_names.count(attribute_name) > 0)
            {
                if (attribute.data.size() == 0)
                {
                    attribute.data = attribute_map[attribute_name].second;
                    attribute.is_sent = true;
                }
            }

            for (size_t i = 0; i < attribute.data.size(); i++)
            {
                double *data = &attribute.data[i];
                const double conversion = 1.0 / conversion_map[attribute_map[attribute_name].first][i];
                receive_data_vec.emplace_back(data, conversion);
                response_meta_data_json["receive"][object_name][attribute_name].append(*data * conversion);
            }
        }
    }
}

void MultiverseServer::send_response_meta_data()
{
    send_buffer_size = send_data_vec.size();
    receive_buffer_size = receive_data_vec.size();

    if (should_shut_down)
    {
        response_meta_data_json["time"] = -1.0;
    }

    if (continue_state)
    {
        continue_state = false;
    }

    const std::string message_str = response_meta_data_json.toStyledString();

    // Send buffer sizes and send_data (if exists) over ZMQ
    zmq::message_t response_message(message_str.size());
    memcpy(response_message.data(), message_str.data(), message_str.size());
    socket.send(response_message, zmq::send_flags::none);

    send_buffer = (double *)calloc(send_buffer_size, sizeof(double));
    receive_buffer = (double *)calloc(receive_buffer_size, sizeof(double));
}

void MultiverseServer::receive_send_data()
{
    // Receive send_data over ZMQ
    try
    {
        sockets_need_clean_up[socket_addr] = false;
        zmq::recv_result_t recv_result_t = socket.recv(message, zmq::recv_flags::none);
        sockets_need_clean_up[socket_addr] = true;
        if (message.to_string()[0] != '{' && message.to_string()[1] != '}')
        {
            memcpy(send_buffer, message.data(), send_buffer_size * sizeof(double));
        }
    }
    catch (const zmq::error_t &e)
    {
        should_shut_down = true;
        printf("[Server] %s, socket %s prepares to close.\n", e.what(), socket_addr.c_str());
    }
}

void MultiverseServer::wait_for_receive_data()
{
    for (const std::string &object_name : send_objects_json.getMemberNames())
    {
        for (const Json::Value &attribute_json : send_objects_json[object_name])
        {
            const std::string attribute_name = attribute_json.asString();
            worlds[world_name].objects[object_name].attributes[attribute_name].is_sent = true;
        }
    }

    if (!is_receive_data_sent)
    {
        for (const std::string &object_name : receive_objects_json.getMemberNames())
        {
            for (const Json::Value &attribute_json : receive_objects_json[object_name])
            {
                const std::string attribute_name = attribute_json.asString();
                double start = get_time_now();
                while ((worlds[world_name].objects.count(object_name) == 0 || worlds[world_name].objects[object_name].attributes.count(attribute_name) == 0 || !worlds[world_name].objects[object_name].attributes[attribute_name].is_sent) && !should_shut_down)
                {
                    const double now = get_time_now();
                    if (now - start > 1)
                    {
                        printf("[Server] Socket %s is waiting for data of [%s][%s][%s] to be sent.\n", socket_addr.c_str(), world_name.c_str(), object_name.c_str(), attribute_name.c_str());
                        start = now;
                    }
                }
            }
        }

        is_receive_data_sent = true;
    }
}

void MultiverseServer::compute_cumulative_data()
{
    for (const std::string &object_name : receive_objects_json.getMemberNames())
    {
        for (const std::string &attribute_name : cumulative_attribute_names)
        {
            if (std::find(receive_objects_json[object_name].begin(), receive_objects_json[object_name].end(), attribute_name) == receive_objects_json[object_name].end())
            {
                continue;
            }

            std::vector<double> &data = worlds[world_name].objects[object_name].attributes[attribute_name].data;
            data = attribute_map[attribute_name].second;
            for (size_t i = 0; i < data.size(); i++)
            {
                for (std::pair<const std::string, Simulation> &simulation_pair : worlds[world_name].simulations)
                {
                    if (simulation_pair.second.objects.count(object_name) == 0)
                    {
                        continue;
                    }

                    const std::string &simulation_name = simulation_pair.first;
                    const std::vector<double> &simulation_data = (*simulation_pair.second.objects[object_name]).attributes[attribute_name].simulation_data[simulation_name];
                    if (simulation_data.size() != data.size())
                    {
                        continue;
                    }

                    data[i] += simulation_data[i];
                }
            }
        }
    }
}

void MultiverseServer::send_receive_data()
{
    // Send receive_data over ZMQ
    if (should_shut_down)
    {
        receive_buffer[0] = -1.0;
    }
    else if (worlds[world_name].simulations[simulation_name].meta_data_state == EMetaDataState::WaitAfterSendReceiveData)
    {
        receive_buffer[0] = -2.0;
    }
    else if (worlds[world_name].simulations[simulation_name].meta_data_state == EMetaDataState::Reset)
    {
        receive_buffer[0] = 0.0;
        worlds[world_name].simulations[simulation_name].meta_data_state = EMetaDataState::Normal;
    }

    zmq::message_t reply_data(receive_buffer_size * sizeof(double));
    memcpy(reply_data.data(), receive_buffer, receive_buffer_size * sizeof(double));
    socket.send(reply_data, zmq::send_flags::none);
}

void start_multiverse_server(const std::string &server_socket_addr)
{
    std::map<std::string, std::thread> workers;
    zmq::socket_t server_socket = zmq::socket_t(server_context, zmq::socket_type::rep);
    server_socket.bind(server_socket_addr);
    printf("[Server] Create server socket %s\n", server_socket_addr.c_str());

    std::string receive_addr;
    while (!should_shut_down)
    {
        try
        {
            zmq::message_t request;
            printf("[Server] Waiting for request...\n");
            zmq::recv_result_t recv_result_t = server_socket.recv(request, zmq::recv_flags::none);
            receive_addr = request.to_string();
            printf("[Server] Received request from %s.\n", receive_addr.c_str());
        }
        catch (const zmq::error_t &e)
        {
            should_shut_down = true;
            printf("[Server] %s, server socket %s prepares to close.\n", e.what(), server_socket_addr.c_str());
            break;
        }

        if (workers.count(receive_addr) == 0)
        {
            workers[receive_addr] = std::thread([receive_addr]()
                                                { MultiverseServer multiverse_server(receive_addr); multiverse_server.start(); });
        }

        zmq::message_t response(receive_addr.size());
        memcpy(response.data(), receive_addr.c_str(), receive_addr.size());
        printf("[Server] Sending response to %s.\n", receive_addr.c_str());
        server_socket.send(response, zmq::send_flags::none);
        printf("[Server] Sent response to %s.\n", receive_addr.c_str());
    }

    for (std::pair<const std::string, std::thread> &worker : workers)
    {
        worker.second.join();
    }
}