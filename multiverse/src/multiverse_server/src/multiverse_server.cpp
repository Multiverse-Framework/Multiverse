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
#include <zmq_addon.hpp>

#include "multiverse_server.h"

#define STRING_SIZE 2000

using namespace std::chrono_literals;

bool should_shut_down = false;
std::map<std::string, bool> sockets_need_clean_up;
zmq::context_t server_context{1};

std::set<std::string> cumulative_attribute_names = {"force", "torque"};

std::map<std::string, std::pair<EAttribute, std::vector<double>>> attribute_map_double =
    {
        {"time", {EAttribute::Time, {0.0}}},
        {"scalar", {EAttribute::Scalar, {std::numeric_limits<double>::quiet_NaN()}}},
        {"position", {EAttribute::Position, std::vector<double>(3, std::numeric_limits<double>::quiet_NaN())}},
        {"quaternion", {EAttribute::Quaternion, std::vector<double>(4, std::numeric_limits<double>::quiet_NaN())}},
        {"relative_velocity", {EAttribute::RelativeVelocity, std::vector<double>(6, 0.0)}},
        {"odometric_velocity", {EAttribute::OdometricVelocity, std::vector<double>(6, 0.0)}},
        {"joint_rvalue", {EAttribute::JointRvalue, {std::numeric_limits<double>::quiet_NaN()}}},
        {"joint_tvalue", {EAttribute::JointTvalue, {std::numeric_limits<double>::quiet_NaN()}}},
        {"joint_linear_velocity", {EAttribute::JointLinearVelocity, {std::numeric_limits<double>::quiet_NaN()}}},
        {"joint_angular_velocity", {EAttribute::JointAngularVelocity, {std::numeric_limits<double>::quiet_NaN()}}},
        {"joint_linear_acceleration", {EAttribute::JointLinearAcceleration, {std::numeric_limits<double>::quiet_NaN()}}},
        {"joint_angular_acceleration", {EAttribute::JointAngularAcceleration, {std::numeric_limits<double>::quiet_NaN()}}},
        {"joint_force", {EAttribute::JointForce, {std::numeric_limits<double>::quiet_NaN()}}},
        {"joint_torque", {EAttribute::JointTorque, {std::numeric_limits<double>::quiet_NaN()}}},
        {"cmd_joint_rvalue", {EAttribute::CmdJointRvalue, {std::numeric_limits<double>::quiet_NaN()}}},
        {"cmd_joint_tvalue", {EAttribute::CmdJointTvalue, {std::numeric_limits<double>::quiet_NaN()}}},
        {"cmd_joint_linear_velocity", {EAttribute::CmdJointLinearVelocity, {std::numeric_limits<double>::quiet_NaN()}}},
        {"cmd_joint_angular_velocity", {EAttribute::CmdJointAngularVelocity, {std::numeric_limits<double>::quiet_NaN()}}},
        {"cmd_joint_linear_acceleration", {EAttribute::CmdJointLinearAcceleration, {std::numeric_limits<double>::quiet_NaN()}}},
        {"cmd_joint_angular_acceleration", {EAttribute::CmdJointAngularAcceleration, {std::numeric_limits<double>::quiet_NaN()}}},
        {"cmd_joint_force", {EAttribute::CmdJointForce, {std::numeric_limits<double>::quiet_NaN()}}},
        {"cmd_joint_torque", {EAttribute::CmdJointTorque, {std::numeric_limits<double>::quiet_NaN()}}},
        {"joint_position", {EAttribute::JointPosition, std::vector<double>(3, std::numeric_limits<double>::quiet_NaN())}},
        {"joint_quaternion", {EAttribute::JointQuaternion, std::vector<double>(4, std::numeric_limits<double>::quiet_NaN())}},
        {"force", {EAttribute::Force, std::vector<double>(3, 0.0)}},
        {"torque", {EAttribute::Torque, std::vector<double>(3, 0.0)}}};

std::map<std::string, std::pair<EAttribute, std::vector<uint8_t>>> attribute_map_uint8_t =
    {
        {"rgb_3840_2160", {EAttribute::RGB_3840_2160, std::vector<uint8_t>(3840 * 2160 * 3, std::numeric_limits<uint8_t>::quiet_NaN())}},
        {"rgb_1280_1024", {EAttribute::RGB_1280_1024, std::vector<uint8_t>(1280 * 1024 * 3, std::numeric_limits<uint8_t>::quiet_NaN())}},
        {"rgb_640_480", {EAttribute::RGB_640_480, std::vector<uint8_t>(640 * 480 * 3, std::numeric_limits<uint8_t>::quiet_NaN())}},
        {"rgb_128_128", {EAttribute::RGB_128_128, std::vector<uint8_t>(128 * 128 * 3, std::numeric_limits<uint8_t>::quiet_NaN())}}};

std::map<std::string, std::pair<EAttribute, std::vector<uint16_t>>> attribute_map_uint16_t =
    {
        {"depth_3840_2160", {EAttribute::Depth_3840_2160, std::vector<uint16_t>(3840 * 2160, std::numeric_limits<uint16_t>::quiet_NaN())}},
        {"depth_1280_1024", {EAttribute::Depth_1280_1024, std::vector<uint16_t>(1280 * 1024, std::numeric_limits<uint16_t>::quiet_NaN())}},
        {"depth_640_480", {EAttribute::Depth_640_480, std::vector<uint16_t>(640 * 480, std::numeric_limits<uint16_t>::quiet_NaN())}},
        {"depth_128_128", {EAttribute::Depth_128_128, std::vector<uint16_t>(128 * 128, std::numeric_limits<uint16_t>::quiet_NaN())}}};

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
        {EAttribute::Scalar,
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
        {EAttribute::JointLinearAcceleration,
         {{"rhs", {1.0}},
          {"lhs", {1.0}}}},
        {EAttribute::JointAngularAcceleration,
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
        {EAttribute::CmdJointLinearAcceleration,
         {{"rhs", {1.0}},
          {"lhs", {1.0}}}},
        {EAttribute::CmdJointAngularAcceleration,
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
    WaitAfterOtherSendRequestMetaData,
    WaitAfterOtherNormal
};

std::mutex mtx;

template <class T>
struct TypedAttribute
{
    std::vector<T> data;
    std::map<std::string, std::vector<T>> simulation_data;
    bool is_sent = false;
};

struct Attribute
{
    TypedAttribute<double> attribute_double;
    TypedAttribute<uint8_t> attribute_uint8_t;
    TypedAttribute<uint16_t> attribute_uint16_t;
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
    std::vector<std::map<std::string, std::vector<std::string>>> api_callbacks;
    std::vector<std::map<std::string, std::vector<std::string>>> api_callbacks_response;
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

static Json::Value sort_json_array(const Json::Value &original)
{
    std::vector<std::string> vec;
    for (const auto &item : original)
    {
        vec.push_back(item.asString());
    }

    std::sort(vec.begin(), vec.end());

    Json::Value sorted;
    for (const auto &item : vec)
    {
        sorted.append(item);
    }

    return sorted;
}

static Json::Value sort_json_by_key(const Json::Value &original)
{
    std::vector<std::string> keys;
    for (const std::string &key : original.getMemberNames())
    {
        keys.push_back(key);
    }
    std::sort(keys.begin(), keys.end()); // Sort keys alphabetically

    Json::Value sorted(Json::objectValue);
    for (const std::string &key : keys)
    {
        sorted[key] = original[key].isObject() ? sort_json_by_key(original[key]) : original[key].isArray() ? sort_json_array(original[key])
                                                                                                           : original[key];
    }
    return sorted;
}

static Json::Value sort_meta_data_json(const Json::Value &original)
{
    Json::Value sorted(original);
    sorted["send"] = sort_json_by_key(sorted["send"]);
    sorted["receive"] = sort_json_by_key(sorted["receive"]);
    return sorted;
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

    if (send_buffer.buffer_double.data != nullptr)
    {
        free(send_buffer.buffer_double.data);
    }
    if (send_buffer.buffer_uint8_t.data != nullptr)
    {
        free(send_buffer.buffer_uint8_t.data);
    }
    if (send_buffer.buffer_uint16_t.data != nullptr)
    {
        free(send_buffer.buffer_uint16_t.data);
    }
    if (receive_buffer.buffer_double.data != nullptr)
    {
        free(receive_buffer.buffer_double.data);
    }
    if (receive_buffer.buffer_uint8_t.data != nullptr)
    {
        free(receive_buffer.buffer_uint8_t.data);
    }
    if (receive_buffer.buffer_uint16_t.data != nullptr)
    {
        free(receive_buffer.buffer_uint16_t.data);
    }

    printf("[Server] Clean up socket %s.\n", socket_addr.c_str());

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
            send_buffer.buffer_double.size = 0;
            send_buffer.buffer_uint8_t.size = 0;
            send_buffer.buffer_uint16_t.size = 0;
            receive_buffer.buffer_double.size = 0;
            receive_buffer.buffer_uint8_t.size = 0;
            receive_buffer.buffer_uint16_t.size = 0;

            is_receive_data_sent = false;

            flag = receive_data();
            break;
        }

        case EMultiverseServerState::BindObjects:
        {
            // printf("[Server] Received meta data at socket %s:\n%s", socket_addr.c_str(), request_meta_data_json.toStyledString().c_str());
            bind_meta_data();

            mtx.lock();
            bind_send_objects();
            validate_meta_data();
            mtx.unlock();

            wait_for_objects();

            if (should_shut_down)
            {
                break;
            }

            mtx.lock();
            bind_receive_objects();
            mtx.unlock();

            if (request_meta_data_json.isMember("api_callbacks") && !request_meta_data_json["api_callbacks"].empty())
            {
                wait_for_api_callbacks_response();
            }

            flag = EMultiverseServerState::SendResponseMetaData;
        }

        case EMultiverseServerState::SendResponseMetaData:
        {
            send_response_meta_data();
            init_send_and_receive_data();
            // printf("[Server] Sent meta data to socket %s:\n%s", socket_addr.c_str(), response_meta_data_json.toStyledString().c_str());

            flag = EMultiverseServerState::ReceiveSendData;
            break;
        }

        case EMultiverseServerState::ReceiveSendData:
        {
            flag = receive_data();
            break;
        }

        case EMultiverseServerState::BindSendData:
        {
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
                wait_for_other_send_data();
            }

            mtx.lock();
            bind_send_data();
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

            bind_receive_data();

            flag = EMultiverseServerState::SendReceiveData;
            break;
        }

        case EMultiverseServerState::SendReceiveData:
        {
            Simulation &simulation = worlds[world_name].simulations[simulation_name];
            if (simulation.meta_data_state == EMetaDataState::WaitAfterSendReceiveData)
            {
                receive_new_request_meta_data();

                flag = EMultiverseServerState::BindObjects;
            }
            else
            {
                if ((strcmp(request_world_name.c_str(), world_name.c_str()) != 0 || strcmp(request_simulation_name.c_str(), simulation_name.c_str()) != 0) && worlds[request_world_name].simulations.count(request_simulation_name) > 0)
                {
                    wait_for_other_send_data();
                }
                send_receive_data();

                flag = EMultiverseServerState::ReceiveSendData;

                simulation.meta_data_state = EMetaDataState::Normal;
            }

            break;
        }

        default:
            break;
        }
    }

    if (sockets_need_clean_up[socket_addr])
    {
        if (flag == EMultiverseServerState::BindSendData)
        {
            receive_data();
        }
        if (flag != EMultiverseServerState::ReceiveSendData &&
            flag != EMultiverseServerState::ReceiveRequestMetaData)
        {
            try
            {
                send_receive_data();
            }
            catch (const zmq::error_t &e)
            {
                printf("[Server] %s, socket %s is terminated.\n", e.what(), socket_addr.c_str());
                return;
            }
        }

        printf("[Server] Unbind socket %s.\n", socket_addr.c_str());
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

EMultiverseServerState MultiverseServer::receive_data()
{
    try
    {
        std::vector<zmq::message_t> request_array;
        sockets_need_clean_up[socket_addr] = false;
        zmq::recv_result_t recv_result_t = zmq::recv_multipart(socket, std::back_inserter(request_array), zmq::recv_flags::none);
        assert(recv_result_t.has_value());
        sockets_need_clean_up[socket_addr] = true;

        const size_t request_array_size = request_array.size();
        if (request_array_size == 0)
        {
            throw std::invalid_argument("[Server] Received empty message at socket " + socket_addr + ".");
        }
        else
        {
            int message_spec_int;
            memcpy(&message_spec_int, request_array[0].data(), sizeof(int));
            if (message_spec_int == 0 && request_array_size == 1)
            {
                printf("[Server] Received close signal at socket %s.\n", socket_addr.c_str());
                send_response_meta_data();
                worlds[world_name].simulations[simulation_name].meta_data_state = EMetaDataState::Normal;
                return EMultiverseServerState::ReceiveRequestMetaData;
            }
            else if (message_spec_int == 1 && request_array_size == 2)
            {
                if (reader.parse(request_array[1].to_string(), request_meta_data_json) && !request_meta_data_json.empty())
                {
                    request_meta_data_json = sort_meta_data_json(request_meta_data_json);
                    send_buffer.buffer_double.data_vec.clear();
                    send_buffer.buffer_uint8_t.data_vec.clear();
                    send_buffer.buffer_uint16_t.data_vec.clear();
                    receive_buffer.buffer_double.data_vec.clear();
                    receive_buffer.buffer_uint8_t.data_vec.clear();
                    receive_buffer.buffer_uint16_t.data_vec.clear();
                    return EMultiverseServerState::BindObjects;
                }
                else
                {
                    throw std::invalid_argument("[Server] Received invalid message [" + request_array[1].to_string() + "] at socket " + socket_addr + ".");
                }
            }
            else if ((message_spec_int == 2 && request_array_size == 2) ||
                     (message_spec_int == 3 && request_array_size == 3) ||
                     (message_spec_int == 4 && request_array_size == 4) ||
                     (message_spec_int == 5 && request_array_size == 5))
            {
                memcpy(&worlds[world_name].time, request_array[1].data(), sizeof(double));

                if (worlds[world_name].time < 0.0)
                {
                    throw std::invalid_argument("[Server] Received invalid message [time = " + std::to_string(worlds[world_name].time) + "] at socket " + socket_addr + ".");
                }

                if (message_spec_int == 3 && request_array_size == 3)
                {
                    if (send_buffer.buffer_double.size > 0 && send_buffer.buffer_uint8_t.size == 0 && send_buffer.buffer_uint16_t.size == 0)
                    {
                        memcpy(send_buffer.buffer_double.data, request_array[2].data(), send_buffer.buffer_double.size * sizeof(double));
                    }
                    else if (send_buffer.buffer_double.size == 0 && send_buffer.buffer_uint8_t.size > 0 && send_buffer.buffer_uint16_t.size == 0)
                    {
                        memcpy(send_buffer.buffer_uint8_t.data, request_array[2].data(), send_buffer.buffer_uint8_t.size * sizeof(uint8_t));
                    }
                    else if (send_buffer.buffer_double.size == 0 && send_buffer.buffer_uint8_t.size == 0 && send_buffer.buffer_uint16_t.size > 0)
                    {
                        memcpy(send_buffer.buffer_uint16_t.data, request_array[2].data(), send_buffer.buffer_uint16_t.size * sizeof(uint16_t));
                    }
                    else
                    {
                        throw std::invalid_argument("[Server] Received invalid message [message_spec_int = " + std::to_string(message_spec_int) + ", request_array_size = " + std::to_string(request_array_size) + "] at socket " + socket_addr + ".");
                    }
                }
                else if (message_spec_int == 4 && request_array_size == 4)
                {
                    if (send_buffer.buffer_double.size > 0 && send_buffer.buffer_uint8_t.size > 0 && send_buffer.buffer_uint16_t.size == 0)
                    {
                        memcpy(send_buffer.buffer_double.data, request_array[2].data(), send_buffer.buffer_double.size * sizeof(double));
                        memcpy(send_buffer.buffer_uint8_t.data, request_array[3].data(), send_buffer.buffer_uint8_t.size * sizeof(uint8_t));
                    }
                    else if (send_buffer.buffer_double.size > 0 && send_buffer.buffer_uint8_t.size == 0 && send_buffer.buffer_uint16_t.size > 0)
                    {
                        memcpy(send_buffer.buffer_double.data, request_array[2].data(), send_buffer.buffer_double.size * sizeof(double));
                        memcpy(send_buffer.buffer_uint16_t.data, request_array[3].data(), send_buffer.buffer_uint16_t.size * sizeof(uint16_t));
                    }
                    else if (send_buffer.buffer_double.size == 0 && send_buffer.buffer_uint8_t.size > 0 && send_buffer.buffer_uint16_t.size > 0)
                    {
                        memcpy(send_buffer.buffer_uint8_t.data, request_array[2].data(), send_buffer.buffer_uint8_t.size * sizeof(uint8_t));
                        memcpy(send_buffer.buffer_uint16_t.data, request_array[3].data(), send_buffer.buffer_uint16_t.size * sizeof(uint16_t));
                    }
                    else
                    {
                        throw std::invalid_argument("[Server] Received invalid message [message_spec_int = " + std::to_string(message_spec_int) + ", request_array_size = " + std::to_string(request_array_size) + "] at socket " + socket_addr + ".");
                    }
                }
                else if (message_spec_int == 5 && request_array_size == 5)
                {
                    if (send_buffer.buffer_double.size > 0 && send_buffer.buffer_uint8_t.size > 0 && send_buffer.buffer_uint16_t.size > 0)
                    {
                        memcpy(send_buffer.buffer_double.data, request_array[2].data(), send_buffer.buffer_double.size * sizeof(double));
                        memcpy(send_buffer.buffer_uint8_t.data, request_array[3].data(), send_buffer.buffer_uint8_t.size * sizeof(uint8_t));
                        memcpy(send_buffer.buffer_uint16_t.data, request_array[4].data(), send_buffer.buffer_uint16_t.size * sizeof(uint16_t));
                    }
                    else
                    {
                        throw std::invalid_argument("[Server] Received invalid message [message_spec_int = " + std::to_string(message_spec_int) + ", request_array_size = " + std::to_string(request_array_size) + "] at socket " + socket_addr + ".");
                    }
                }
                return EMultiverseServerState::BindSendData;
            }
            else
            {
                throw std::invalid_argument("[Server] Received invalid message [message_spec_int = " + std::to_string(message_spec_int) + ", request_array_size = " + std::to_string(request_array_size) + "] at socket " + socket_addr + ".");
            }
        }
    }
    catch (const zmq::error_t &e)
    {
        should_shut_down = true;
        printf("[Server] %s, socket %s prepares to close.\n", e.what(), socket_addr.c_str());
        return EMultiverseServerState::ReceiveRequestMetaData;
    }
}

void MultiverseServer::bind_meta_data()
{
    if (!request_meta_data_json.isMember("meta_data") || request_meta_data_json["meta_data"].empty())
    {
        throw std::invalid_argument("[Server] Request meta data at socket " + socket_addr + " doesn't have meta data.");
    }

    Json::Value &meta_data = request_meta_data_json["meta_data"];
    if (!meta_data.isMember("world_name") || meta_data["world_name"].asString().empty())
    {
        throw std::invalid_argument("[Server] Request meta data at socket " + socket_addr + " doesn't have a world name.");
    }
    request_world_name = meta_data["world_name"].asString();

    if (!meta_data.isMember("simulation_name") || meta_data["simulation_name"].asString().empty())
    {
        throw std::invalid_argument("[Server] Request meta data at socket " + socket_addr + " doesn't have a simulation name.");
    }
    request_simulation_name = meta_data["simulation_name"].asString();

    if (simulation_name.empty() && worlds[request_world_name].simulations.count(request_simulation_name) > 0)
    {
        throw std::invalid_argument("[Server] Request meta data at socket " + socket_addr + " requires an existing simulation name (" + request_simulation_name + ").");
    }

    if (!simulation_name.empty() && worlds[request_world_name].simulations.count(request_simulation_name) == 0)
    {
        printf("[Server] Socket %s requests a non-existing simulation (%s).\n", socket_addr.c_str(), request_simulation_name.c_str());
    }

    if (request_simulation_name != simulation_name && !simulation_name.empty() && worlds[request_world_name].simulations.count(request_simulation_name) > 0)
    {
        if (request_meta_data_json.isMember("api_callbacks") && !request_meta_data_json["api_callbacks"].empty())
        {
            throw std::invalid_argument("[Server] Request meta data at socket " + socket_addr + " has API callbacks while requesting a different simulation.");
        }
        printf("[Server] Socket %s (%s) requests a different simulation (%s).\n", socket_addr.c_str(), simulation_name.c_str(), request_simulation_name.c_str());
        Simulation &request_simulation = worlds[request_world_name].simulations[request_simulation_name];

        double start = get_time_now();
        double now = start;
        while (!should_shut_down && request_simulation.meta_data_state != EMetaDataState::Normal)
        {
            now = get_time_now();
            if (now - start > 1)
            {
                printf("[Server] Socket %s is waiting for %s to be in the normal state.\n", socket_addr.c_str(), request_simulation_name.c_str());
                start = now;
            }
        }

        for (const char *const &type_str : {"send", "receive"})
        {
            for (const std::string &object_name : request_meta_data_json[type_str].getMemberNames())
            {
                if (object_name.empty())
                {
                    break;
                }

                Json::Value &attributes = request_simulation.request_meta_data_json[type_str][object_name];

                if (request_meta_data_json[type_str][object_name].empty())
                {
                    attributes = Json::Value(Json::arrayValue);
                    continue;
                }

                for (const Json::Value &attribute : request_meta_data_json[type_str][object_name])
                {
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

    if (request_meta_data_json.isMember("api_callbacks") && !request_meta_data_json["api_callbacks"].empty())
    {
        const Json::Value api_callbacks = request_meta_data_json["api_callbacks"];
        for (const std::string &called_simulation_name : api_callbacks.getMemberNames())
        {
            Simulation &simulation = worlds[world_name].simulations[called_simulation_name];
            simulation.meta_data_state = EMetaDataState::WaitAfterSendReceiveData;
        }
    }

    worlds[world_name].simulations[simulation_name].request_meta_data_json = request_meta_data_json;
    EMetaDataState &meta_data_state = worlds[world_name].simulations[simulation_name].meta_data_state;
    if (request_simulation_name == simulation_name && meta_data_state == EMetaDataState::WaitAfterOtherSendRequestMetaData)
    {
        meta_data_state = EMetaDataState::WaitAfterOtherNormal;
    }
    else if (request_simulation_name != simulation_name || meta_data_state != EMetaDataState::WaitAfterOtherNormal)
    {
        meta_data_state = EMetaDataState::Normal;
    }

    const std::string length_unit = meta_data.isMember("length_unit") ? meta_data["length_unit"].asString() : "m";
    const std::string angle_unit = meta_data.isMember("angle_unit") ? meta_data["angle_unit"].asString() : "rad";
    const std::string handedness = meta_data.isMember("handedness") ? meta_data["handedness"].asString() : "rhs";
    const std::string mass_unit = meta_data.isMember("mass_unit") ? meta_data["mass_unit"].asString() : "kg";
    const std::string time_unit = meta_data.isMember("time_unit") ? meta_data["time_unit"].asString() : "s";

    std::map<EAttribute, std::vector<double>> &conversion_map_double = conversion_map.conversion_map_double;
    for (const std::pair<const std::string, std::pair<EAttribute, std::vector<double>>> &attribute : attribute_map_double)
    {
        conversion_map_double.emplace(attribute.second);
    }

    std::for_each(conversion_map_double[EAttribute::Time].begin(), conversion_map_double[EAttribute::Time].end(),
                  [time_unit](double &time)
                  { time = unit_scale[time_unit]; });

    std::for_each(conversion_map_double[EAttribute::Scalar].begin(), conversion_map_double[EAttribute::Scalar].end(),
                  [](double &scalar)
                  { scalar = 1.0; });

    std::for_each(conversion_map_double[EAttribute::Position].begin(), conversion_map_double[EAttribute::Position].end(),
                  [length_unit](double &position)
                  { position = unit_scale[length_unit]; });

    std::for_each(conversion_map_double[EAttribute::Quaternion].begin(), conversion_map_double[EAttribute::Quaternion].end(),
                  [](double &quaternion)
                  { quaternion = 1.0; });

    std::for_each(conversion_map_double[EAttribute::JointRvalue].begin(), conversion_map_double[EAttribute::JointRvalue].end(),
                  [angle_unit](double &joint_rvalue)
                  { joint_rvalue = unit_scale[angle_unit]; });

    std::for_each(conversion_map_double[EAttribute::JointTvalue].begin(), conversion_map_double[EAttribute::JointTvalue].end(),
                  [length_unit](double &joint_tvalue)
                  { joint_tvalue = unit_scale[length_unit]; });

    std::for_each(conversion_map_double[EAttribute::JointLinearVelocity].begin(), conversion_map_double[EAttribute::JointLinearVelocity].end(),
                  [length_unit, time_unit](double &joint_linear_velocity)
                  { joint_linear_velocity = unit_scale[length_unit] / unit_scale[time_unit]; });

    std::for_each(conversion_map_double[EAttribute::JointAngularVelocity].begin(), conversion_map_double[EAttribute::JointAngularVelocity].end(),
                  [angle_unit, time_unit](double &joint_angular_velocity)
                  { joint_angular_velocity = unit_scale[angle_unit] / unit_scale[time_unit]; });

    std::for_each(conversion_map_double[EAttribute::JointLinearAcceleration].begin(), conversion_map_double[EAttribute::JointLinearAcceleration].end(),
                  [length_unit, time_unit](double &joint_linear_acceleration)
                  { joint_linear_acceleration = unit_scale[length_unit] / (unit_scale[time_unit] * unit_scale[time_unit]); });

    std::for_each(conversion_map_double[EAttribute::JointAngularAcceleration].begin(), conversion_map_double[EAttribute::JointAngularAcceleration].end(),
                  [angle_unit, time_unit](double &joint_angular_acceleration)
                  { joint_angular_acceleration = unit_scale[angle_unit] / (unit_scale[time_unit] * unit_scale[time_unit]); });

    std::for_each(conversion_map_double[EAttribute::JointForce].begin(), conversion_map_double[EAttribute::JointForce].end(),
                  [mass_unit, length_unit, time_unit](double &force)
                  { force = unit_scale[mass_unit] * unit_scale[length_unit] / (unit_scale[time_unit] * unit_scale[time_unit]); });

    std::for_each(conversion_map_double[EAttribute::JointTorque].begin(), conversion_map_double[EAttribute::JointTorque].end(),
                  [mass_unit, length_unit, time_unit](double &torque)
                  { torque = unit_scale[mass_unit] * unit_scale[length_unit] * unit_scale[length_unit] / (unit_scale[time_unit] * unit_scale[time_unit]); });

    std::for_each(conversion_map_double[EAttribute::JointPosition].begin(), conversion_map_double[EAttribute::JointPosition].end(),
                  [length_unit](double &joint_position)
                  { joint_position = unit_scale[length_unit]; });

    std::for_each(conversion_map_double[EAttribute::JointQuaternion].begin(), conversion_map_double[EAttribute::JointQuaternion].end(),
                  [](double &joint_quaternion)
                  { joint_quaternion = 1.0; });

    std::for_each(conversion_map_double[EAttribute::Force].begin(), conversion_map_double[EAttribute::Force].end(),
                  [mass_unit, length_unit, time_unit](double &force)
                  { force = unit_scale[mass_unit] * unit_scale[length_unit] / (unit_scale[time_unit] * unit_scale[time_unit]); });

    std::for_each(conversion_map_double[EAttribute::Torque].begin(), conversion_map_double[EAttribute::Torque].end(),
                  [mass_unit, length_unit, time_unit](double &torque)
                  { torque = unit_scale[mass_unit] * unit_scale[length_unit] * unit_scale[length_unit] / (unit_scale[time_unit] * unit_scale[time_unit]); });

    for (size_t i = 0; i < 3; i++)
    {
        conversion_map_double[EAttribute::RelativeVelocity][i] = unit_scale[length_unit] / unit_scale[time_unit];
    }
    for (size_t i = 3; i < 6; i++)
    {
        conversion_map_double[EAttribute::RelativeVelocity][i] = unit_scale[angle_unit] / unit_scale[time_unit];
    }

    conversion_map_double[EAttribute::CmdJointRvalue] = conversion_map_double[EAttribute::JointRvalue];

    conversion_map_double[EAttribute::CmdJointTvalue] = conversion_map_double[EAttribute::JointTvalue];

    conversion_map_double[EAttribute::CmdJointLinearVelocity] = conversion_map_double[EAttribute::JointLinearVelocity];

    conversion_map_double[EAttribute::CmdJointAngularVelocity] = conversion_map_double[EAttribute::JointAngularVelocity];

    conversion_map_double[EAttribute::CmdJointLinearAcceleration] = conversion_map_double[EAttribute::JointLinearAcceleration];

    conversion_map_double[EAttribute::CmdJointAngularAcceleration] = conversion_map_double[EAttribute::JointAngularAcceleration];

    conversion_map_double[EAttribute::CmdJointForce] = conversion_map_double[EAttribute::Force];

    conversion_map_double[EAttribute::CmdJointTorque] = conversion_map_double[EAttribute::Torque];

    conversion_map_double[EAttribute::OdometricVelocity] = conversion_map_double[EAttribute::RelativeVelocity];

    for (std::pair<const EAttribute, std::vector<double>> &conversion_scale : conversion_map_double)
    {
        std::vector<double>::iterator conversion_scale_it = conversion_scale.second.begin();
        std::vector<double>::iterator handedness_scale_it = handedness_scale[conversion_scale.first][handedness].begin();
        for (size_t i = 0; i < conversion_scale.second.size(); i++)
        {
            *(conversion_scale_it++) *= *(handedness_scale_it++);
        }
    }

    std::map<EAttribute, std::vector<uint8_t>> &conversion_map_uint8_t = conversion_map.conversion_map_uint8_t;
    for (const std::pair<const std::string, std::pair<EAttribute, std::vector<uint8_t>>> &attribute : attribute_map_uint8_t)
    {
        conversion_map_uint8_t.emplace(attribute.second);
    }

    std::map<EAttribute, std::vector<uint16_t>> &conversion_map_uint16_t = conversion_map.conversion_map_uint16_t;
    for (const std::pair<const std::string, std::pair<EAttribute, std::vector<uint16_t>>> &attribute : attribute_map_uint16_t)
    {
        conversion_map_uint16_t.emplace(attribute.second);
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

    for (const std::string &object_name : send_objects_json.getMemberNames())
    {
        response_meta_data_json["send"][object_name] = Json::objectValue;
        Object &object = objects[object_name];
        simulation.objects[object_name] = &object;
        for (const Json::Value &attribute_json : send_objects_json[object_name])
        {
            const std::string &attribute_name = attribute_json.asString();
            Attribute &attribute = object.attributes[attribute_name];
            if (cumulative_attribute_names.count(attribute_name) == 0)
            {
                if (attribute.attribute_double.data.size() == 0)
                {
                    attribute.attribute_double.data = attribute_map_double[attribute_name].second;
                    for (size_t i = 0; i < attribute.attribute_double.data.size(); i++)
                    {
                        double *data = &attribute.attribute_double.data[i];
                        const double conversion = conversion_map.conversion_map_double[attribute_map_double[attribute_name].first][i];
                        send_buffer.buffer_double.data_vec.emplace_back(data, conversion);
                        response_meta_data_json["send"][object_name][attribute_name].append(*data * conversion);
                    }
                }
                else
                {
                    // printf("[Server] Continue state [%s - %s] on socket %s\n", object_name.c_str(), attribute_name.c_str(), socket_addr.c_str());
                    continue_state = true;
                    attribute.attribute_double.is_sent = true;

                    for (size_t i = 0; i < attribute.attribute_double.data.size(); i++)
                    {
                        double *data = &attribute.attribute_double.data[i];
                        const double conversion = conversion_map.conversion_map_double[attribute_map_double[attribute_name].first][i];
                        send_buffer.buffer_double.data_vec.emplace_back(data, conversion);
                        response_meta_data_json["send"][object_name][attribute_name].append(*data * conversion);
                    }
                }
                if (attribute.attribute_uint8_t.data.size() == 0)
                {
                    attribute.attribute_uint8_t.data = attribute_map_uint8_t[attribute_name].second;
                    for (size_t i = 0; i < attribute.attribute_uint8_t.data.size(); i++)
                    {
                        uint8_t *data = &attribute.attribute_uint8_t.data[i];
                        const uint8_t conversion = conversion_map.conversion_map_uint8_t[attribute_map_uint8_t[attribute_name].first][i];
                        send_buffer.buffer_uint8_t.data_vec.emplace_back(data, conversion);
                        response_meta_data_json["send"][object_name][attribute_name].append(*data >> conversion);
                    }
                }
                else
                {
                    // printf("[Server] Continue state [%s - %s] on socket %s\n", object_name.c_str(), attribute_name.c_str(), socket_addr.c_str());
                    continue_state = true;
                    attribute.attribute_uint8_t.is_sent = true;

                    for (size_t i = 0; i < attribute.attribute_uint8_t.data.size(); i++)
                    {
                        uint8_t *data = &attribute.attribute_uint8_t.data[i];
                        const uint8_t conversion = conversion_map.conversion_map_uint8_t[attribute_map_uint8_t[attribute_name].first][i];
                        send_buffer.buffer_uint8_t.data_vec.emplace_back(data, conversion);
                        response_meta_data_json["send"][object_name][attribute_name].append(*data >> conversion);
                    }
                }
                if (attribute.attribute_uint16_t.data.size() == 0)
                {
                    attribute.attribute_uint16_t.data = attribute_map_uint16_t[attribute_name].second;
                    for (size_t i = 0; i < attribute.attribute_uint16_t.data.size(); i++)
                    {
                        uint16_t *data = &attribute.attribute_uint16_t.data[i];
                        const uint16_t conversion = conversion_map.conversion_map_uint16_t[attribute_map_uint16_t[attribute_name].first][i];
                        send_buffer.buffer_uint16_t.data_vec.emplace_back(data, conversion);
                        response_meta_data_json["send"][object_name][attribute_name].append(*data >> conversion);
                    }
                }
                else
                {
                    // printf("[Server] Continue state [%s - %s] on socket %s\n", object_name.c_str(), attribute_name.c_str(), socket_addr.c_str());
                    continue_state = true;
                    attribute.attribute_uint16_t.is_sent = true;

                    for (size_t i = 0; i < attribute.attribute_uint16_t.data.size(); i++)
                    {
                        uint16_t *data = &attribute.attribute_uint16_t.data[i];
                        const uint16_t conversion = conversion_map.conversion_map_uint16_t[attribute_map_uint16_t[attribute_name].first][i];
                        send_buffer.buffer_uint16_t.data_vec.emplace_back(data, conversion);
                        response_meta_data_json["send"][object_name][attribute_name].append(*data >> conversion);
                    }
                }
            }
            else
            {
                std::vector<double> &simulation_data_double = attribute.attribute_double.simulation_data[simulation_name];
                if (simulation_data_double.size() == 0)
                {
                    simulation_data_double = attribute_map_double[attribute_name].second;
                }
                for (size_t i = 0; i < simulation_data_double.size(); i++)
                {
                    double *data = &simulation_data_double[i];
                    const double conversion = conversion_map.conversion_map_double[attribute_map_double[attribute_name].first][i];
                    send_buffer.buffer_double.data_vec.emplace_back(data, conversion);
                    response_meta_data_json["send"][object_name][attribute_name].append(*data * conversion);
                }

                std::vector<uint8_t> &simulation_data_uint8_t = attribute.attribute_uint8_t.simulation_data[simulation_name];
                if (simulation_data_uint8_t.size() == 0)
                {
                    simulation_data_uint8_t = attribute_map_uint8_t[attribute_name].second;
                }
                for (size_t i = 0; i < simulation_data_uint8_t.size(); i++)
                {
                    uint8_t *data = &simulation_data_uint8_t[i];
                    const uint8_t conversion = conversion_map.conversion_map_uint8_t[attribute_map_uint8_t[attribute_name].first][i];
                    send_buffer.buffer_uint8_t.data_vec.emplace_back(data, conversion);
                    response_meta_data_json["send"][object_name][attribute_name].append(*data >> conversion);
                }

                std::vector<uint16_t> &simulation_data_uint16_t = attribute.attribute_uint16_t.simulation_data[simulation_name];
                if (simulation_data_uint16_t.size() == 0)
                {
                    simulation_data_uint16_t = attribute_map_uint16_t[attribute_name].second;
                }
                for (size_t i = 0; i < simulation_data_uint16_t.size(); i++)
                {
                    uint16_t *data = &simulation_data_uint16_t[i];
                    const uint16_t conversion = conversion_map.conversion_map_uint16_t[attribute_map_uint16_t[attribute_name].first][i];
                    send_buffer.buffer_uint16_t.data_vec.emplace_back(data, conversion);
                    response_meta_data_json["send"][object_name][attribute_name].append(*data >> conversion);
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
        receive_objects_json = Json::objectValue;
        for (const std::pair<const std::string, Object> &object : worlds[world_name].objects)
        {
            for (const std::pair<const std::string, Attribute> &attribute_pair : object.second.attributes)
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

                receive_objects_json[object_name] = Json::arrayValue;
                for (const std::pair<const std::string, Attribute> &attribute_pair : worlds[world_name].objects[object_name].attributes)
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
                for (const std::pair<const std::string, Object> &object_pair : worlds[world_name].objects)
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
    std::map<std::string, Object> &objects = worlds[world_name].objects;
    Simulation &simulation = worlds[world_name].simulations[simulation_name];

    for (const std::string &object_name : receive_objects_json.getMemberNames())
    {
        Object &object = objects[object_name];
        simulation.objects[object_name] = &object;
        response_meta_data_json["receive"][object_name] = Json::objectValue;
        for (const Json::Value &attribute_json : receive_objects_json[object_name])
        {
            const std::string attribute_name = attribute_json.asString();
            Attribute &attribute = worlds[world_name].objects[object_name].attributes[attribute_name];
            if (cumulative_attribute_names.count(attribute_name) > 0)
            {
                if (attribute.attribute_double.data.size() == 0)
                {
                    attribute.attribute_double.data = attribute_map_double[attribute_name].second;
                    attribute.attribute_double.is_sent = true;
                }
                if (attribute.attribute_uint8_t.data.size() == 0)
                {
                    attribute.attribute_uint8_t.data = attribute_map_uint8_t[attribute_name].second;
                    attribute.attribute_uint8_t.is_sent = true;
                }
                if (attribute.attribute_uint16_t.data.size() == 0)
                {
                    attribute.attribute_uint16_t.data = attribute_map_uint16_t[attribute_name].second;
                    attribute.attribute_uint16_t.is_sent = true;
                }
            }

            for (size_t i = 0; i < attribute.attribute_double.data.size(); i++)
            {
                double *data = &attribute.attribute_double.data[i];
                const double conversion = 1.0 / conversion_map.conversion_map_double[attribute_map_double[attribute_name].first][i];
                receive_buffer.buffer_double.data_vec.emplace_back(data, conversion);
                response_meta_data_json["receive"][object_name][attribute_name].append(*data * conversion);
            }
            for (size_t i = 0; i < attribute.attribute_uint8_t.data.size(); i++)
            {
                uint8_t *data = &attribute.attribute_uint8_t.data[i];
                const uint8_t conversion = conversion_map.conversion_map_uint8_t[attribute_map_uint8_t[attribute_name].first][i];
                receive_buffer.buffer_uint8_t.data_vec.emplace_back(data, conversion);
                response_meta_data_json["receive"][object_name][attribute_name].append(*data >> conversion);
            }
            for (size_t i = 0; i < attribute.attribute_uint16_t.data.size(); i++)
            {
                uint16_t *data = &attribute.attribute_uint16_t.data[i];
                const uint16_t conversion = conversion_map.conversion_map_uint16_t[attribute_map_uint16_t[attribute_name].first][i];
                receive_buffer.buffer_uint16_t.data_vec.emplace_back(data, conversion);
                response_meta_data_json["receive"][object_name][attribute_name].append(*data >> conversion);
            }
        }
    }
}

void MultiverseServer::wait_for_api_callbacks_response()
{
    const Json::Value api_callbacks = request_meta_data_json["api_callbacks"];

    for (const std::string &called_simulation_name : api_callbacks.getMemberNames())
    {
        Simulation &simulation = worlds[world_name].simulations[called_simulation_name];
        simulation.request_meta_data_json["api_callbacks"] = api_callbacks[called_simulation_name];
        for (const Json::Value &api_callback : api_callbacks[called_simulation_name])
        {
            for (const std::string &callback_key : api_callback.getMemberNames())
            {
                std::map<std::string, std::vector<std::string>> api_callback_map;
                api_callback_map[callback_key] = std::vector<std::string>{};
                for (const Json::Value &param : api_callback[callback_key])
                {
                    api_callback_map[callback_key].push_back(param.asString());
                }
                simulation.api_callbacks.push_back(api_callback_map);
            }
        }
        simulation.meta_data_state = EMetaDataState::WaitAfterSendReceiveData;
    }

    double start = get_time_now();
    double now = get_time_now();
    bool stop = true;
    while (!should_shut_down)
    {
        now = get_time_now();
        stop = true;
        for (const std::string &called_simulation_name : api_callbacks.getMemberNames())
        {
            Simulation &simulation = worlds[world_name].simulations[called_simulation_name];
            if (simulation.meta_data_state != EMetaDataState::Normal)
            {
                stop = false;
                if (now - start > 1)
                {
                    if (simulation.api_callbacks.size() != 0)
                    {
                        printf("[Server] Socket %s is waiting for %s to send API callbacks response data.\n", socket_addr.c_str(), called_simulation_name.c_str());
                    }
                    else
                    {
                        printf("[Server] Socket %s is waiting for %s to send data.\n", socket_addr.c_str(), called_simulation_name.c_str());
                    }
                }
            }
        }
        if (now - start > 1)
        {
            start = now;
        }
        if (stop)
        {
            break;
        }
    }

    for (const std::string &called_simulation_name : api_callbacks.getMemberNames())
    {
        Simulation &simulation = worlds[world_name].simulations[called_simulation_name];
        response_meta_data_json["api_callbacks_response"][called_simulation_name] = Json::arrayValue;
        for (const std::map<std::string, std::vector<std::string>> &api_callbacks_response : simulation.api_callbacks_response)
        {
            for (const std::pair<const std::string, std::vector<std::string>> &api_callback_response : api_callbacks_response)
            {
                Json::Value api_callback_response_json;
                api_callback_response_json[api_callback_response.first] = Json::arrayValue;
                for (const std::string &param : api_callback_response.second)
                {
                    api_callback_response_json[api_callback_response.first].append(param);
                }
                response_meta_data_json["api_callbacks_response"][called_simulation_name].append(api_callback_response_json);
            }
        }
    }
}

void MultiverseServer::send_response_meta_data()
{
    if (continue_state)
    {
        continue_state = false;
    }

    if (should_shut_down)
    {
        const int message_int = 0;
        zmq::message_t response_message_int(sizeof(message_int));
        memcpy(response_message_int.data(), &message_int, sizeof(message_int));
        socket.send(response_message_int, zmq::send_flags::none);
    }
    else
    {
        const int message_int = 1;
        zmq::message_t response_message_int(sizeof(message_int));
        memcpy(response_message_int.data(), &message_int, sizeof(message_int));
        socket.send(response_message_int, zmq::send_flags::sndmore);

        const std::string message_str = response_meta_data_json.toStyledString();
        zmq::message_t response_message_str(message_str.size());
        memcpy(response_message_str.data(), message_str.c_str(), message_str.size());
        socket.send(response_message_str, zmq::send_flags::none);
    }
}

void MultiverseServer::init_send_and_receive_data()
{
    send_buffer.buffer_double.size = send_buffer.buffer_double.data_vec.size();
    send_buffer.buffer_double.data = (double *)calloc(send_buffer.buffer_double.size, sizeof(double));
    send_buffer.buffer_uint8_t.size = send_buffer.buffer_uint8_t.data_vec.size();
    send_buffer.buffer_uint8_t.data = (uint8_t *)calloc(send_buffer.buffer_uint8_t.size, sizeof(uint8_t));
    send_buffer.buffer_uint16_t.size = send_buffer.buffer_uint16_t.data_vec.size();
    send_buffer.buffer_uint16_t.data = (uint16_t *)calloc(send_buffer.buffer_uint16_t.size, sizeof(uint16_t));
    receive_buffer.buffer_double.size = receive_buffer.buffer_double.data_vec.size();
    receive_buffer.buffer_double.data = (double *)calloc(receive_buffer.buffer_double.size, sizeof(double));
    receive_buffer.buffer_uint8_t.size = receive_buffer.buffer_uint8_t.data_vec.size();
    receive_buffer.buffer_uint8_t.data = (uint8_t *)calloc(receive_buffer.buffer_uint8_t.size, sizeof(uint8_t));
    receive_buffer.buffer_uint16_t.size = receive_buffer.buffer_uint16_t.data_vec.size();
    receive_buffer.buffer_uint16_t.data = (uint16_t *)calloc(receive_buffer.buffer_uint16_t.size, sizeof(uint16_t));
}

void MultiverseServer::wait_for_other_send_data()
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

void MultiverseServer::bind_send_data()
{
    for (size_t i = 0; i < send_buffer.buffer_double.size; i++)
    {
        *send_buffer.buffer_double.data_vec[i].first = send_buffer.buffer_double.data[i] * send_buffer.buffer_double.data_vec[i].second;
    }
    for (size_t i = 0; i < send_buffer.buffer_uint8_t.size; i++)
    {
        *send_buffer.buffer_uint8_t.data_vec[i].first = send_buffer.buffer_uint8_t.data[i] >> send_buffer.buffer_uint8_t.data_vec[i].second;
    }
    for (size_t i = 0; i < send_buffer.buffer_uint16_t.size; i++)
    {
        *send_buffer.buffer_uint16_t.data_vec[i].first = send_buffer.buffer_uint16_t.data[i] >> send_buffer.buffer_uint16_t.data_vec[i].second;
    }
}

void MultiverseServer::wait_for_receive_data()
{
    for (const std::string &object_name : send_objects_json.getMemberNames())
    {
        for (const Json::Value &attribute_json : send_objects_json[object_name])
        {
            const std::string attribute_name = attribute_json.asString();
            worlds[world_name].objects[object_name].attributes[attribute_name].attribute_double.is_sent = true;
            worlds[world_name].objects[object_name].attributes[attribute_name].attribute_uint8_t.is_sent = true;
            worlds[world_name].objects[object_name].attributes[attribute_name].attribute_uint16_t.is_sent = true;
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
                while ((worlds[world_name].objects.count(object_name) == 0 ||
                        worlds[world_name].objects[object_name].attributes.count(attribute_name) == 0 ||
                        !worlds[world_name].objects[object_name].attributes[attribute_name].attribute_double.is_sent ||
                        !worlds[world_name].objects[object_name].attributes[attribute_name].attribute_uint8_t.is_sent ||
                        !worlds[world_name].objects[object_name].attributes[attribute_name].attribute_uint16_t.is_sent) &&
                       !should_shut_down)
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

            std::vector<double> &double_data = worlds[world_name].objects[object_name].attributes[attribute_name].attribute_double.data;
            double_data = attribute_map_double[attribute_name].second;
            for (size_t i = 0; i < double_data.size(); i++)
            {
                for (std::pair<const std::string, Simulation> &simulation_pair : worlds[world_name].simulations)
                {
                    if (simulation_pair.second.objects.count(object_name) == 0)
                    {
                        continue;
                    }

                    const std::string &simulation_name = simulation_pair.first;
                    const std::vector<double> &simulation_data = (*simulation_pair.second.objects[object_name]).attributes[attribute_name].attribute_double.simulation_data[simulation_name];
                    if (simulation_data.size() != double_data.size())
                    {
                        continue;
                    }

                    double_data[i] += simulation_data[i];
                }
            }

            std::vector<uint8_t> &uint8_t_data = worlds[world_name].objects[object_name].attributes[attribute_name].attribute_uint8_t.data;
            uint8_t_data = attribute_map_uint8_t[attribute_name].second;
            for (size_t i = 0; i < uint8_t_data.size(); i++)
            {
                for (std::pair<const std::string, Simulation> &simulation_pair : worlds[world_name].simulations)
                {
                    if (simulation_pair.second.objects.count(object_name) == 0)
                    {
                        continue;
                    }

                    const std::string &simulation_name = simulation_pair.first;
                    const std::vector<uint8_t> &simulation_data = (*simulation_pair.second.objects[object_name]).attributes[attribute_name].attribute_uint8_t.simulation_data[simulation_name];
                    if (simulation_data.size() != uint8_t_data.size())
                    {
                        continue;
                    }

                    uint8_t_data[i] += simulation_data[i];
                }
            }

            std::vector<uint16_t> &uint16_t_data = worlds[world_name].objects[object_name].attributes[attribute_name].attribute_uint16_t.data;
            uint16_t_data = attribute_map_uint16_t[attribute_name].second;
            for (size_t i = 0; i < uint16_t_data.size(); i++)
            {
                for (std::pair<const std::string, Simulation> &simulation_pair : worlds[world_name].simulations)
                {
                    if (simulation_pair.second.objects.count(object_name) == 0)
                    {
                        continue;
                    }

                    const std::string &simulation_name = simulation_pair.first;
                    const std::vector<uint16_t> &simulation_data = (*simulation_pair.second.objects[object_name]).attributes[attribute_name].attribute_uint16_t.simulation_data[simulation_name];
                    if (simulation_data.size() != uint16_t_data.size())
                    {
                        continue;
                    }

                    uint16_t_data[i] += simulation_data[i];
                }
            }
        }
    }
}

void MultiverseServer::bind_receive_data()
{
    for (size_t i = 0; i < receive_buffer.buffer_double.size; i++)
    {
        receive_buffer.buffer_double.data[i] = *receive_buffer.buffer_double.data_vec[i].first * receive_buffer.buffer_double.data_vec[i].second;
    }
    for (size_t i = 0; i < receive_buffer.buffer_uint8_t.size; i++)
    {
        receive_buffer.buffer_uint8_t.data[i] = *receive_buffer.buffer_uint8_t.data_vec[i].first >> receive_buffer.buffer_uint8_t.data_vec[i].second;
    }
    for (size_t i = 0; i < receive_buffer.buffer_uint16_t.size; i++)
    {
        receive_buffer.buffer_uint16_t.data[i] = *receive_buffer.buffer_uint16_t.data_vec[i].first >> receive_buffer.buffer_uint16_t.data_vec[i].second;
    }
}

void MultiverseServer::receive_new_request_meta_data()
{
    printf("[Server] Socket %s has received new request meta data.\n", socket_addr.c_str());

    Simulation &simulation = worlds[world_name].simulations[simulation_name];
    simulation.meta_data_state = EMetaDataState::WaitAfterOtherBindSendData;
    double start = get_time_now();
    double now = get_time_now();
    while (!should_shut_down)
    {
        if (simulation.meta_data_state == EMetaDataState::WaitAfterOtherSendRequestMetaData)
        {
            break;
        }

        if (simulation.api_callbacks.size() != 0)
        {
            response_meta_data_json["api_callbacks"] = simulation.request_meta_data_json["api_callbacks"];
            send_response_meta_data();

            receive_data();
            init_send_and_receive_data();

            response_meta_data_json.removeMember("api_callbacks");
            simulation.api_callbacks.clear();
            simulation.request_meta_data_json.removeMember("api_callbacks");
            simulation.api_callbacks_response.clear();
            simulation.request_meta_data_json["send"] = request_meta_data_json["send"];
            simulation.request_meta_data_json["receive"] = request_meta_data_json["receive"];

            const Json::Value api_callbacks_response = request_meta_data_json["api_callbacks_response"];
            for (const Json::Value &api_callback_response : api_callbacks_response)
            {
                for (const std::string &callback_key : api_callback_response.getMemberNames())
                {
                    std::map<std::string, std::vector<std::string>> api_callback_map;
                    api_callback_map[callback_key] = std::vector<std::string>{};
                    for (const Json::Value &param : api_callback_response[callback_key])
                    {
                        api_callback_map[callback_key].push_back(param.asString());
                    }
                    simulation.api_callbacks_response.push_back(api_callback_map);
                }
            }

            break;
        }

        now = get_time_now();
        if (now - start > 1)
        {
            printf("[Server] Socket %s is waiting for send data to be sent.\n", socket_addr.c_str());
            start = now;
        }
    }

    request_meta_data_json = simulation.request_meta_data_json;

    send_buffer.buffer_double.data_vec.clear();
    send_buffer.buffer_uint8_t.data_vec.clear();
    send_buffer.buffer_uint16_t.data_vec.clear();
    receive_buffer.buffer_double.data_vec.clear();
    receive_buffer.buffer_uint8_t.data_vec.clear();
    receive_buffer.buffer_uint16_t.data_vec.clear();
}

void MultiverseServer::send_receive_data()
{
    if (should_shut_down)
    {
        const int message_spec_int = 0;
        zmq::message_t message_spec(sizeof(int));
        memcpy(message_spec.data(), &message_spec_int, sizeof(int));
        socket.send(message_spec, zmq::send_flags::none);
    }
    else
    {
        const int message_spec_int = 2 + (receive_buffer.buffer_double.size > 0) + (receive_buffer.buffer_uint8_t.size > 0) + (receive_buffer.buffer_uint16_t.size > 0);
        zmq::message_t message_spec(sizeof(int));
        memcpy(message_spec.data(), &message_spec_int, sizeof(int));
        socket.send(message_spec, zmq::send_flags::sndmore);
    }

    zmq::message_t message_time(sizeof(double));
    if (worlds[world_name].simulations[simulation_name].meta_data_state == EMetaDataState::Reset)
    {
        worlds[world_name].simulations[simulation_name].meta_data_state = EMetaDataState::Normal;
        const double world_time = 0.0;
        memcpy(message_time.data(), &world_time, sizeof(double));
    }
    else
    {
        memcpy(message_time.data(), &worlds[world_name].time, sizeof(double));
    }

    if (receive_buffer.buffer_double.size > 0 || receive_buffer.buffer_uint8_t.size > 0 || receive_buffer.buffer_uint16_t.size > 0)
    {
        socket.send(message_time, zmq::send_flags::sndmore);
        if (receive_buffer.buffer_double.size > 0)
        {
            zmq::message_t message_double(receive_buffer.buffer_double.size * sizeof(double));
            memcpy(message_double.data(), receive_buffer.buffer_double.data, receive_buffer.buffer_double.size * sizeof(double));
            if (receive_buffer.buffer_uint8_t.size > 0 || receive_buffer.buffer_uint16_t.size > 0)
            {
                socket.send(message_double, zmq::send_flags::sndmore);
            }
            else
            {
                socket.send(message_double, zmq::send_flags::none);
            }
        }

        if (receive_buffer.buffer_uint8_t.size > 0)
        {
            zmq::message_t message_uint8_t(receive_buffer.buffer_uint8_t.size * sizeof(uint8_t));
            memcpy(message_uint8_t.data(), receive_buffer.buffer_uint8_t.data, receive_buffer.buffer_uint8_t.size * sizeof(uint8_t));
            if (receive_buffer.buffer_uint16_t.size > 0)
            {
                socket.send(message_uint8_t, zmq::send_flags::sndmore);
            }
            else
            {
                socket.send(message_uint8_t, zmq::send_flags::none);
            }
        }

        if (receive_buffer.buffer_uint16_t.size > 0)
        {
            zmq::message_t message_uint16_t(receive_buffer.buffer_uint16_t.size * sizeof(uint16_t));
            memcpy(message_uint16_t.data(), receive_buffer.buffer_uint16_t.data, receive_buffer.buffer_uint16_t.size * sizeof(uint16_t));
            socket.send(message_uint16_t, zmq::send_flags::none);
        }
    }
    else
    {
        socket.send(message_time, zmq::send_flags::none);
    }
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
            assert(recv_result_t.has_value());
            receive_addr = request.to_string();
            printf("[Server] Received request to open socket %s.\n", receive_addr.c_str());
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
        printf("[Server] Sending response to open socket %s.\n", receive_addr.c_str());
        server_socket.send(response, zmq::send_flags::none);
        printf("[Server] Sent response to open socket %s.\n", receive_addr.c_str());
    }

    for (std::pair<const std::string, std::thread> &worker : workers)
    {
        worker.second.join();
    }
}