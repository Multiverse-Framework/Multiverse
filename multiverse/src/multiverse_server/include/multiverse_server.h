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

#include <zmq.hpp>
#include <cmath>
#include <map>
#include <vector>
#ifdef __linux__
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/reader.h>
#elif _WIN32
#include <json/json.h>
#include <json/reader.h>
#include <boost/filesystem.hpp>
#endif

enum class EAttribute : unsigned char
{
    Time,
    Position,
    Quaternion,
    RelativeVelocity,
    OdometricVelocity,
    JointRvalue,
    JointTvalue,
    JointLinearVelocity,
    JointAngularVelocity,
    JointForce,
    JointTorque,
    CmdJointRvalue,
    CmdJointTvalue,
    CmdJointLinearVelocity,
    CmdJointAngularVelocity,
    CmdJointForce,
    CmdJointTorque,
    JointPosition,
    JointQuaternion,
    Force,
    Torque
};

enum class EMultiverseServerState : unsigned char
{
    ReceiveRequestMetaData,
    BindObjects,
    SendResponseMetaData,
    ReceiveSendData,
    BindSendData,
    BindReceiveData,
    SendReceiveData,
};

class MultiverseServer
{
public:
    MultiverseServer(const std::string &in_socket_addr);

    ~MultiverseServer();

public:
    void start();

private:
    void receive_request_meta_data();

    void bind_send_objects();

    void bind_meta_data();

    void validate_meta_data();

    void wait_for_objects();

    void bind_receive_objects();

    void send_response_meta_data();

    void receive_send_data();

    void wait_for_receive_data();

    void compute_cumulative_data();

    void send_receive_data();

private:
    EMultiverseServerState flag = EMultiverseServerState::ReceiveRequestMetaData;

    zmq::message_t message;

    std::string socket_addr;

    zmq::socket_t socket;

    Json::Value request_meta_data_json;

    Json::Value send_objects_json;

    Json::Value response_meta_data_json;

    Json::Value receive_objects_json;

    size_t send_buffer_size = 1;

    size_t receive_buffer_size = 1;

    double *send_buffer;

    double *receive_buffer;

    std::vector<std::pair<double *, double>> send_data_vec;

    std::vector<std::pair<double *, double>> receive_data_vec;

    std::map<EAttribute, std::vector<double>> conversion_map;

    std::string world_name;

    std::string request_world_name;

    std::string simulation_name;

    std::string request_simulation_name;

    Json::Reader reader;

    bool is_receive_data_sent;

    bool continue_state = false;
};

void start_multiverse_server(const std::string &server_socket_addr);

extern bool should_shut_down;

extern std::map<std::string, bool> sockets_need_clean_up;

extern zmq::context_t server_context;