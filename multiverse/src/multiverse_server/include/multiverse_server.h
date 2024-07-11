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
    Torque,
    RGB_3840_2160,
    RGB_1280_1024,
    RGB_640_480,
    RGB_128_128
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

template<class T>
struct TypedBuffer
{
    T *data;
    size_t size = 0;
    std::vector<std::pair<T *, T>> data_vec;
};

struct Buffer
{
    TypedBuffer<double> buffer_double;
    TypedBuffer<uint8_t> buffer_uint8_t;
};

struct ConversionMap
{
    std::map<EAttribute, std::vector<double>> conversion_map_double;
    std::map<EAttribute, std::vector<uint8_t>> conversion_map_uint8_t;
};

class MultiverseServer
{
public:
    MultiverseServer(const std::string &in_socket_addr);

    ~MultiverseServer();

public:
    void start();

private:
    void receive_data();

    void bind_send_objects();

    void bind_meta_data();

    void validate_meta_data();

    void wait_for_objects();

    void bind_receive_objects();

    void send_response_meta_data();

    void init_send_and_receive_data();

    void wait_for_receive_data();

    void compute_cumulative_data();

    void send_receive_data();

private:
    EMultiverseServerState flag = EMultiverseServerState::ReceiveRequestMetaData;

    std::string socket_addr;

    zmq::socket_t socket;

    Json::Value request_meta_data_json;

    Json::Value send_objects_json;

    Json::Value response_meta_data_json;

    Json::Value receive_objects_json;

    Buffer send_buffer;

    Buffer receive_buffer;

    ConversionMap conversion_map;

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