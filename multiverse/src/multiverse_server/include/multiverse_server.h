// Copyright (c) 2023, Giang Hoang Nguyen - Institute for Artificial
// Intelligence, University Bremen

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

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

/**
 * @brief Attributes that can be sent and received between the server and the
 * client.
 *
 */
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
    JointLinearAcceleration,
    JointAngularAcceleration,
    JointForce,
    JointTorque,
    CmdJointRvalue,
    CmdJointTvalue,
    CmdJointLinearVelocity,
    CmdJointAngularVelocity,
    CmdJointLinearAcceleration,
    CmdJointAngularAcceleration,
    CmdJointForce,
    CmdJointTorque,
    JointPosition,
    JointQuaternion,
    Force,
    Torque,
    RGB_3840_2160,
    RGB_1280_1024,
    RGB_640_480,
    RGB_128_128,
    Depth_3840_2160,
    Depth_1280_1024,
    Depth_640_480,
    Depth_128_128,
};

/**
 * @brief The state of the server.
 *
 */
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

/**
 * @brief TypedBuffer is a buffer that contains data of a specific type.
 *
 * @tparam T The type of the data.
 */
template <class T>
struct TypedBuffer
{
    T *data;
    size_t size = 0;
    std::vector<std::pair<T *, T>> data_vec;
};

/**
 * @brief Buffer contains buffers for double and uint8_t data.
 *
 */
struct Buffer
{
    TypedBuffer<double> buffer_double;
    TypedBuffer<uint8_t> buffer_uint8_t;
    TypedBuffer<uint16_t> buffer_uint16_t;
};

/**
 * @brief ConversionMap contains the conversion map for double and uint8_t data.
 *
 */
struct ConversionMap
{
    std::map<EAttribute, std::vector<double>> conversion_map_double;
    std::map<EAttribute, std::vector<uint8_t>> conversion_map_uint8_t;
    std::map<EAttribute, std::vector<uint16_t>> conversion_map_uint16_t;
};

/**
 * @brief MultiverseServer is the server that communicates with the clients.
 *
 */
class MultiverseServer
{
public:
    MultiverseServer(const std::string &in_socket_addr);

    ~MultiverseServer();

public:
    /**
     * @brief Start the server, this function will run indefinitely until the
     * server is shut down.
     *
     */
    void start();

private:
    /**
     * @brief Receive the request meta data or the data from the client,
     * depending on the data received. This function will return the next state
     * of the server, depending on the state of the server and the data
     * received.
     *
     * @return EMultiverseServerState The next state of the server.
     */
    EMultiverseServerState receive_data();

    /**
     * @brief Bind the objects that are received from the client to
     * send_objects_json, send_buffer and receive_objects_json, receive_buffer.
     *
     */
    void bind_send_objects();

    /**
     * @brief Bind the meta data to the state of the server, including
     * world_name, simulation_name, request_world_name, request_simulation_name,
     * request_meta_data_json, response_meta_data_json.
     *
     */
    void bind_meta_data();
    
    /**
     * @brief Validate the meta data, check if there are empty fields in the
     * send fields and receive fields from request_meta_data_json.
     *
     */
    void validate_meta_data();

    /**
     * @brief Wait for the objects to be declared from the client.
     *
     */
    void wait_for_objects();

    /**
     * @brief Bind the objects that are declared from the client to response_meta_data_json.
     *
     */
    void bind_receive_objects();

    /**
     * @brief Wait for the API callbacks response from other clients, which
     * receive the API callbacks from this client.
     *
     */
    void wait_for_api_callbacks_response();

    /**
     * @brief Send the response meta data to the client.
     *
     */
    void send_response_meta_data();

    /**
     * @brief Initialize the send_buffer and receive_buffer according to the
     * size from the response_meta_data_json.
     *
     */
    void init_send_and_receive_data();

    /**
     * @brief Wait for the other clients to send the data.
     *
     */
    void wait_for_other_send_data();

    /**
     * @brief Bind the send data, which is received from the client to the send_buffer.
     *
     */
    void bind_send_data();

    /**
     * @brief Wait for the data to be received from the client (constantly check
     * if the data is nan).
     *
     */
    void wait_for_receive_data();

    /**
     * @brief Compute the cumulative data, such as force and torque.
     *
     */
    void compute_cumulative_data();

    /**
     * @brief Bind the receive data, which will be sent to the client to the receive_buffer.
     *
     */
    void bind_receive_data();

    /**
     * @brief If request_simulation_name != simulation_name, then the server will receive the new request meta data,
     * this process will wait until the other client sends the new request meta data and the data after that.
     *
     */
    void receive_new_request_meta_data();

    /**
     * @brief Send the receive data to the client.
     *
     */
    void send_receive_data();

private:
    /**
     * @brief Flag to indicate the state of the server.
     * 
     */
    EMultiverseServerState flag = EMultiverseServerState::ReceiveRequestMetaData;

    /**
     * @brief The socket address of the client.
     * 
     */
    std::string socket_addr;

    /**
     * @brief The socket of the client.
     * 
     */
    zmq::socket_t socket;

    /**
     * @brief The request meta data from the client.
     * 
     */
    Json::Value request_meta_data_json;

    /**
     * @brief The send objects from the client, request_meta_data_json["send"] without empty fields.
     * 
     */
    Json::Value send_objects_json;

    /**
     * @brief The response meta data from the client.
     * 
     */
    Json::Value response_meta_data_json;

    /**
     * @brief The receive objects from the client, request_meta_data_json["receive"] without empty fields.
     * 
     */
    Json::Value receive_objects_json;

    /**
     * @brief The send buffer.
     * 
     */
    Buffer send_buffer;

    /**
     * @brief The receive buffer.
     * 
     */
    Buffer receive_buffer;

    /**
     * @brief The conversion map for the data.
     * 
     */
    ConversionMap conversion_map;

    /**
     * @brief The name of the world.
     * 
     */
    std::string world_name;

    /**
     * @brief The name of the requested world from another client, if the
     * request_world_name != world_name, then the server will receive the new
     * request meta data that contains the new world name.
     *
     */
    std::string request_world_name;

    /**
     * @brief The name of the simulation.
     * 
     */
    std::string simulation_name;

    /**
     * @brief The name of the requested simulation from another client, if the
     * request_simulation_name != simulation_name, then the server will receive
     * the new request meta data that contains the new simulation name.
     *
     */
    std::string request_simulation_name;

    /**
     * @brief The JSON reader.
     * 
     */
    Json::Reader reader;

    /**
     * @brief If the data is nan, then the server will wait for the data to be
     * received, this flag will be set to false.
     * 
     */
    bool is_receive_data_sent;

    /**
     * @brief If the data is non-nan before sending the response meta data, then
     * the server will send the response meta data with the values of the data.
     *
     */
    bool continue_state = false;
};

/**
 * @brief Start the multiverse server with the server socket address.
 * 
 * @param server_socket_addr The server socket address.
 */
void start_multiverse_server(const std::string &server_socket_addr);

/**
 * @brief The flag to indicate if the server should shut down.
 * 
 */
extern bool should_shut_down;

/**
 * @brief The map that contains the sockets that need to be cleaned up.
 * 
 */
extern std::map<std::string, bool> sockets_need_clean_up;

/**
 * @brief The context of the server.
 * 
 */
extern zmq::context_t server_context;