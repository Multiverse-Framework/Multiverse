// Copyright (c) 2023, Hoang Giang Nguyen - Institute for Artificial Intelligence, University Bremen

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

#include <jsoncpp/json/json.h>
#include <thread>

enum class EMultiverseClientState : unsigned char
{
    None,
    StartConnection,
    BindSendMetaData,
    SendMetaData,
    ReceiveMetaData,
    BindReceiveMetaData,
    InitSendAndReceiveData,
    BindSendData,
    SendData,
    ReceiveData,
    BindReceiveData
};

class MultiverseClient
{
public:
    /**
     * @brief Connect the socket
     *
     */
    void connect(const std::string &in_host, const std::string &in_port);

    /**
     * @brief Communicate with the server
     *
     */
    void communicate(const bool resend_meta_data = false);

    /**
     * @brief Send close signal to the server
     *
     */
    void disconnect();

public:
    /**
     * @brief Get the current time in time_unit
     *
     * @return double
     */
    virtual double get_time_now();

protected:
    /**
     * @brief Send the meta data and receive the response from server
     *
     */
    void send_and_receive_meta_data();

protected:
    /**
     * @brief Start send and receive meta data thread
     *
     */
    virtual void start_meta_data_thread() = 0;

    /**
     * @brief Wait for send and receive meta data thread to finish
     *
     */
    virtual void wait_for_meta_data_thread_finish() = 0;

    /**
     * @brief Initalize the objects from configuration
     *
     */
    virtual bool init_objects() = 0;

    /**
     * @brief Bind the meta data from the objects
     *
     */
    virtual void bind_send_meta_data() = 0;

    /**
     * @brief Bind the objects from the receive meta data
     *
     */
    virtual void bind_receive_meta_data() = 0;

    /**
     * @brief Initialize the send and receive data
     * 
     */
    virtual void init_send_and_receive_data() = 0;

    /**
     * @brief Bind the send buffer from the send data
     *
     */
    virtual void bind_send_data() = 0;

    /**
     * @brief Bind the receive data from the receive buffer
     *
     */
    virtual void bind_receive_data() = 0;

    /**
     * @brief Clean up pointer
     *
     */
    virtual void clean_up() = 0;

private:
    bool connect_to_server();

    void run();

    void send_meta_data();

    void receive_meta_data();

    bool check_buffer_size();

    void init_buffer();

protected:
    std::string server_socket_addr = "tcp://127.0.0.1:7000";

    std::string host;

    std::string port;

    size_t send_buffer_size = 1;

    size_t receive_buffer_size = 1;

    double *send_buffer;

    double *receive_buffer;

    Json::Value send_meta_data_json;

    std::string send_meta_data_str;

    Json::Value receive_meta_data_json;

    std::string receive_meta_data_str;

private:
    std::thread connect_to_server_thread;

    std::string socket_addr;

    EMultiverseClientState flag;

    void *context;

    void *socket_client;

    Json::Reader reader;

    bool should_shut_down = false;
};