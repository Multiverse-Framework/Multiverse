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

class MultiverseClient
{
public:
    /**
     * @brief Initialize the socket with host and port
     *
     */
    void init(const std::string &in_host, const int in_port);

    /**
     * @brief Connect the socket
     *
     */
    void connect();

    /**
     * @brief Communicate with the server
     *
     */
    void communicate();

    /**
     * @brief Send close signal to the server
     *
     */
    void disconnect();

public:
    /**
     * @brief Send the meta data and receive the response from server
     *
     */
    void send_and_receive_meta_data();

    /**
     * @brief Get the current time in time_unit
     *
     * @return double
     */
    virtual double get_time_now();

public:
    bool should_shut_down = false;

protected:
    /**
     * @brief Start send and receive meta data thread
     *
     */
    virtual void start_meta_data_thread() = 0;

    /**
     * @brief Stop send and receive meta data thread
     *
     */
    virtual void stop_meta_data_thread() = 0;

    /**
     * @brief Initalize the objects from configuration
     *
     */
    virtual void init_objects() = 0;

    /**
     * @brief Validate the objects with the current state
     *
     */
    virtual void validate_objects() = 0;

    /**
     * @brief Construct the meta data from the objects
     *
     */
    virtual void construct_meta_data() = 0;

    /**
     * @brief Bind object data to send and receive data
     *
     */
    virtual void bind_object_data() = 0;

    /**
     * @brief Clean up pointer
     *
     */
    virtual void clean_up() = 0;

    /**
     * @brief Bind send data to send buffer
     *
     */
    virtual void bind_send_data() = 0;

    /**
     * @brief Bind receive data from receive buffer
     *
     */
    virtual void bind_receive_data() = 0;

protected:
    size_t send_buffer_size = 1;

    size_t receive_buffer_size = 1;

    double *send_buffer;

    double *receive_buffer;

    Json::Value meta_data_json;

    Json::Value meta_data_res_json;

private:
    std::string host;

    int port;

    bool is_enabled = false;

    void *context;

    void *socket_client;

    std::string socket_addr;
};