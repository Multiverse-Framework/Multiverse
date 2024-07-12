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

#include <map>
#include <string>
#include <atomic>

template<class T>
struct TypedBuffer
{
    T *data;
    size_t size = 0;
};

struct Buffer
{
    TypedBuffer<double> buffer_double;
    TypedBuffer<uint8_t> buffer_uint8_t;
};

enum class EMultiverseClientState : unsigned char;
class MultiverseClient
{
public:
    virtual ~MultiverseClient() = default;

public:
    /**
     * @brief Connect the socket with host and port
     *
     */
    void connect(const std::string &in_host, const std::string &in_port);

    /**
     * @brief Connect the socket
     *
     */
    void connect();

    /**
     * @brief Start the client
     * 
     */
    void start();

    /**
     * @brief Communicate with the server
     * 
     * @param resend_request_meta_data 
     * @return true if the communication is successful
     * @return false if the communication is not successful
     */
    virtual bool communicate(const bool resend_request_meta_data = false);

    /**
     * @brief Send close signal to the server
     *
     */
    void disconnect();

public:
    /**
     * @brief Get the current time in time_unit
     *
     * @return double current time in time_unit
     */
    virtual double get_time_now();

public:
    std::string host;

    std::string port;

protected:
    /**
     * @brief Send the meta data and receive the response from server
     *
     */
    void send_and_receive_meta_data();

    /**
     * @brief Send request to the server to open a socket
     *
     */
    void connect_to_server();

protected:
    /**
     * @brief Start connect_to_server thread
     *
     */
    virtual void start_connect_to_server_thread() = 0;

    /**
     * @brief Wait for connect_to_server thread to finish
     *
     */
    virtual void wait_for_connect_to_server_thread_finish() = 0;

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
     * @brief Initalize the objects from request_meta_data
     *
     */
    virtual bool init_objects(bool from_request_meta_data = false) = 0;

    /**
     * @brief Bind the meta data from the objects
     *
     */
    virtual void bind_request_meta_data() = 0;

    /**
     * @brief Compute request_meta_data and response_meta_data from response_meta_data_str
     *
     * @return true
     * @return false
     */
    virtual bool compute_request_and_response_meta_data() = 0;

    /**
     * @brief Compute request buffer sizes
     *
     * @param send_buffer_size
     * @param receive_buffer_size
     */
    virtual void compute_request_buffer_sizes(std::map<std::string, size_t> &req_send_buffer_size, std::map<std::string, size_t> &req_receive_buffer_size) const = 0;

    /**
     * @brief Compute response buffer sizes
     *
     * @param send_buffer_size
     * @param receive_buffer_size
     */
    virtual void compute_response_buffer_sizes(std::map<std::string, size_t> &res_send_buffer_size, std::map<std::string, size_t> &res_receive_buffer_size) const = 0;

    /**
     * @brief Bind the objects from the receive meta data
     *
     */
    virtual void bind_response_meta_data() = 0;

    /**
     * @brief Bind the API callbacks
     * 
     * @param api_callbacks 
     */
    virtual void bind_api_callbacks() = 0;

    /**
     * @brief Bind the API callbacks response
     * 
     */
    virtual void bind_api_callbacks_response() = 0;

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

    /**
     * @brief Reset the simulation
     * 
     */
    virtual void reset() = 0;

private:
    void run();

    void send_request_meta_data();

    void send_send_data();

    void receive_data();

    void check_response_meta_data();

    bool check_buffer_size();

    void init_buffer();

protected:
    std::string server_socket_addr = "tcp://127.0.0.1:7000";

    Buffer send_buffer;

    Buffer receive_buffer;

    std::string request_meta_data_str;

    std::string response_meta_data_str;

    std::atomic<EMultiverseClientState> flag;

    double *world_time = (double *)calloc(0.0, sizeof(double));

private:
    std::string socket_addr;

    void *context;

    void *client_socket;

    bool should_shut_down = false;
};