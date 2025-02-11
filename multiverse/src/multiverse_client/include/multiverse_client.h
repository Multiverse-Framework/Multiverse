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
    TypedBuffer<uint16_t> buffer_uint16_t;
};

enum class EMultiverseClientState : unsigned char;
class MultiverseClient
{
public:
    virtual ~MultiverseClient() = default;

public:
    /**
     * @brief Connect the socket with host, server_port, and client_port
     *
     */
    void connect(const std::string &in_host, const std::string &in_server_port, const std::string &in_client_port);

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
    virtual double get_time_now() const;

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
     * @brief Initalize the objects, either from initialization or from
     * request_meta_data_str
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
    virtual void compute_request_buffer_sizes(std::map<std::string, size_t> &send_buffer_size, std::map<std::string, size_t> &receive_buffer_size) const = 0;

    /**
     * @brief Compute response buffer sizes
     *
     * @param send_buffer_size
     * @param receive_buffer_size
     */
    virtual void compute_response_buffer_sizes(std::map<std::string, size_t> &send_buffer_size, std::map<std::string, size_t> &receive_buffer_size) const = 0;

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
    /**
     * @brief Run the Multiverse Client
     *
     */
    void run();

    /**
     * @brief Send the request meta data to the server
     *
     */
    void send_request_meta_data();

    /**
     * @brief Send the data to the server
     *
     */
    void send_send_data();

    /**
     * @brief Receive the data from the server
     *
     */
    void receive_data();

    /**
     * @brief Check the response meta data, decide whether the request is successful or not
     *
     */
    void check_response_meta_data();

    /**
     * @brief Check the buffer size
     *
     * @return true if the buffer size is correct
     * @return false if the buffer size is not correct
     */
    bool check_buffer_size();

    /**
     * @brief Initialize the buffer
     * 
     */
    void init_buffer();

protected:
    /**
     * @brief The host IP address of the server
     * 
     */
    std::string host;

    /**
     * @brief The port number of the server
     * 
     */
    std::string server_port = "7000";

    /**
     * @brief The port number of the client, each client has a unique port number
     * 
     */
    std::string client_port;

    /**
     * @brief The buffer for the send data
     * 
     */
    Buffer send_buffer;

    /**
     * @brief The buffer for the receive data
     * 
     */
    Buffer receive_buffer;

    /**
     * @brief The request_meta_data to send to the server as a string
     * 
     */
    std::string request_meta_data_str;

    /**
     * @brief The response_meta_data received from the server as a string
     * 
     */
    std::string response_meta_data_str;

    /**
     * @brief The current state of the client
     * 
     */
    std::atomic<EMultiverseClientState> flag;

    /**
     * @brief The time of the world
     * 
     */
    double *world_time = (double *)calloc(1, sizeof(double));

private:
    /**
     * @brief The socket address of the client
     * 
     */
    std::string socket_addr;

    /**
     * @brief The context of the client, used for the client socket
     * 
     */
    void *context = nullptr;

    /**
     * @brief The client socket
     * 
     */
    void *client_socket = nullptr;

    /**
     * @brief True if the client should be shut down
     * 
     */
    bool should_shut_down = false;

    /**
     * @brief Reset cool down in seconds
     * 
     */
    double reset_cool_down = 1.0;
    
    /**
     * @brief Reset time
     * 
     */
    double reset_time = 0.0;
};