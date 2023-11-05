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

#include "multiverse_client.h"

#include <chrono>
#include <cmath>
#include <map>
#include <zmq.hpp>

#define STRING_SIZE 200

enum class EMultiverseClientState : unsigned char
{
    None,
    StartConnection,
    BindRequestMetaData,
    SendRequestMetaData,
    ReceiveResponseMetaData,
    BindResponseMetaData,
    InitSendAndReceiveData,
    BindSendData,
    SendData,
    ReceiveData,
    BindReceiveData
};

void MultiverseClient::connect_to_server()
{
    zmq_disconnect(client_socket, socket_addr.c_str());

    if (should_shut_down)
    {
        return;
    }

    if (flag == EMultiverseClientState::ReceiveData || flag == EMultiverseClientState::ReceiveResponseMetaData)
    {
        zmq_sleep(1);
    }

    zmq_connect(client_socket, server_socket_addr.c_str());

    zmq_msg_t request;
    zmq_msg_init_size(&request, socket_addr.size());
    memcpy(zmq_msg_data(&request), socket_addr.c_str(), socket_addr.size());
    zmq_msg_send(&request, client_socket, 0);
    zmq_msg_close(&request);

    std::string receive_socket_addr;
    try
    {
        zmq_msg_t response;
        zmq_msg_init(&response);
        zmq_msg_recv(&response, client_socket, 0);
        receive_socket_addr = std::string((char *)zmq_msg_data(&response), zmq_msg_size(&response));
        zmq_msg_close(&response);
    }
    catch (const zmq::error_t &e)
    {
        should_shut_down = true;
        printf("[Client %s] %s, prepares to disconnect from server socket %s.", port.c_str(), e.what(), server_socket_addr.c_str());
    }

    zmq_disconnect(client_socket, server_socket_addr.c_str());

    if (socket_addr.compare(receive_socket_addr) != 0)
    {
        flag = EMultiverseClientState::None;
    }
    else if (flag == EMultiverseClientState::None || flag == EMultiverseClientState::ReceiveData)
    {
        flag = EMultiverseClientState::StartConnection;

        printf("[Client %s] Opened the socket %s.\n", port.c_str(), socket_addr.c_str());
    }
    else if (flag == EMultiverseClientState::ReceiveResponseMetaData)
    {
        zmq_connect(client_socket, socket_addr.c_str());

        flag = EMultiverseClientState::SendRequestMetaData;
    }
}

void MultiverseClient::connect(const std::string &in_host, const std::string &in_port)
{
    host = in_host;

    port = in_port;

    connect();
}

void MultiverseClient::start()
{
    if (flag == EMultiverseClientState::StartConnection)
    {
        printf("[Client %s] Start.\n", port.c_str());
        run();
    }
}

void MultiverseClient::connect()
{
    flag = EMultiverseClientState::None;

    socket_addr = host + ":" + port;

    clean_up();

    if (!init_objects())
    {
        return;
    }

    context = zmq_ctx_new();
    client_socket = zmq_socket(context, ZMQ_REQ);

    wait_for_connect_to_server_thread_finish();
    start_connect_to_server_thread();
}

double MultiverseClient::get_time_now()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000000.0;
}

void MultiverseClient::run()
{
    while (!should_shut_down)
    {
        EMultiverseClientState current_flag = flag.load();
        switch (current_flag)
        {
        case EMultiverseClientState::StartConnection:
            zmq_disconnect(client_socket, socket_addr.c_str());
            zmq_connect(client_socket, socket_addr.c_str());

            flag = EMultiverseClientState::BindRequestMetaData;
            break;

        case EMultiverseClientState::BindRequestMetaData:
            bind_request_meta_data();

            wait_for_meta_data_thread_finish();
            start_meta_data_thread();
            flag = EMultiverseClientState::InitSendAndReceiveData;
            return;

        case EMultiverseClientState::SendRequestMetaData:
            send_request_meta_data();

            flag = EMultiverseClientState::ReceiveResponseMetaData;
            break;

        case EMultiverseClientState::ReceiveResponseMetaData:
            receive_response_meta_data();

            if (should_shut_down)
            {
                flag = EMultiverseClientState::BindResponseMetaData;
            }
            else if (compute_response_meta_data() && check_buffer_size())
            {
                init_buffer();
                flag = EMultiverseClientState::BindResponseMetaData;
            }
            else
            {
                printf("[Client %s] The socket %s from the server has been terminated, resending the meta data.\n", port.c_str(), socket_addr.c_str());
                connect_to_server();
            }
            break;

        case EMultiverseClientState::BindResponseMetaData:
            bind_response_meta_data();

            flag = EMultiverseClientState::InitSendAndReceiveData;
            return;

        case EMultiverseClientState::InitSendAndReceiveData:
            wait_for_connect_to_server_thread_finish();
            wait_for_meta_data_thread_finish();
            clean_up();
            init_send_and_receive_data();

            printf("[Client %s] Starting the communication (send: %zu, receive: %zu).\n", port.c_str(), send_buffer_size, receive_buffer_size);

            flag = EMultiverseClientState::BindSendData;
            break;

        case EMultiverseClientState::BindSendData:
            bind_send_data();

            flag = EMultiverseClientState::SendData;
            break;

        case EMultiverseClientState::SendData:
            zmq_send(client_socket, send_buffer, send_buffer_size * sizeof(double), 0);

            flag = EMultiverseClientState::ReceiveData;
            break;

        case EMultiverseClientState::ReceiveData:
            zmq_recv(client_socket, receive_buffer, receive_buffer_size * sizeof(double), 0);

            if (!should_shut_down)
            {
                if (std::isnan(*receive_buffer) || *receive_buffer == -1.0)
                {
                    printf("[Client %s] The socket %s from the server has been terminated, returning to resend the meta data.\n", port.c_str(), socket_addr.c_str());

                    wait_for_connect_to_server_thread_finish();
                    start_connect_to_server_thread();
                    return;
                }
                else if (*receive_buffer == -2.0)
                {
                    printf("[Client %s] The socket %s from the server has been received new meta data.\n", port.c_str(), socket_addr.c_str());
                    send_request_meta_data();
                    zmq_msg_t message;
                    zmq_msg_init(&message);
                    zmq_msg_recv(&message, client_socket, 0);
                    request_meta_data_str = std::string(static_cast<char *>(zmq_msg_data(&message)), zmq_msg_size(&message));
                    printf("request_meta_data_str: %s\n", request_meta_data_str.c_str());
                    zmq_msg_close(&message);
                    init_objects(true);
                    flag = EMultiverseClientState::BindRequestMetaData;
                    break;
                }
            }
            flag = EMultiverseClientState::BindReceiveData;
            break;

        case EMultiverseClientState::BindReceiveData:
            bind_receive_data();

            flag = EMultiverseClientState::BindSendData;
            return;

        default:
            return;
        }
    }

    if (flag != EMultiverseClientState::ReceiveResponseMetaData && flag != EMultiverseClientState::ReceiveData)
    {
        printf("[Client %s] Closing the socket %s.\n", port.c_str(), socket_addr.c_str());

        if (flag == EMultiverseClientState::BindRequestMetaData ||
            flag == EMultiverseClientState::SendRequestMetaData ||
            flag == EMultiverseClientState::BindResponseMetaData ||
            flag == EMultiverseClientState::InitSendAndReceiveData ||
            flag == EMultiverseClientState::BindSendData ||
            flag == EMultiverseClientState::SendData ||
            flag == EMultiverseClientState::BindReceiveData)
        {
            const std::string close_data = "{}";
            zmq_send(client_socket, close_data.c_str(), close_data.size(), 0);
            free(send_buffer);
            free(receive_buffer);
        }

        clean_up();

        zmq_disconnect(client_socket, socket_addr.c_str());
    }
}

void MultiverseClient::send_and_receive_meta_data()
{
    flag = EMultiverseClientState::SendRequestMetaData;

    run();
}

void MultiverseClient::send_request_meta_data()
{
    zmq_send(client_socket, request_meta_data_str.c_str(), request_meta_data_str.size(), 0);
}

void MultiverseClient::receive_response_meta_data()
{
    zmq_msg_t message;
    zmq_msg_init(&message);
    zmq_msg_recv(&message, client_socket, 0);
    response_meta_data_str = std::string(static_cast<char *>(zmq_msg_data(&message)), zmq_msg_size(&message));
    zmq_msg_close(&message);
}

bool MultiverseClient::check_buffer_size()
{
    std::map<std::string, size_t> request_buffer_sizes = {{"send", 1}, {"receive", 1}};
    compute_request_buffer_sizes(request_buffer_sizes["send"], request_buffer_sizes["receive"]);

    std::map<std::string, size_t> response_buffer_sizes = {{"send", 1}, {"receive", 1}};
    compute_response_buffer_sizes(response_buffer_sizes["send"], response_buffer_sizes["receive"]);

    if (request_buffer_sizes["receive"] != -1 &&
        (response_buffer_sizes["send"] != request_buffer_sizes["send"] || response_buffer_sizes["receive"] != request_buffer_sizes["receive"]))
    {
        printf("[Client %s] Failed to initialize the buffers %s: send_buffer_size(server = %zu, client = %zu), receive_buffer_size(server = %zu, client = %zu).\n",
               port.c_str(),
               socket_addr.c_str(),
               response_buffer_sizes["send"],
               request_buffer_sizes["send"],
               response_buffer_sizes["receive"],
               request_buffer_sizes["receive"]);
        return false;
    }

    send_buffer_size = response_buffer_sizes["send"];
    receive_buffer_size = response_buffer_sizes["receive"];
    return true;
}

void MultiverseClient::init_buffer()
{
    send_buffer = (double *)calloc(send_buffer_size, sizeof(double));
    receive_buffer = (double *)calloc(receive_buffer_size, sizeof(double));
}

void MultiverseClient::communicate(const bool resend_request_meta_data)
{
    if (should_shut_down || flag == EMultiverseClientState::None)
    {
        return;
    }

    if (flag == EMultiverseClientState::StartConnection)
    {
        run();
    }
    else if (resend_request_meta_data)
    {
        wait_for_meta_data_thread_finish();
        if (flag == EMultiverseClientState::BindSendData)
        {
            init_objects();
        }
        clean_up();

        flag = EMultiverseClientState::BindRequestMetaData;

        run();
    }
    else if (flag == EMultiverseClientState::BindSendData || flag == EMultiverseClientState::InitSendAndReceiveData)
    {
        run();
    }
}

void MultiverseClient::disconnect()
{
    should_shut_down = true;

    run();

    zmq_ctx_shutdown(context);

    wait_for_meta_data_thread_finish();

    wait_for_connect_to_server_thread_finish();
}