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
    if (!client_socket)
    {
        return;
    }

    zmq_disconnect(client_socket, socket_addr.c_str());

    if (should_shut_down)
    {
        return;
    }

    const EMultiverseClientState current_flag = flag.load();
    if (current_flag == EMultiverseClientState::ReceiveData || current_flag == EMultiverseClientState::ReceiveResponseMetaData)
    {
        zmq_sleep(1); // sleep for 1 second to finish the previous communication
    }

    const std::string server_socket_addr = host + ":" + server_port;

    zmq_connect(client_socket, server_socket_addr.c_str());

    zmq_msg_t request;
    zmq_msg_init_size(&request, socket_addr.size());
    memcpy(zmq_msg_data(&request), socket_addr.c_str(), socket_addr.size());
    printf("[Client %s] Sending request %s to %s.\n", client_port.c_str(), socket_addr.c_str(), server_socket_addr.c_str());
    zmq_msg_send(&request, client_socket, 0);
    printf("[Client %s] Sent request %s to %s.\n", client_port.c_str(), socket_addr.c_str(), server_socket_addr.c_str());
    zmq_msg_close(&request);

    std::string receive_socket_addr;
    try
    {
        zmq_msg_t response;
        zmq_msg_init(&response);
        zmq_msg_recv(&response, client_socket, 0);
        receive_socket_addr = std::string((char *)zmq_msg_data(&response), zmq_msg_size(&response));
        printf("[Client %s] Received response %s from %s.\n", client_port.c_str(), socket_addr.c_str(), server_socket_addr.c_str());
        zmq_msg_close(&response);
    }
    catch (const zmq::error_t &e)
    {
        should_shut_down = true;
        printf("[Client %s] %s, prepares to disconnect from server socket %s.", client_port.c_str(), e.what(), server_socket_addr.c_str());
    }

    zmq_disconnect(client_socket, server_socket_addr.c_str());

    if (socket_addr.compare(receive_socket_addr) != 0)
    {
        flag = EMultiverseClientState::None;
    }
    else if (current_flag == EMultiverseClientState::None || current_flag == EMultiverseClientState::ReceiveData)
    {
        flag = EMultiverseClientState::StartConnection;

        printf("[Client %s] Opened the socket %s.\n", client_port.c_str(), socket_addr.c_str());
    }
    else if (current_flag == EMultiverseClientState::ReceiveResponseMetaData)
    {
        zmq_connect(client_socket, socket_addr.c_str());

        flag = EMultiverseClientState::SendRequestMetaData;
    }
}

void MultiverseClient::connect(const std::string &in_host, const std::string &in_server_port, const std::string &in_client_port)
{
    host = in_host;

    server_port = in_server_port;

    client_port = in_client_port;

    connect();
}

void MultiverseClient::start()
{
    const EMultiverseClientState current_flag = flag.load();
    if (current_flag == EMultiverseClientState::StartConnection)
    {
        printf("[Client %s] Start.\n", client_port.c_str());
        run();
    }
}

void MultiverseClient::connect()
{
    flag = EMultiverseClientState::None;

    socket_addr = host + ":" + client_port;

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

double MultiverseClient::get_time_now() const
{
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000000.0;
}

void MultiverseClient::run()
{
    while (!should_shut_down)
    {
        const EMultiverseClientState current_flag = flag.load();
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
            return;

        case EMultiverseClientState::SendRequestMetaData:
            send_request_meta_data();

            flag = EMultiverseClientState::ReceiveResponseMetaData;
            break;

        case EMultiverseClientState::ReceiveResponseMetaData:
            receive_data();

            check_response_meta_data();
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

            printf("[Client %s] Starting the communication (send: [%zu - %zu - %zu], receive: [%zu - %zu - %zu]).\n",
                   client_port.c_str(),
                   send_buffer.buffer_double.size,
                   send_buffer.buffer_uint8_t.size,
                   send_buffer.buffer_uint16_t.size,
                   receive_buffer.buffer_double.size,
                   receive_buffer.buffer_uint8_t.size,
                   receive_buffer.buffer_uint16_t.size);

            flag = EMultiverseClientState::BindSendData;
            break;

        case EMultiverseClientState::BindSendData:
            bind_send_data();

            flag = EMultiverseClientState::SendData;
            break;

        case EMultiverseClientState::SendData:
            send_send_data();

            flag = EMultiverseClientState::ReceiveData;
            break;

        case EMultiverseClientState::ReceiveData:
            receive_data();
            break;

        case EMultiverseClientState::BindReceiveData:
            bind_receive_data();

            flag = EMultiverseClientState::BindSendData;
            return;

        default:
            return;
        }
    }

    const EMultiverseClientState current_flag = flag.load();
    if (current_flag != EMultiverseClientState::ReceiveResponseMetaData && current_flag != EMultiverseClientState::ReceiveData)
    {
        printf("[Client %s] Closing the socket %s.\n", client_port.c_str(), socket_addr.c_str());

        if (current_flag == EMultiverseClientState::BindRequestMetaData ||
            current_flag == EMultiverseClientState::SendRequestMetaData ||
            current_flag == EMultiverseClientState::BindResponseMetaData ||
            current_flag == EMultiverseClientState::InitSendAndReceiveData ||
            current_flag == EMultiverseClientState::BindSendData ||
            current_flag == EMultiverseClientState::SendData ||
            current_flag == EMultiverseClientState::BindReceiveData)
        {
            const int message_int = 0;
            zmq_send(client_socket, &message_int, sizeof(message_int), 0);
            if (send_buffer.buffer_double.data != nullptr)
            {
                free(send_buffer.buffer_double.data);
                send_buffer.buffer_double.data = nullptr;
            }
            if (send_buffer.buffer_uint8_t.data != nullptr)
            {
                free(send_buffer.buffer_uint8_t.data);
                send_buffer.buffer_uint8_t.data = nullptr;
            }
            if (send_buffer.buffer_uint16_t.data != nullptr)
            {
                free(send_buffer.buffer_uint16_t.data);
                send_buffer.buffer_uint16_t.data = nullptr;
            }
            if (receive_buffer.buffer_double.data != nullptr)
            {
                free(receive_buffer.buffer_double.data);
                receive_buffer.buffer_double.data = nullptr;
            }
            if (receive_buffer.buffer_uint8_t.data != nullptr)
            {
                free(receive_buffer.buffer_uint8_t.data);
                receive_buffer.buffer_uint8_t.data = nullptr;
            }
            if (receive_buffer.buffer_uint16_t.data != nullptr)
            {
                free(receive_buffer.buffer_uint16_t.data);
                receive_buffer.buffer_uint16_t.data = nullptr;
            }
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
    if (should_shut_down)
    {
        const int message_spec_int = 0;
        zmq_send(client_socket, &message_spec_int, sizeof(int), 0);
    }
    else
    {
        const int message_spec_int = 1;
        zmq_send(client_socket, &message_spec_int, sizeof(int), 2);
        zmq_send(client_socket, request_meta_data_str.c_str(), request_meta_data_str.size(), 0);
    }
}

void MultiverseClient::send_send_data()
{
    const int message_spec_int = 2 + (send_buffer.buffer_double.size > 0) + (send_buffer.buffer_uint8_t.size > 0) + (send_buffer.buffer_uint16_t.size > 0);
    zmq_send(client_socket, &message_spec_int, sizeof(int), 2);

    if (message_spec_int == 2)
    {
        zmq_send(client_socket, world_time, sizeof(double), 0);
    }
    else
    {
        zmq_send(client_socket, world_time, sizeof(double), 2);
        if (send_buffer.buffer_double.size > 0)
        {
            if (send_buffer.buffer_uint8_t.size == 0 && send_buffer.buffer_uint16_t.size == 0)
            {
                zmq_send(client_socket, send_buffer.buffer_double.data, send_buffer.buffer_double.size * sizeof(double), 0);
            }
            else
            {
                zmq_send(client_socket, send_buffer.buffer_double.data, send_buffer.buffer_double.size * sizeof(double), 2);
            }
        }
        if (send_buffer.buffer_uint8_t.size > 0)
        {
            if (send_buffer.buffer_uint16_t.size == 0)
            {
                zmq_send(client_socket, send_buffer.buffer_uint8_t.data, send_buffer.buffer_uint8_t.size * sizeof(uint8_t), 0);
            }
            else
            {
                zmq_send(client_socket, send_buffer.buffer_uint8_t.data, send_buffer.buffer_uint8_t.size * sizeof(uint8_t), 2);
            }
        }
        if (send_buffer.buffer_uint16_t.size > 0)
        {
            zmq_send(client_socket, send_buffer.buffer_uint16_t.data, send_buffer.buffer_uint16_t.size * sizeof(uint16_t), 0);
        }
    }
}

void MultiverseClient::receive_data()
{
    int message_spec_int;
    if (zmq_recv(client_socket, &message_spec_int, sizeof(int), 0) == -1)
    {
        should_shut_down = true;
        return;
    }

    if (message_spec_int == 0)
    {
        should_shut_down = true;
        return;
    }
    else if (message_spec_int == 1)
    {
        zmq_msg_t message;
        zmq_msg_init(&message);
        zmq_msg_recv(&message, client_socket, 0);
        response_meta_data_str = std::string(static_cast<char *>(zmq_msg_data(&message)), zmq_msg_size(&message));
        zmq_msg_close(&message);
        const EMultiverseClientState current_flag = flag.load();
        if (current_flag == EMultiverseClientState::ReceiveResponseMetaData)
        {
            flag = EMultiverseClientState::BindResponseMetaData;
        }
        else if (current_flag == EMultiverseClientState::ReceiveData)
        {
            printf("[Client %s] The socket %s from the server has received new meta data.\n", client_port.c_str(), socket_addr.c_str());
            check_response_meta_data();
            bind_api_callbacks();
            init_objects(true);
            bind_api_callbacks_response();
            flag = EMultiverseClientState::BindRequestMetaData;
        }
        else
        {
            throw std::runtime_error("[Client " + client_port + "] The client is in the wrong state.");
        }
        return;
    }
    else if (message_spec_int >= 2)
    {
        zmq_recv(client_socket, world_time, sizeof(*world_time), 0);
        if (message_spec_int == 3)
        {
            if (receive_buffer.buffer_double.size > 0 && receive_buffer.buffer_uint8_t.size == 0 && receive_buffer.buffer_uint16_t.size == 0)
            {
                zmq_recv(client_socket, receive_buffer.buffer_double.data, receive_buffer.buffer_double.size * sizeof(double), 0);
            }
            else if (receive_buffer.buffer_double.size == 0 && receive_buffer.buffer_uint8_t.size > 0 && receive_buffer.buffer_uint16_t.size == 0)
            {
                zmq_recv(client_socket, receive_buffer.buffer_uint8_t.data, receive_buffer.buffer_uint8_t.size * sizeof(uint8_t), 0);
            }
            else if (receive_buffer.buffer_double.size == 0 && receive_buffer.buffer_uint8_t.size == 0 && receive_buffer.buffer_uint16_t.size > 0)
            {
                zmq_recv(client_socket, receive_buffer.buffer_uint16_t.data, receive_buffer.buffer_uint16_t.size * sizeof(uint16_t), 0);
            }
            else
            {
                throw std::runtime_error("The receive buffer is not initialized correctly.");
            }
        }
        else if (message_spec_int == 4)
        {
            zmq_recv(client_socket, world_time, sizeof(double), 0);
            if (receive_buffer.buffer_uint16_t.size == 0)
            {
                zmq_recv(client_socket, receive_buffer.buffer_double.data, receive_buffer.buffer_double.size * sizeof(double), 0);
                zmq_recv(client_socket, receive_buffer.buffer_uint8_t.data, receive_buffer.buffer_uint8_t.size * sizeof(uint8_t), 0);
            }
            else if (receive_buffer.buffer_double.size == 0)
            {
                zmq_recv(client_socket, receive_buffer.buffer_uint8_t.data, receive_buffer.buffer_uint8_t.size * sizeof(uint8_t), 0);
                zmq_recv(client_socket, receive_buffer.buffer_uint16_t.data, receive_buffer.buffer_uint16_t.size * sizeof(uint16_t), 0);
            }
            else if (receive_buffer.buffer_uint8_t.size == 0)
            {
                zmq_recv(client_socket, receive_buffer.buffer_double.data, receive_buffer.buffer_double.size * sizeof(double), 0);
                zmq_recv(client_socket, receive_buffer.buffer_uint16_t.data, receive_buffer.buffer_uint16_t.size * sizeof(uint16_t), 0);
            }
            else
            {
                throw std::runtime_error("The receive buffer is not initialized correctly.");
            }
        }
        else if (message_spec_int == 5)
        {
            zmq_recv(client_socket, world_time, sizeof(double), 0);
            zmq_recv(client_socket, receive_buffer.buffer_double.data, receive_buffer.buffer_double.size * sizeof(double), 0);
            zmq_recv(client_socket, receive_buffer.buffer_uint8_t.data, receive_buffer.buffer_uint8_t.size * sizeof(uint8_t), 0);
            zmq_recv(client_socket, receive_buffer.buffer_uint16_t.data, receive_buffer.buffer_uint16_t.size * sizeof(uint16_t), 0);
        }
        else if (message_spec_int != 2)
        {
            throw std::runtime_error("The message type [" + std::to_string(message_spec_int) + "] is not recognized.");
        }
    }
    else
    {
        throw std::runtime_error("The message type [" + std::to_string(message_spec_int) + "] is not recognized.");
    }

    if (!should_shut_down && *world_time == 0.0)
    {
        const double time_now = get_time_now();
        if (time_now - reset_time > reset_cool_down)
        {
            printf("[Client %s] The socket %s from the server has received reset command.\n", client_port.c_str(), socket_addr.c_str());
            reset_time = time_now;
            reset();
        }
    }

    flag = EMultiverseClientState::BindReceiveData;
}

void MultiverseClient::check_response_meta_data()
{
    if (should_shut_down)
    {
        flag = EMultiverseClientState::BindResponseMetaData;
    }
    else if (compute_request_and_response_meta_data() && check_buffer_size())
    {
        init_buffer();
        flag = EMultiverseClientState::BindResponseMetaData;
    }
    else
    {
        throw std::runtime_error("[Client " + client_port + "] The client failed to check the response meta data.");
    }
}

bool MultiverseClient::check_buffer_size()
{
    std::map<std::string, std::map<std::string, size_t>> request_buffer_sizes =
        {{"send", {{"double", 0}, {"uint8", 0}, {"uint16", 0}}}, {"receive", {{"double", 0}, {"uint8", 0}, {"uint16", 0}}}};
    compute_request_buffer_sizes(request_buffer_sizes["send"], request_buffer_sizes["receive"]);

    std::map<std::string, std::map<std::string, size_t>> response_buffer_sizes =
        {{"send", {{"double", 0}, {"uint8", 0}, {"uint16", 0}}}, {"receive", {{"double", 0}, {"uint8", 0}, {"uint16", 0}}}};
    compute_response_buffer_sizes(response_buffer_sizes["send"], response_buffer_sizes["receive"]);

    if (request_buffer_sizes["receive"]["double"] != -1 && request_buffer_sizes["receive"]["uint8"] != -1 && request_buffer_sizes["receive"]["uint16"] != -1 &&
        (request_buffer_sizes["send"]["double"] != response_buffer_sizes["send"]["double"] ||
         request_buffer_sizes["send"]["uint8"] != response_buffer_sizes["send"]["uint8"] ||
         request_buffer_sizes["send"]["uint16"] != response_buffer_sizes["send"]["uint16"] ||
         request_buffer_sizes["receive"]["double"] != response_buffer_sizes["receive"]["double"] ||
         request_buffer_sizes["receive"]["uint8"] != response_buffer_sizes["receive"]["uint8"] ||
         request_buffer_sizes["receive"]["uint16"] != response_buffer_sizes["receive"]["uint16"]))
    {
        printf("[Client %s] Failed to initialize the buffers %s: send_buffer_size(server = [%zu - %zu - %zu], client = [%zu - %zu - %zu]), receive_buffer_size(server = [%zu - %zu - %zu], client = [%zu - %zu - %zu]).\n",
               client_port.c_str(),
               socket_addr.c_str(),
               request_buffer_sizes["send"]["double"],
               request_buffer_sizes["send"]["uint8"],
               request_buffer_sizes["send"]["uint16"],
               response_buffer_sizes["send"]["double"],
               response_buffer_sizes["send"]["uint8"],
               response_buffer_sizes["send"]["uint16"],
               request_buffer_sizes["receive"]["double"],
               request_buffer_sizes["receive"]["uint8"],
               request_buffer_sizes["receive"]["uint16"],
               response_buffer_sizes["receive"]["double"],
               response_buffer_sizes["receive"]["uint8"],
               response_buffer_sizes["receive"]["uint16"]);
        return false;
    }

    send_buffer.buffer_double.size = response_buffer_sizes["send"]["double"];
    send_buffer.buffer_uint8_t.size = response_buffer_sizes["send"]["uint8"];
    send_buffer.buffer_uint16_t.size = response_buffer_sizes["send"]["uint16"];
    receive_buffer.buffer_double.size = response_buffer_sizes["receive"]["double"];
    receive_buffer.buffer_uint8_t.size = response_buffer_sizes["receive"]["uint8"];
    receive_buffer.buffer_uint16_t.size = response_buffer_sizes["receive"]["uint16"];
    return true;
}

void MultiverseClient::init_buffer()
{
    send_buffer.buffer_double.data = (double *)calloc(send_buffer.buffer_double.size, sizeof(double));
    send_buffer.buffer_uint8_t.data = (uint8_t *)calloc(send_buffer.buffer_uint8_t.size, sizeof(uint8_t));
    send_buffer.buffer_uint16_t.data = (uint16_t *)calloc(send_buffer.buffer_uint16_t.size, sizeof(uint16_t));
    receive_buffer.buffer_double.data = (double *)calloc(receive_buffer.buffer_double.size, sizeof(double));
    receive_buffer.buffer_uint8_t.data = (uint8_t *)calloc(receive_buffer.buffer_uint8_t.size, sizeof(uint8_t));
    receive_buffer.buffer_uint16_t.data = (uint16_t *)calloc(receive_buffer.buffer_uint16_t.size, sizeof(uint16_t));
}

bool MultiverseClient::communicate(const bool resend_request_meta_data)
{
    const EMultiverseClientState current_flag = flag.load();
    if (should_shut_down || current_flag == EMultiverseClientState::None)
    {
        return false;
    }

    if (current_flag == EMultiverseClientState::StartConnection)
    {
        run();
        return true;
    }

    if (resend_request_meta_data)
    {
        wait_for_meta_data_thread_finish();
        if (current_flag == EMultiverseClientState::BindSendData)
        {
            init_objects();
        }
        clean_up();

        flag = EMultiverseClientState::BindRequestMetaData;

        run();
        return true;
    }

    if (current_flag == EMultiverseClientState::BindSendData || current_flag == EMultiverseClientState::InitSendAndReceiveData)
    {
        run();

        return true;
    }

    return false;
}

void MultiverseClient::disconnect()
{
    if (context == nullptr)
    {
        return;
    }
    should_shut_down = true;

    run();

    zmq_ctx_shutdown(context);

    wait_for_meta_data_thread_finish();

    wait_for_connect_to_server_thread_finish();
}
