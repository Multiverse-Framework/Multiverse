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

#include "multiverse_client.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <zmq.hpp>

std::map<std::string, size_t> attribute_map = {
    {"", 0},
    {"position", 3},
    {"quaternion", 4},
    {"relative_velocity", 6},
    {"joint_rvalue", 1},
    {"joint_tvalue", 1},
    {"joint_position", 3},
    {"joint_quaternion", 4},
    {"force", 3},
    {"torque", 3}};

bool MultiverseClient::connect_to_server()
{
    zmq_disconnect(socket_client, socket_addr.c_str());

    if (should_shut_down)
    {
        return false;
    }

    zmq_connect(socket_client, server_socket_addr.c_str());

    zmq_send(socket_client, socket_addr.c_str(), socket_addr.size(), 0);

    std::string receive_socket_addr;
    try
    {
        zmq_msg_t message;
        zmq_msg_init(&message);
        zmq_msg_recv(&message, socket_client, 0);
        receive_socket_addr = static_cast<char *>(zmq_msg_data(&message)), zmq_msg_size(&message);
        zmq_msg_close(&message);
    }
    catch (const zmq::error_t &e)
    {
        should_shut_down = true;
        printf("[Client] %s, prepares to disconnect from server socket %s.", e.what(), server_socket_addr.c_str());
    }

    zmq_disconnect(socket_client, server_socket_addr.c_str());

    return socket_addr.compare(receive_socket_addr) == 0;
}

void MultiverseClient::connect(const std::string &in_host, const std::string &in_port)
{
    flag = EMultiverseClientState::None;

    host = in_host;
    port = in_port;
    socket_addr = host + ":" + port;

    clean_up();

    if (!init_objects())
    {
        return;
    }

    context = zmq_ctx_new();
    socket_client = zmq_socket(context, ZMQ_REQ);

    if (connect_to_server_thread.joinable())
    {
        connect_to_server_thread.join();
    }

    connect_to_server_thread = std::thread([&]()
                                           {
        if (!connect_to_server())
        {
            return;
        }
        
        flag = EMultiverseClientState::StartConnection;

        printf("[Client %s] Opened the socket %s.\n", port.c_str(), socket_addr.c_str());

        run(); });
}

double MultiverseClient::get_time_now()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000000.0;
}

void MultiverseClient::run()
{
    while (!should_shut_down)
    {
        switch (flag)
        {
        case EMultiverseClientState::StartConnection:
            zmq_disconnect(socket_client, socket_addr.c_str());
            zmq_connect(socket_client, socket_addr.c_str());

            flag = EMultiverseClientState::BindSendMetaData;
            break;

        case EMultiverseClientState::BindSendMetaData:
            send_meta_data_json = Json::Value();
            bind_send_meta_data();

            send_meta_data_str = send_meta_data_json.toStyledString();
            printf("[Client %s] Sending meta data to the server:\n%s", port.c_str(), send_meta_data_str.c_str());

            start_meta_data_thread();
            return;

        case EMultiverseClientState::SendMetaData:
            send_meta_data();

            flag = EMultiverseClientState::ReceiveMetaData;
            break;

        case EMultiverseClientState::ReceiveMetaData:
            receive_meta_data();

            if (receive_meta_data_str.empty() ||
                !reader.parse(receive_meta_data_str, receive_meta_data_json) ||
                !receive_meta_data_json.isMember("time") ||
                receive_meta_data_json["time"].asDouble() < 0)
            {
                printf("[Client %s] The socket %s from the server has been terminated, resending the meta data.\n", port.c_str(), socket_addr.c_str());

                zmq_sleep(1); // Wait for the server to terminate completely

                if (connect_to_server())
                {
                    zmq_connect(socket_client, socket_addr.c_str());
                    flag = EMultiverseClientState::SendMetaData;
                }
                else
                {
                    flag = EMultiverseClientState::None;
                }
            }
            else if (check_buffer_size())
            {
                init_buffer();
                flag = EMultiverseClientState::BindReceiveMetaData;
            }
            else
            {
                zmq_sleep(1); // Wait for the server to terminate completely

                if (connect_to_server())
                {
                    zmq_connect(socket_client, socket_addr.c_str());
                    flag = EMultiverseClientState::SendMetaData;
                }
                else
                {
                    flag = EMultiverseClientState::None;
                }
            }
            break;

        case EMultiverseClientState::BindReceiveMetaData:
            bind_receive_meta_data();

            flag = EMultiverseClientState::InitSendAndReceiveData;
            return;

        case EMultiverseClientState::InitSendAndReceiveData:
            wait_for_meta_data_thread_finish();
            init_send_and_receive_data();

            printf("[Client %s] Starting the communication (send: %ld, receive: %ld).\n", port.c_str(), send_buffer_size, receive_buffer_size);

            flag = EMultiverseClientState::BindSendData;
            break;

        case EMultiverseClientState::BindSendData:
            bind_send_data();

            flag = EMultiverseClientState::SendData;
            break;

        case EMultiverseClientState::SendData:
            send_buffer[0] = get_time_now();
            zmq_send(socket_client, send_buffer, send_buffer_size * sizeof(double), 0);

            flag = EMultiverseClientState::ReceiveData;
            break;

        case EMultiverseClientState::ReceiveData:
            zmq_recv(socket_client, receive_buffer, receive_buffer_size * sizeof(double), 0);

            if (std::isnan(*receive_buffer) || *receive_buffer < 0)
            {
                printf("[Client %s] The socket %s from the server has been terminated, returning to resend the meta data.\n", port.c_str(), socket_addr.c_str());

                if (connect_to_server_thread.joinable())
                {
                    connect_to_server_thread.join();
                }

                connect_to_server_thread = std::thread([&]()
                                                       {
                    zmq_sleep(1); // Wait for the server to terminate completely

                    if (!connect_to_server())
                    {
                        return;
                    }
                    
                    flag = EMultiverseClientState::StartConnection;

                    wait_for_meta_data_thread_finish();

                    run(); });

                return;
            }
            else
            {
                flag = EMultiverseClientState::BindReceiveData;
            }
            break;

        case EMultiverseClientState::BindReceiveData:
            bind_receive_data();

            flag = EMultiverseClientState::BindSendData;
            return;

        default:
            return;
        }
    }

    if (flag != EMultiverseClientState::ReceiveMetaData && flag != EMultiverseClientState::ReceiveData)
    {
        printf("[Client %s] Closing the socket %s.\n", port.c_str(), socket_addr.c_str());

        if (flag == EMultiverseClientState::BindSendMetaData ||
            flag == EMultiverseClientState::SendMetaData)
        {
            const std::string close_data = "{}";
            zmq_send(socket_client, close_data.c_str(), close_data.size(), 0);
        }
        else if (flag == EMultiverseClientState::BindReceiveMetaData ||
                 flag == EMultiverseClientState::InitSendAndReceiveData ||
                 flag == EMultiverseClientState::BindSendData ||
                 flag == EMultiverseClientState::SendData ||
                 flag == EMultiverseClientState::BindReceiveData)
        {
            send_buffer[0] = -1.0;
            zmq_send(socket_client, send_buffer, send_buffer_size * sizeof(double), 0);
            free(send_buffer);
            free(receive_buffer);
        }

        clean_up();

        zmq_disconnect(socket_client, socket_addr.c_str());
    }
}

void MultiverseClient::send_and_receive_meta_data()
{
    flag = EMultiverseClientState::SendMetaData;
    run();
}

void MultiverseClient::send_meta_data()
{
    zmq_send(socket_client, send_meta_data_str.c_str(), send_meta_data_str.size(), 0);
}

void MultiverseClient::receive_meta_data()
{
    zmq_msg_t message;
    zmq_msg_init(&message);
    zmq_msg_recv(&message, socket_client, 0);
    receive_meta_data_str = static_cast<char *>(zmq_msg_data(&message)), zmq_msg_size(&message);
    zmq_msg_close(&message);
}

bool MultiverseClient::check_buffer_size()
{
    bool skip_compare = false;

    std::map<std::string, size_t> request_buffer_sizes = {{"send", 1}, {"receive", 1}};
    for (std::pair<const std::string, size_t> &request_buffer_size : request_buffer_sizes)
    {
        for (const std::string &object_name : send_meta_data_json[request_buffer_size.first].getMemberNames())
        {
            if (strcmp(object_name.c_str(), "") == 0)
            {
                skip_compare = true;
                break;
            }
            for (const Json::Value &attribute : send_meta_data_json[request_buffer_size.first][object_name])
            {
                if (strcmp(attribute.asString().c_str(), "") == 0)
                {
                    skip_compare = true;
                    break;
                }
                request_buffer_size.second += attribute_map[attribute.asString()];
            }
        }
    }

    std::map<std::string, size_t> response_buffer_sizes = {{"send", 1}, {"receive", 1}};
    for (std::pair<const std::string, size_t> &response_buffer_size : response_buffer_sizes)
    {
        for (const std::string &object_name : receive_meta_data_json[response_buffer_size.first].getMemberNames())
        {
            for (const std::string &attribute_name : receive_meta_data_json[response_buffer_size.first][object_name].getMemberNames())
            {
                response_buffer_size.second += receive_meta_data_json[response_buffer_size.first][object_name][attribute_name].size();
            }
        }
    }

    if (!skip_compare &&
        (response_buffer_sizes["send"] != request_buffer_sizes["send"] || response_buffer_sizes["receive"] != request_buffer_sizes["receive"]))
    {
        printf("[Client %s] Failed to initialize the buffers %s: send_buffer_size(server = %ld, client = %ld), receive_buffer_size(server = %ld, client = %ld).\n",
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

void MultiverseClient::communicate(const bool resend_meta_data)
{
    if (should_shut_down)
    {
        return;
    }

    if (resend_meta_data)
    {
        if (flag == EMultiverseClientState::BindSendData)
        {
            clean_up();
            flag = EMultiverseClientState::BindSendMetaData;
            run();
        }
        else if (flag == EMultiverseClientState::InitSendAndReceiveData)
        {
            wait_for_meta_data_thread_finish();
            clean_up();
            flag = EMultiverseClientState::BindSendMetaData;
            run();
        }
    }
    else
    {
        if (flag == EMultiverseClientState::BindSendData || flag == EMultiverseClientState::InitSendAndReceiveData)
        {
            run();
        }
    }
}

void MultiverseClient::disconnect()
{
    should_shut_down = true;

    zmq_ctx_shutdown(context);

    wait_for_meta_data_thread_finish();

    if (connect_to_server_thread.joinable())
    {
        connect_to_server_thread.join();
    }
}