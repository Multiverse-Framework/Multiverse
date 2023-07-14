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
#include <zmq.h>

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

std::map<EMultiverseClientState, std::string> flag_map =
    {
        {EMultiverseClientState::StartConnection, "StartConnection"},
        {EMultiverseClientState::BindSendMetaData, "BindSendMetaData"},
        {EMultiverseClientState::SendMetaData, "SendMetaData"},
        {EMultiverseClientState::ReceiveMetaData, "ReceiveMetaData"},
        {EMultiverseClientState::BindReceiveMetaData, "BindReceiveMetaData"},
        {EMultiverseClientState::InitSendAndReceiveData, "InitSendAndReceiveData"},
        {EMultiverseClientState::BindSendData, "BindSendData"},
        {EMultiverseClientState::SendData, "SendData"},
        {EMultiverseClientState::ReceiveData, "ReceiveData"},
        {EMultiverseClientState::BindReceiveData, "BindReceiveData"}};

void MultiverseClient::init(const std::string &in_host, const std::string &in_port)
{
    clean_up();

    host = in_host;
    port = in_port;
    socket_addr = host + ":" + port;

    if (!init_objects())
    {
        return;
    }

    context = zmq_ctx_new();
    socket_client = zmq_socket(context, ZMQ_REQ);

    flag = EMultiverseClientState::StartConnection;

    printf("[Client %s] Opened the socket %s.\n", port.c_str(), socket_addr.c_str());

    connect();
}

double MultiverseClient::get_time_now()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000000.0;
}

void MultiverseClient::run()
{
    while (!should_shut_down)
    {
        printf("[Client %s] STATE: %s\n", socket_addr.c_str(), flag_map[flag].c_str());
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

            // TODO: CHANGE HERE!!!
            flag = EMultiverseClientState::BindReceiveMetaData;
            return;

        case EMultiverseClientState::SendMetaData:
            send_meta_data();

            flag = EMultiverseClientState::ReceiveMetaData;
            break;

        case EMultiverseClientState::ReceiveMetaData:
            receive_meta_data();

            if (check_buffer_size())
            {
                init_buffer();
                flag = EMultiverseClientState::BindReceiveMetaData;
                printf("AAAAAAAAAAAAAAAAAAFFFFFFFFFFFFFFFFFF\n");
                return;
            }
            else
            {
                flag = EMultiverseClientState::StartConnection;
                break;
            }

        case EMultiverseClientState::BindReceiveMetaData:
            printf("AAAAAAAAAAAAAAAAAAAAAAAAAA\n");
            wait_for_meta_data_thread_finish();
            printf("BBBBBBBBBBBBBBBBBBBBBBBBBBB\n");
            bind_receive_meta_data();

            flag = EMultiverseClientState::InitSendAndReceiveData;
            break;

        case EMultiverseClientState::InitSendAndReceiveData:
            init_send_and_receive_data();

            flag = EMultiverseClientState::BindSendData;
            break;

        case EMultiverseClientState::BindSendData:
            bind_send_data();

            flag = EMultiverseClientState::SendData;
            break;

        case EMultiverseClientState::SendData:
            zmq_send(socket_client, send_buffer, send_buffer_size * sizeof(double), 0);

            flag = EMultiverseClientState::ReceiveData;
            break;

        case EMultiverseClientState::ReceiveData:
            zmq_recv(socket_client, receive_buffer, receive_buffer_size * sizeof(double), 0);

            if (std::isnan(*receive_buffer) || *receive_buffer < 0)
            {
                printf("[Client %s] The socket %s from the server has been terminated, returning to resend the meta data.\n", port.c_str(), socket_addr.c_str());
                flag = EMultiverseClientState::StartConnection;
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

void MultiverseClient::connect()
{
    flag = EMultiverseClientState::StartConnection;

    run();

    printf("[Client %s] Starting the communication (send: %ld, receive: %ld).\n", port.c_str(), send_buffer_size, receive_buffer_size);
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

    std::string receive_meta_data_str(static_cast<char *>(zmq_msg_data(&message)), zmq_msg_size(&message));

    zmq_msg_close(&message);

    if (!receive_meta_data_str.empty())
    {
        reader.parse(receive_meta_data_str, receive_meta_data_json);
    }
}

bool MultiverseClient::check_buffer_size()
{
    if (receive_meta_data_json.empty())
    {
        printf("[Client %s] The socket %s from the server has been terminated, resending the meta data.\n", port.c_str(), socket_addr.c_str());
        return false;
    }

    std::map<std::string, size_t> request_buffer_sizes = {{"send", 1}, {"receive", 1}};
    for (std::pair<const std::string, size_t> &request_buffer_size : request_buffer_sizes)
    {
        for (const std::string &object_name : send_meta_data_json[request_buffer_size.first].getMemberNames())
        {
            for (const Json::Value &attribute : send_meta_data_json[request_buffer_size.first][object_name])
            {
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

    if (!send_meta_data_json["receive"].isMember("") &&
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
    if (resend_meta_data && flag == EMultiverseClientState::SendData)
    {
        clean_up();
        flag = EMultiverseClientState::BindSendMetaData;
        run();
    }
    else if (!resend_meta_data && flag == EMultiverseClientState::SendData)
    {
        run();
    }
}

void MultiverseClient::disconnect()
{
    should_shut_down = true;

    zmq_ctx_shutdown(context);

    wait_for_meta_data_thread_finish();
}