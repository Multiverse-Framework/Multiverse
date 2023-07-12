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

#include <chrono>
#include <zmq.h>

void MultiverseClient::init(const std::string &in_host, const int in_port)
{
    host = in_host;
    port = in_port;

    init_objects();
}

void MultiverseClient::connect()
{
    context = zmq_ctx_new();

    socket_client = zmq_socket(context, ZMQ_REQ);
    socket_addr = host + ":" + std::to_string(port);

    printf("Open the socket connection on %s\n", socket_addr.c_str());
    zmq_disconnect(socket_client, socket_addr.c_str());
    zmq_connect(socket_client, socket_addr.c_str());
    clean_up();
    construct_meta_data();
    start_meta_data_thread();
}

void MultiverseClient::communicate()
{
    if (is_enabled)
    {
        bind_send_data();

        zmq_send(socket_client, send_buffer, send_buffer_size * sizeof(double), 0);

        zmq_recv(socket_client, receive_buffer, receive_buffer_size * sizeof(double), 0);

        if (*receive_buffer < 0)
        {
            is_enabled = false;
            printf("The socket server at %s has been terminated, returning to resend the meta data.\n", socket_addr.c_str());
            stop_meta_data_thread();
            zmq_disconnect(socket_client, socket_addr.c_str());
            zmq_connect(socket_client, socket_addr.c_str());
            start_meta_data_thread();
            return;
        }

        bind_receive_data();
    }
}

void MultiverseClient::disconnect()
{
    if (is_enabled)
    {
        printf("Closing the socket client on %s.\n", socket_addr.c_str());
        const std::string close_data = "{}";

        zmq_send(socket_client, close_data.c_str(), close_data.size(), 0);

        free(send_buffer);
        free(receive_buffer);

        clean_up();

        zmq_disconnect(socket_client, socket_addr.c_str());

        is_enabled = false;
    }

    zmq_ctx_shutdown(context);

    stop_meta_data_thread();
}

double MultiverseClient::get_time_now()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000000.0;
}

void MultiverseClient::send_and_receive_meta_data()
{
    const std::string meta_data_str = meta_data_json.toStyledString();
    printf("%s", meta_data_str.c_str());

    zmq_msg_t response_message;
    zmq_msg_init(&response_message);

    std::map<std::string, size_t> response_buffer_sizes = {{"send", 1}, {"receive", 1}};

    Json::Reader reader;
    while (!should_shut_down)
    {
        // Send JSON string over ZMQ
        zmq_send(socket_client, meta_data_str.c_str(), meta_data_str.size(), 0);

        // Receive response over ZMQ
        zmq_msg_recv(&response_message, socket_client, 0);

        std::string response_message_str(static_cast<char *>(zmq_msg_data(&response_message)), zmq_msg_size(&response_message));

        double* buffer = static_cast<double*>(zmq_msg_data(&response_message));
        
        if (response_message_str.empty() || !reader.parse(response_message_str, meta_data_res_json) || meta_data_res_json["time"].asDouble() < 0)
        {
            zmq_msg_init(&response_message);
            printf("The socket server at %s has been terminated, resending the meta data\n", socket_addr.c_str());
            zmq_disconnect(socket_client, socket_addr.c_str());
            zmq_connect(socket_client, socket_addr.c_str());
        }
        else
        {
            for (std::pair<const std::string, size_t> &response_buffer_size : response_buffer_sizes)
            {
                const Json::Value json_object = meta_data_res_json[response_buffer_size.first];
                for (auto object_it = json_object.begin(); object_it != json_object.end(); ++object_it)
                {
                    const std::string object_name = object_it.key().asString();
                    const Json::Value json_object_data = json_object[object_name];
                    for (auto object_data_it = json_object_data.begin(); object_data_it != json_object_data.end(); ++object_data_it)
                    {
                        const std::string attribute_name = object_data_it.key().asString();
                        response_buffer_size.second += json_object_data[attribute_name].size();
                    }
                }
            }

            break;
        }
    }

    if (!meta_data_json["receive"].isMember("") && (response_buffer_sizes["send"] != send_buffer_size || response_buffer_sizes["receive"] != receive_buffer_size))
    {
        printf("Failed to initialize the socket at %s: send_buffer_size(server = %ld, client = %ld), receive_buffer_size(server = %ld, client = %ld).\n",
            socket_addr.c_str(),
            response_buffer_sizes["send"],
            send_buffer_size,
            response_buffer_sizes["receive"],
            receive_buffer_size);
        zmq_disconnect(socket_client, socket_addr.c_str());
        return;
    }

    send_buffer_size = response_buffer_sizes["send"];
    receive_buffer_size = response_buffer_sizes["receive"];

    printf("Initialized the socket at %s successfully.\n", socket_addr.c_str());
    printf("Start communication on %s (send: %ld, receive: %ld)\n", socket_addr.c_str(), send_buffer_size, receive_buffer_size);
    send_buffer = (double *)calloc(send_buffer_size, sizeof(double));
    receive_buffer = (double *)calloc(receive_buffer_size, sizeof(double));
    is_enabled = true;

    bind_object_data();
}