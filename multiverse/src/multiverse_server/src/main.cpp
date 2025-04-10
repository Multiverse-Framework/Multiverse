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

#define _USE_MATH_DEFINES
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

#include "multiverse_server.h"

/**
 * @brief multiverse_server takes one argument as the server socket address, default is "tcp://<*>:7000", 
 * meaning the server will listen on all interfaces on port 7000.
 * 
 * @param argc Number of arguments
 * @param argv The arguments, the second argument is the server socket address
 * @return int Return 0 if successful
 */
int main(int argc, char **argv)
{
    printf("Start Multiverse Server...\n");

    // register signal SIGINT and signal handler
    signal(SIGINT, [](int signum)
           {
        printf("[Server] Caught SIGINT (Ctrl+C), wait for 1s then shutdown.\n");
        should_shut_down = true; 
        zmq_sleep(1);
        server_context.shutdown(); });

    std::string server_socket_addr;
    if (argc > 1)
    {
        server_socket_addr = std::string(argv[1]);
    }
    else
    {
        server_socket_addr = "tcp://*:7000";
    }

    std::thread multiverse_server_thread(start_multiverse_server, server_socket_addr);

    while (!should_shut_down)
    {
        zmq_sleep(0.1);
    }

    bool can_shut_down = true;
    do
    {
        can_shut_down = true;
        for (const std::pair<const std::string, bool> &socket_needs_clean_up : sockets_need_clean_up)
        {
            if (socket_needs_clean_up.second)
            {
                can_shut_down = false;
                break;
            }
        }
    } while (!can_shut_down);

    zmq_sleep(1);

    server_context.close();

    if (multiverse_server_thread.joinable())
    {
        multiverse_server_thread.join();
    }
}