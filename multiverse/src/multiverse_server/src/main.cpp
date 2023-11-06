#define _USE_MATH_DEFINES
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

#include "multiverse_server.h"

int main(int argc, char **argv)
{
    printf("Start Multiverse Server...\n");

    // register signal SIGINT and signal handler
    signal(SIGINT, [](int signum)
           {
        printf("[Server] Interrupt signal (%d) received, wait for 1s then shutdown.\n", signum);
        zmq_sleep(1);
        should_shut_down = true; 
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
    }

    bool can_shut_down = true;
    do
    {
        can_shut_down = true;
        for (const std::pair<std::string, bool> &socket_needs_clean_up : sockets_need_clean_up)
        {
            if (socket_needs_clean_up.second)
            {
                can_shut_down = false;
                break;
            }
        }
    } while (!can_shut_down);

    zmq_sleep(1);

    context.shutdown();

    if (multiverse_server_thread.joinable())
    {
        multiverse_server_thread.join();
    }
}