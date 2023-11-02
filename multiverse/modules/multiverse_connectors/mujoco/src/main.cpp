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

#ifdef VISUAL
#include "mj_visual.h"
#endif
#include "mj_multiverse_client.h"
#include "mj_simulate.h"
#ifdef __linux__
#include <jsoncpp/json/reader.h>
#elif _WIN32
#include <json/reader.h>
#endif
#include <thread>
#include <csignal>

static MjSimulate &mj_simulate = MjSimulate::get_instance();
#ifdef VISUAL
static MjVisual &mj_visual = MjVisual::get_instance();
#endif
static MjMultiverseClient &mj_multiverse_client = MjMultiverseClient::get_instance();

// Signal handler function
void signal_handler(int signum) {
    printf("Interrupt signal (%d) received.\n", signum);
    
    stop = true;

    // Exit program
    exit(signum);
}

int main(int argc, char **argv)
{
    // Register signal handler for SIGINT
    signal(SIGINT, signal_handler);

    // print version, check compatibility
    printf("MuJoCo version %s\n", mj_versionString());

    if (argc < 2)
    {
        mju_error("USAGE:  mujoco mjcf.xml\n");
    }
    scene_xml_path = argv[1];
    
    mj_simulate.init();
#ifdef VISUAL
    mj_visual.init();
#endif

    if (argc > 2)
    {
        const std::string multiverse_params_str = argv[2];
        Json::Value multiverse_params_json;
        Json::Reader reader;
        if (reader.parse(multiverse_params_str, multiverse_params_json) && !multiverse_params_json.empty())
        {
            const Json::Value multiverse_server_params_json = multiverse_params_json["multiverse_server"];
            std::string server_host = multiverse_server_params_json["host"].toStyledString();
            server_host.erase(std::remove(server_host.begin(), server_host.end(), '"'), server_host.end());
            server_host.erase(std::remove(server_host.begin(), server_host.end(), '\n'), server_host.end());
            std::string server_port = multiverse_server_params_json["port"].toStyledString();
            server_port.erase(std::remove(server_port.begin(), server_port.end(), '"'), server_port.end());
            server_port.erase(std::remove(server_port.begin(), server_port.end(), '\n'), server_port.end());

            const Json::Value multiverse_client_params_json = multiverse_params_json["multiverse_client"];
            std::string client_port = multiverse_client_params_json["port"].toStyledString();
            client_port.erase(std::remove(client_port.begin(), client_port.end(), '"'), client_port.end());
            client_port.erase(std::remove(client_port.begin(), client_port.end(), '\n'), client_port.end());
            std::string world = multiverse_client_params_json["world"].toStyledString();
            world.erase(std::remove(world.begin(), world.end(), '"'), world.end());
            world.erase(std::remove(world.begin(), world.end(), '\n'), world.end());

            mj_multiverse_client.init(server_host, server_port, client_port, multiverse_client_params_json["send"], multiverse_client_params_json["receive"], world);
        }
    }

    std::thread sim_thread([](){
        while (!stop)
        {
            mj_multiverse_client.communicate();
            mj_simulate.step();
        }
    });
#ifdef VISUAL
    mj_visual.run();
#endif
    
    sim_thread.join();

    mj_multiverse_client.disconnect();

    return 0;
}