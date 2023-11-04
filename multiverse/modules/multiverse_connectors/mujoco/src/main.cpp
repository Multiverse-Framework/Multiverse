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

double get_time_now()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000000.0;
}

void simulate(const double rtf_desired, const double max_time_step, const double min_time_step)
{
    start_time = get_time_now();
    double last_real_time = get_time_now() - start_time;
    int i = 0;
    while (!stop)
    {
        mj_multiverse_client.communicate();
        mj_simulate.step();

        // Calculate real time factor
        int num_step = mju_ceil(1 / m->opt.timestep);
        static std::deque<double> last_sim_times;
        static std::deque<double> last_real_times;
        double error_time;
        if (i == 0)
        {
            last_sim_times.clear();
            last_real_times.clear();
        }
        do
        {
            real_time = get_time_now() - start_time;
            error_time = real_time - d->time / rtf_desired;
        } while (error_time < -1E-6 && i != 0);
        
        last_sim_times.push_front(d->time);
        last_real_times.push_front(real_time);
        if (i == num_step)
        {
            last_sim_times.pop_back();
            last_real_times.pop_back();
        }
        else
        {
            i++;
        }
        rtf = (real_time - last_real_times.back()) / (d->time - last_sim_times.back());

        // Change timestep when out of sync
        if (error_time > 1E-3)
        {
            if (m->opt.timestep < max_time_step)
            {
                m->opt.timestep *= 2;
            }
        }
        else
        {
            if (m->opt.timestep > min_time_step)
            {
                m->opt.timestep /= 2;
            }
        }
    }
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
            mj_multiverse_client.init(multiverse_params_json);
        }
    }

    std::map<std::string, double> config_params_map = {
        {"rtf_desired", 1.0},
        {"max_time_step", 0.01},
        {"min_time_step", 0.001}
    };
    
    if (argc > 3)
    {
        const std::string config_params_str = argv[3];
        Json::Value config_params_json;
        Json::Reader reader;
        if (reader.parse(config_params_str, config_params_json) && !config_params_json.empty())
        {
            for (std::pair<const std::string, double> &config_param : config_params_map)
            {
                if (config_params_json[config_param.first].isDouble())
                {
                    config_param.second = config_params_json[config_param.first].asDouble();
                }
            }
        }
    }

    std::thread sim_thread(&simulate, config_params_map["rtf_desired"], config_params_map["max_time_step"], config_params_map["min_time_step"]);
#ifdef VISUAL
    mj_visual.run();
#endif
    
    sim_thread.join();

    mj_multiverse_client.disconnect();

    return 0;
}