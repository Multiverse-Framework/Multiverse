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

#include "mj_model.h"

#include "multiverse_client_json.h"
#include <set>
#include <thread>

class MjMultiverseClient final : public MultiverseClientJson
{
public:
    MjMultiverseClient(const MjMultiverseClient &) = delete;

    void operator=(MjMultiverseClient const &) = delete;

    static MjMultiverseClient &get_instance()
    {
        static MjMultiverseClient mj_multiverse_client;
        return mj_multiverse_client;
    }

public:
    void init(const Json::Value &multiverse_params_json);

private:
    bool spawn_objects(std::set<std::string> &objects);

    bool destroy_objects(std::set<std::string> &objects);

    void weld(const Json::Value &arguments);

    std::string get_weld_response(const Json::Value &arguments) const;

    void unweld(const Json::Value &arguments);

    std::string get_unweld_response(const Json::Value &arguments) const;

    void attach(const Json::Value &arguments);

    std::string get_attach_response(const Json::Value &arguments) const;

    void detach(const Json::Value &arguments);

    std::string get_detach_response(const Json::Value &arguments) const;

    std::set<std::string> get_get_contact_bodies_response(const Json::Value &arguments) const;

    std::set<std::string> get_get_contact_islands_response(const Json::Value &arguments) const;

    std::string get_get_constraint_effort_response(const Json::Value &arguments) const;

    std::vector<std::string> get_get_rays_response(const Json::Value &arguments) const;

public:
    void communicate(const bool resend_meta_data = false) override;

public:
    std::map<std::string, std::set<std::string>> send_objects;

    std::map<std::string, std::set<std::string>> receive_objects;

    static std::mutex mutex;

private:
    std::string world_name;

    std::string simulation_name;

    double world_time;

    Json::Value send_objects_json;

    Json::Value receive_objects_json;

    std::thread connect_to_server_thread;

    std::thread meta_data_thread;

    std::vector<mjtNum *> send_data_vec;

    std::vector<mjtNum *> receive_data_vec;

    std::map<int, mjtNum *> relative_velocities;

    std::map<int, mjtNum *> odom_velocities;

    std::map<int, mjtNum *> contact_efforts;

    std::set<std::string> resources;

private:
    void start_connect_to_server_thread() override;

    void wait_for_connect_to_server_thread_finish() override;

    void start_meta_data_thread() override;

    void wait_for_meta_data_thread_finish() override;

    bool init_objects(bool from_request_meta_data = false) override;

    void bind_request_meta_data() override;

    void bind_response_meta_data() override;    

    void bind_api_callbacks() override;

    void bind_api_callbacks_response() override;

    void init_send_and_receive_data() override;

    void bind_send_data() override;

    void bind_receive_data() override;

    void clean_up() override;

    void reset() override;

private:
    MjMultiverseClient()
    {

    }

    ~MjMultiverseClient()
    {

    }
};