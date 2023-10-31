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
    void init(const std::string &server_host, const std::string &server_port, const std::string &client_port, const Json::Value &in_send_objects_json, const Json::Value &in_receive_objects_json, const std::string &in_world);

public:
    void communicate(const bool resend_meta_data = false) override;

public:
    std::map<std::string, std::set<std::string>> send_objects;

    std::map<std::string, std::set<std::string>> receive_objects;

    static std::mutex mutex;

private:
    std::string world;

    Json::Value send_objects_json;

    Json::Value receive_objects_json;

    std::thread connect_to_server_thread;

    std::thread meta_data_thread;

    std::vector<mjtNum *> send_data_vec;

    std::vector<mjtNum *> receive_data_vec;

    std::map<int, mjtNum *> odom_velocities;

    std::map<int, mjtNum *> contact_efforts;

private:
    void start_connect_to_server_thread() override;

    void wait_for_connect_to_server_thread_finish() override;

    void start_meta_data_thread() override;

    void wait_for_meta_data_thread_finish() override;

    bool init_objects() override;

    void bind_request_meta_data() override;

    void bind_response_meta_data() override;    

    void init_send_and_receive_data() override;

    void bind_send_data() override;

    void bind_receive_data() override;

    void clean_up() override;

private:
    MjMultiverseClient()
    {

    }

    ~MjMultiverseClient()
    {

    }
};