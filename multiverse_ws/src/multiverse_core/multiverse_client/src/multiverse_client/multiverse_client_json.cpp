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

#include "multiverse_client_json.h"
#include <algorithm>

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


bool MultiverseClientJson::compute_receive_meta_data()
{
    return !receive_meta_data_str.empty() &&
           reader.parse(receive_meta_data_str, receive_meta_data_json) &&
           receive_meta_data_json.isMember("time") &&
           receive_meta_data_json["time"].asDouble() > 0;
}

void MultiverseClientJson::compute_request_buffer_sizes(size_t &send_buffer_size, size_t &receive_buffer_size) const
{
    std::map<std::string, size_t> request_buffer_sizes = {{"send", 1}, {"receive", 1}};
    for (std::pair<const std::string, size_t> &request_buffer_size : request_buffer_sizes)
    {
        for (const std::string &object_name : send_meta_data_json[request_buffer_size.first].getMemberNames())
        {
            if (object_name.compare("") == 0 || request_buffer_size.second == -1)
            {
                request_buffer_size.second = -1;
                break;
            }
            for (const Json::Value &attribute : send_meta_data_json[request_buffer_size.first][object_name])
            {
                if (attribute.asString().compare("") == 0)
                {
                    request_buffer_size.second = -1;
                    break;
                }
                request_buffer_size.second += attribute_map[attribute.asString()];
            }
        }
    }

    send_buffer_size = request_buffer_sizes["send"];
    receive_buffer_size = request_buffer_sizes["receive"];
}

void MultiverseClientJson::compute_response_buffer_sizes(size_t &send_buffer_size, size_t &receive_buffer_size) const
{
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

    send_buffer_size = response_buffer_sizes["send"];
    receive_buffer_size = response_buffer_sizes["receive"];
}