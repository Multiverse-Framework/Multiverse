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

#include "multiverse_client_json.h"

#include <algorithm>

std::map<std::string, size_t> attribute_map_double = {
    {"", 0},
    {"time", 1},
    {"scalar", 1},
    {"position", 3},
    {"quaternion", 4},
    {"relative_velocity", 6},
    {"odometric_velocity", 6},
    {"joint_rvalue", 1},
    {"joint_tvalue", 1},
    {"joint_linear_velocity", 1},
    {"joint_angular_velocity", 1},
    {"joint_linear_acceleration", 1},
    {"joint_angular_acceleration", 1},
    {"joint_force", 1},
    {"joint_torque", 1},
    {"cmd_joint_rvalue", 1},
    {"cmd_joint_tvalue", 1},
    {"cmd_joint_linear_velocity", 1},
    {"cmd_joint_angular_velocity", 1},
    {"cmd_joint_force", 1},
    {"cmd_joint_torque", 1},
    {"joint_position", 3},
    {"joint_quaternion", 4},
    {"force", 3},
    {"torque", 3}};

bool MultiverseClientJson::compute_request_and_response_meta_data()
{
    if (!response_meta_data_str.empty() &&
        reader.parse(response_meta_data_str, response_meta_data_json) &&
        response_meta_data_json.isMember("time") &&
        response_meta_data_json["time"].asDouble() >= 0)
    {
        request_meta_data_json["meta_data"] = response_meta_data_json["meta_data"];
        request_meta_data_json["send"].clear();
        request_meta_data_json["receive"].clear();

        if (response_meta_data_json.isMember("send"))
        {
            for (const std::string &object_name : response_meta_data_json["send"].getMemberNames())
            {
                request_meta_data_json["send"][object_name] = Json::arrayValue;
                for (const std::string &attribute_name : response_meta_data_json["send"][object_name].getMemberNames())
                {
                    request_meta_data_json["send"][object_name].append(attribute_name);
                }
            }
        }
        if (response_meta_data_json.isMember("receive"))
        {
            for (const std::string &object_name : response_meta_data_json["receive"].getMemberNames())
            {
                request_meta_data_json["receive"][object_name] = Json::arrayValue;
                for (const std::string &attribute_name : response_meta_data_json["receive"][object_name].getMemberNames())
                {
                    request_meta_data_json["receive"][object_name].append(attribute_name);
                }
            }
        }
        return true;
    }
    return false;
}

void MultiverseClientJson::compute_request_buffer_sizes(std::map<std::string, size_t> &send_buffer_size, std::map<std::string, size_t> &receive_buffer_size) const
{
    std::map<std::string, std::map<std::string, size_t>> request_buffer_sizes =
        {{"send", {{"double", 0}, {"uint8", 0}, {"uint16", 0}}}, {"receive", {{"double", 0}, {"uint8", 0}, {"uint16", 0}}}};
    for (std::pair<const std::string, std::map<std::string, size_t>> &request_buffer_size : request_buffer_sizes)
    {
        for (const std::string &object_name : request_meta_data_json[request_buffer_size.first].getMemberNames())
        {
            if (object_name.compare("") == 0 || request_buffer_size.second["double"] == -1 || request_buffer_size.second["uint8"] == -1 || request_buffer_size.second["uint16"] == -1)
            {
                request_buffer_size.second["double"] = -1;
                request_buffer_size.second["uint8"] = -1;
                request_buffer_size.second["uint16"] = -1;
                break;
            }
            for (const Json::Value &attribute : request_meta_data_json[request_buffer_size.first][object_name])
            {
                if (attribute.asString().compare("") == 0)
                {
                    request_buffer_size.second["double"] = -1;
                    request_buffer_size.second["uint8"] = -1;
                    request_buffer_size.second["uint16"] = -1;
                    break;
                }
                if (attribute_map_double.find(attribute.asString()) != attribute_map_double.end())
                {
                    request_buffer_size.second["double"] += attribute_map_double[attribute.asString()];
                }
            }
        }
    }

    send_buffer_size = request_buffer_sizes["send"];
    receive_buffer_size = request_buffer_sizes["receive"];
}

void MultiverseClientJson::compute_response_buffer_sizes(std::map<std::string, size_t> &send_buffer_size, std::map<std::string, size_t> &receive_buffer_size) const
{
    std::map<std::string, std::map<std::string, size_t>> response_buffer_sizes =
        {{"send", {{"double", 0}, {"uint8", 0}, {"uint16", 0}}}, {"receive", {{"double", 0}, {"uint8", 0}, {"uint16", 0}}}};
    for (std::pair<const std::string, std::map<std::string, size_t>> &response_buffer_size : response_buffer_sizes)
    {
        for (const std::string &object_name : response_meta_data_json[response_buffer_size.first].getMemberNames())
        {
            for (const std::string &attribute_name : response_meta_data_json[response_buffer_size.first][object_name].getMemberNames())
            {
                if (attribute_map_double.find(attribute_name) != attribute_map_double.end())
                {
                    response_buffer_size.second["double"] += response_meta_data_json[response_buffer_size.first][object_name][attribute_name].size();
                }
            }
        }
    }

    send_buffer_size = response_buffer_sizes["send"];
    receive_buffer_size = response_buffer_sizes["receive"];
}