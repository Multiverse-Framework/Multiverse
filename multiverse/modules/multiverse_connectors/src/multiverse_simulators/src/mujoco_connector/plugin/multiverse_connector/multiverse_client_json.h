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

#include "multiverse_client.h"
#ifdef __linux__
#include <jsoncpp/json/json.h>
#elif _WIN32
#include <json/json.h>
#endif

class MultiverseClientJson : public MultiverseClient
{
protected:
    bool compute_request_and_response_meta_data() override final;

    void compute_request_buffer_sizes(std::map<std::string, size_t> &send_buffer_size, std::map<std::string, size_t> &receive_buffer_size) const override final;

    void compute_response_buffer_sizes(std::map<std::string, size_t> &send_buffer_size, std::map<std::string, size_t> &receive_buffer_size) const override final;

protected:
    Json::Value request_meta_data_json;

    Json::Value response_meta_data_json;

    Json::Reader reader;
};