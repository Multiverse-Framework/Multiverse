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

#include "multiverse_client.h"

#include <pybind11/chrono.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

std::map<std::string, size_t> attribute_map = {
    {"", 0},
    {"position", 3},
    {"quaternion", 4},
    {"relative_velocity", 6},
    {"joint_rvalue", 1},
    {"joint_tvalue", 1},
    {"joint_linear_velocity", 1},
    {"joint_angular_velocity", 1},
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

class MultiverseClientPybind final : public MultiverseClient
{
public:
    MultiverseClientPybind(const std::string &in_server_socket_addr = "tcp:127.0.0.1:7000")
    {
        server_socket_addr = in_server_socket_addr;
    }

    ~MultiverseClientPybind()
    {
    }

    inline void set_request_meta_data(const pybind11::dict &in_request_meta_data_dict)
    {
        request_meta_data_dict = in_request_meta_data_dict;
    }

    inline pybind11::dict get_response_meta_data()
    {
        return response_meta_data_dict;
    }

    inline void set_send_data(const pybind11::list &in_send_data)
    {
        if (in_send_data.size() != send_buffer_size)
        {
            printf("[Client %s] The size of in_send_data (%ld) does not match with send_buffer_size (%ld).", port.c_str(), in_send_data.size(), send_buffer_size);
        }
        else
        {
            send_data = in_send_data;
        }
    }

    inline pybind11::list get_receive_data() const
    {
        return receive_data;
    }

private:
    pybind11::dict request_meta_data_dict;

    pybind11::dict response_meta_data_dict;

    pybind11::list send_data;

    pybind11::list receive_data;

private:
    bool compute_response_meta_data() override
    {
        if (response_meta_data_str.empty())
        {
            response_meta_data_dict = pybind11::dict();
            return false;
        }

        pybind11::module json_module = pybind11::module::import("json");

        pybind11::object json_loads = json_module.attr("loads");

        pybind11::object parsed_dict = json_loads(response_meta_data_str);

        response_meta_data_dict = parsed_dict.cast<pybind11::dict>();
        
        return response_meta_data_dict.contains("time");
    }

    void compute_request_buffer_sizes(size_t &req_send_buffer_size, size_t &req_receive_buffer_size) const override
    {
        std::map<std::string, size_t> request_buffer_sizes = {{"send", 1}, {"receive", 1}};

        for (std::pair<const std::string, size_t> &request_buffer_size : request_buffer_sizes)
        {
            if (!request_meta_data_dict.contains(request_buffer_size.first.c_str()))
            {
                continue;
            }

            for (const auto &send_objects : request_meta_data_dict[request_buffer_size.first.c_str()].cast<pybind11::dict>())
            {
                const std::string object_name = send_objects.first.cast<std::string>();
                if (object_name.compare("") == 0 || request_buffer_size.second == -1)
                {
                    request_buffer_size.second = -1;
                    break;
                }

                const pybind11::list attributes = send_objects.second.cast<pybind11::list>();
                for (size_t i = 0; i < pybind11::len(attributes); i++)
                {
                    if (attributes[i].cast<std::string>().compare("") == 0)
                    {
                        request_buffer_size.second = -1;
                        break;
                    }
                    request_buffer_size.second += attribute_map[attributes[i].cast<std::string>()];
                }
            }
        }

        req_send_buffer_size = request_buffer_sizes["send"];
        req_receive_buffer_size = request_buffer_sizes["receive"];
    }

    void compute_response_buffer_sizes(size_t &res_send_buffer_size, size_t &res_receive_buffer_size) const override
    {
        std::map<std::string, size_t> response_buffer_sizes = {{"send", 1}, {"receive", 1}};

        for (std::pair<const std::string, size_t> &response_buffer_size : response_buffer_sizes)
        {
            if (!response_meta_data_dict.contains(response_buffer_size.first.c_str()))
            {
                continue;
            }

            for (const auto &receive_objects : response_meta_data_dict[response_buffer_size.first.c_str()].cast<pybind11::dict>())
            {
                const pybind11::dict attributes = receive_objects.second.cast<pybind11::dict>();
                for (const auto &attribute : attributes)
                {
                    response_buffer_size.second += attribute.second.cast<pybind11::list>().size();
                }
            }
        }

        res_send_buffer_size = response_buffer_sizes["send"];
        res_receive_buffer_size = response_buffer_sizes["receive"];
    }

    void start_connect_to_server_thread() override
    {
        MultiverseClientPybind::connect_to_server();
    }

    void wait_for_connect_to_server_thread_finish() override
    {
        
    }

    void start_meta_data_thread() override
    {
        MultiverseClientPybind::send_and_receive_meta_data();
    }

    void wait_for_meta_data_thread_finish() override
    {
        
    }

    bool init_objects(bool from_server = false) override
    {
        return true;
    }

    void bind_request_meta_data() override
    {
        request_meta_data_str = pybind11::str(request_meta_data_dict).cast<std::string>();
        std::replace(request_meta_data_str.begin(), request_meta_data_str.end(), '\'', '"');
    }

    void bind_response_meta_data() override
    {
    }

    void clean_up() override
    {
        // send_data = pybind11::list();

        // receive_data = pybind11::list();
    }

    void reset() override
    {
        
    }

    void init_send_and_receive_data() override
    {
        if (send_buffer_size != send_data.size())
        {
            send_data = pybind11::cast(std::vector<double>(send_buffer_size, 0.0));
        }
        if (receive_buffer_size != receive_data.size())
        {
            receive_data = pybind11::cast(std::vector<double>(receive_buffer_size, 0.0));
        }
    }

    void bind_send_data() override
    {
        if (send_buffer_size != send_data.size())
        {
            printf("[Client %s] The size of in_send_data (%ld) does not match with send_buffer_size (%ld).", port.c_str(), send_data.size(), send_buffer_size);
            return;
        }
        
        send_buffer[0] = std::numeric_limits<double>::quiet_NaN();
        for (size_t i = 1; i < send_buffer_size; i++)
        {
            send_buffer[i] = send_data[i].cast<double>();
        }
    }

    void bind_receive_data() override
    {
        for (size_t i = 0; i < receive_buffer_size; i++)
        {
            receive_data[i] = receive_buffer[i];
        }
    }
};

PYBIND11_MODULE(multiverse_client_pybind, handle)
{
    handle.doc() = "";

    pybind11::class_<MultiverseClient>(handle, "MultiverseClient")
        .def("connect", static_cast<void (MultiverseClient::*)(const std::string &, const std::string &)>(&MultiverseClient::connect))
        .def("start", &MultiverseClient::start)
        .def("communicate", &MultiverseClient::communicate)
        .def("disconnect", &MultiverseClient::disconnect);

    pybind11::class_<MultiverseClientPybind, MultiverseClient>(handle, "MultiverseClientPybind")
        .def(pybind11::init<const std::string &>())
        .def("set_request_meta_data", &MultiverseClientPybind::set_request_meta_data)
        .def("get_response_meta_data", &MultiverseClientPybind::get_response_meta_data)
        .def("set_send_data", &MultiverseClientPybind::set_send_data)
        .def("get_receive_data", &MultiverseClientPybind::get_receive_data);
}