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

#include <algorithm>
#include <pybind11/chrono.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

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

std::map<std::string, size_t> attribute_map_uint8_t = {
    {"rgb_3840_2160", 3840 * 2160 * 3},
    {"rgb_1280_1024", 1280 * 1024 * 3},
    {"rgb_640_480", 640 * 480 * 3},
    {"rgb_128_128", 128 * 128 * 3}};

std::map<std::string, size_t> attribute_map_uint16_t = {
    {"depth_3840_2160", 3840 * 2160},
    {"depth_1280_1024", 1280 * 1024},
    {"depth_640_480", 640 * 480},
    {"depth_128_128", 128 * 128}};

class MultiverseClientPybind final : public MultiverseClient
{
public:
    MultiverseClientPybind()
    {
    }

    ~MultiverseClientPybind()
    {
    }

    inline double get_world_time() const
    {
        return *world_time;
    }

    inline void set_request_meta_data(const pybind11::dict &in_request_meta_data_dict)
    {
        request_meta_data_dict = in_request_meta_data_dict;
        std::map<std::string, std::map<std::string, size_t>> request_buffer_sizes =
            {{"send", {{"double", 0}, {"uint8", 0}, {"uint16", 0}}}, {"receive", {{"double", 0}, {"uint8", 0}, {"uint16", 0}}}};
        compute_request_buffer_sizes(request_buffer_sizes["send"], request_buffer_sizes["receive"]);

        send_buffer.buffer_double.size = request_buffer_sizes["send"]["double"];
        send_buffer.buffer_uint8_t.size = request_buffer_sizes["send"]["uint8"];
        send_buffer.buffer_uint16_t.size = request_buffer_sizes["send"]["uint16"];
        receive_buffer.buffer_double.size = request_buffer_sizes["receive"]["double"];
        receive_buffer.buffer_uint8_t.size = request_buffer_sizes["receive"]["uint8"];
        receive_buffer.buffer_uint16_t.size = request_buffer_sizes["receive"]["uint16"];
    }

    inline pybind11::dict get_response_meta_data()
    {
        return response_meta_data_dict;
    }

    inline void set_send_data(const pybind11::list &in_send_data)
    {
        if (in_send_data.size() != 1 + send_buffer.buffer_double.size + send_buffer.buffer_uint8_t.size + send_buffer.buffer_uint16_t.size)
        {
            printf("[Client %s] The size of in_send_data (%zu) does not match with send_buffer_size (%zu).\n", client_port.c_str(), in_send_data.size(), 1 + send_buffer.buffer_double.size + send_buffer.buffer_uint8_t.size + send_buffer.buffer_uint16_t.size);
        }
        else
        {
            send_data_double.resize(send_buffer.buffer_double.size);
            send_data_uint8_t.resize(send_buffer.buffer_uint8_t.size);
            send_data_uint16_t.resize(send_buffer.buffer_uint16_t.size);

            try
            {
                world_time = new double(in_send_data[0].cast<double>());
                std::transform(in_send_data.begin() + 1, in_send_data.begin() + 1 + send_buffer.buffer_double.size, send_data_double.begin(),
                               [](const pybind11::handle &item)
                               { return item.cast<double>(); });
                std::transform(in_send_data.begin() + 1 + send_buffer.buffer_double.size, in_send_data.begin() + 1 + send_buffer.buffer_double.size + send_buffer.buffer_uint8_t.size, send_data_uint8_t.begin(),
                               [](const pybind11::handle &item)
                               { return item.cast<uint8_t>(); });
                std::transform(in_send_data.begin() + 1 + send_buffer.buffer_double.size + send_buffer.buffer_uint8_t.size, in_send_data.end(), send_data_uint16_t.begin(),
                               [](const pybind11::handle &item)
                               { return item.cast<uint16_t>(); });
            }
            catch (const std::exception &e)
            {
                printf("[Client %s] Error in set_send_data: %s\n", client_port.c_str(), e.what());
                throw std::runtime_error(e.what());
            }
        }
    }

    inline pybind11::list get_receive_data() const
    {
        return pybind11::cast(std::vector<double>({*world_time})) + pybind11::cast(receive_data_double) + pybind11::cast(receive_data_uint8_t) + pybind11::cast(receive_data_uint16_t);
    }

    inline void set_api_callbacks(const std::map<std::string, std::function<void (pybind11::list)>> &in_api_callbacks)
    {
        api_callbacks = in_api_callbacks;
    }

    inline void set_api_callbacks_response(const std::map<std::string, std::function<pybind11::list(pybind11::list)>> &in_api_callbacks_response)
    {
        api_callbacks_response = in_api_callbacks_response;
    }

    inline void set_bind_request_meta_data_callback(const std::function<void ()> &in_bind_request_meta_data_callback)
    {
        bind_request_meta_data_callback = in_bind_request_meta_data_callback;
    }

    inline void set_bind_response_meta_data_callback(const std::function<void ()> &in_bind_response_meta_data_callback)
    {
        bind_response_meta_data_callback = in_bind_response_meta_data_callback;
    }

    inline void set_bind_send_data_callback(const std::function<void ()> &in_bind_send_data_callback)
    {
        bind_send_data_callback = in_bind_send_data_callback;
    }

    inline void set_bind_receive_data_callback(const std::function<void ()> &in_bind_receive_data_callback)
    {
        bind_receive_data_callback = in_bind_receive_data_callback;
    }

    inline void set_init_objects_callback(const std::function<void ()> &in_init_objects_callback)
    {
        init_objects_callback = in_init_objects_callback;
    }

    inline void set_reset_callback(const std::function<void ()> &in_reset_callback)
    {
        reset_callback = in_reset_callback;
    }

private:
    pybind11::dict request_meta_data_dict;

    pybind11::dict response_meta_data_dict;

    std::vector<double> send_data_double;

    std::vector<uint8_t> send_data_uint8_t;

    std::vector<uint16_t> send_data_uint16_t;

    std::vector<double> receive_data_double;

    std::vector<uint8_t> receive_data_uint8_t;

    std::vector<uint16_t> receive_data_uint16_t;

    std::function<void ()> bind_request_meta_data_callback = []() {};

    std::function<void ()> bind_response_meta_data_callback = []() {};

    std::function<void ()> bind_send_data_callback = []() {};

    std::function<void ()> bind_receive_data_callback = []() {};

    std::function<void ()> init_objects_callback = []() {};

    std::function<void ()> reset_callback = [this]() { printf("[Client %s] Resetting the client (will be implemented).\n", client_port.c_str()); };

    std::map<std::string, std::function<void (pybind11::list)>> api_callbacks;

    std::map<std::string, std::function<pybind11::list(pybind11::list)>> api_callbacks_response;

private:
    bool compute_request_and_response_meta_data() override
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

        if (response_meta_data_dict.contains("time"))
        {
            request_meta_data_dict["meta_data"] = response_meta_data_dict["meta_data"];
            request_meta_data_dict["send"] = pybind11::dict();
            request_meta_data_dict["receive"] = pybind11::dict();

            if (response_meta_data_dict.contains("send"))
            {
                for (const auto &send_objects : response_meta_data_dict["send"].cast<pybind11::dict>())
                {
                    request_meta_data_dict["send"][send_objects.first] = pybind11::list();
                    const pybind11::dict attributes = send_objects.second.cast<pybind11::dict>();
                    for (const auto &attribute : attributes)
                    {
                        request_meta_data_dict["send"][send_objects.first].cast<pybind11::list>().append(attribute.first.cast<std::string>());
                    }
                }
            }

            if (response_meta_data_dict.contains("receive"))
            {
                for (const auto &receive_objects : response_meta_data_dict["receive"].cast<pybind11::dict>())
                {
                    request_meta_data_dict["receive"][receive_objects.first] = pybind11::list();
                    const pybind11::dict attributes = receive_objects.second.cast<pybind11::dict>();
                    for (const auto &attribute : attributes)
                    {
                        request_meta_data_dict["receive"][receive_objects.first].cast<pybind11::list>().append(attribute.first.cast<std::string>());
                    }
                }
            }

            return true;
        }

        return false;
    }

    void compute_request_buffer_sizes(std::map<std::string, size_t> &req_send_buffer_size, std::map<std::string, size_t> &req_receive_buffer_size) const override
    {
        std::map<std::string, std::map<std::string, size_t>> request_buffer_sizes =
            {{"send", {{"double", 0}, {"uint8", 0}, {"uint16", 0}}}, {"receive", {{"double", 0}, {"uint8", 0}, {"uint16", 0}}}};

        for (std::pair<const std::string, std::map<std::string, size_t>> &request_buffer_size : request_buffer_sizes)
        {
            if (!request_meta_data_dict.contains(request_buffer_size.first.c_str()))
            {
                continue;
            }

            for (const auto &send_objects : request_meta_data_dict[request_buffer_size.first.c_str()].cast<pybind11::dict>())
            {
                const std::string object_name = send_objects.first.cast<std::string>();
                if (object_name.compare("") == 0 || (int)request_buffer_size.second["double"] == -1 || (int)request_buffer_size.second["uint8"] == -1 || (int)request_buffer_size.second["uint16"] == -1)
                {
                    request_buffer_size.second["double"] = -1;
                    request_buffer_size.second["uint8"] = -1;
                    request_buffer_size.second["uint16"] = -1;
                    break;
                }

                const pybind11::list attributes = send_objects.second.cast<pybind11::list>();
                for (size_t i = 0; i < pybind11::len(attributes); i++)
                {
                    if (attributes[i].cast<std::string>().compare("") == 0)
                    {
                        request_buffer_size.second["double"] = -1;
                        request_buffer_size.second["uint8"] = -1;
                        request_buffer_size.second["uint16"] = -1;
                        break;
                    }
                    if (attribute_map_double.find(attributes[i].cast<std::string>()) != attribute_map_double.end())
                    {
                        request_buffer_size.second["double"] += attribute_map_double[attributes[i].cast<std::string>()];
                    }
                    if (attribute_map_uint8_t.find(attributes[i].cast<std::string>()) != attribute_map_uint8_t.end())
                    {
                        request_buffer_size.second["uint8"] += attribute_map_uint8_t[attributes[i].cast<std::string>()];
                    }
                    if (attribute_map_uint16_t.find(attributes[i].cast<std::string>()) != attribute_map_uint16_t.end())
                    {
                        request_buffer_size.second["uint16"] += attribute_map_uint16_t[attributes[i].cast<std::string>()];
                    }
                }
            }
        }

        req_send_buffer_size = request_buffer_sizes["send"];
        req_receive_buffer_size = request_buffer_sizes["receive"];
    }

    void compute_response_buffer_sizes(std::map<std::string, size_t> &res_send_buffer_size, std::map<std::string, size_t> &res_receive_buffer_size) const override
    {
        std::map<std::string, std::map<std::string, size_t>> response_buffer_sizes =
            {{"send", {{"double", 0}, {"uint8", 0}, {"uint16", 0}}}, {"receive", {{"double", 0}, {"uint8", 0}, {"uint16", 0}}}};

        for (std::pair<const std::string, std::map<std::string, size_t>> &response_buffer_size : response_buffer_sizes)
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
                    if (attribute_map_double.find(attribute.first.cast<std::string>()) != attribute_map_double.end())
                    {
                        response_buffer_size.second["double"] += attribute.second.cast<pybind11::list>().size();
                    }
                    if (attribute_map_uint8_t.find(attribute.first.cast<std::string>()) != attribute_map_uint8_t.end())
                    {
                        response_buffer_size.second["uint8"] += attribute.second.cast<pybind11::list>().size();
                    }
                    if (attribute_map_uint16_t.find(attribute.first.cast<std::string>()) != attribute_map_uint16_t.end())
                    {
                        response_buffer_size.second["uint16"] += attribute.second.cast<pybind11::list>().size();
                    }
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

    bool init_objects(bool from_request_meta_data = false) override
    {
        if (from_request_meta_data)
        {
            bind_request_meta_data();
        }
        init_objects_callback();
        return true;
    }

    void bind_request_meta_data() override
    {
        bind_request_meta_data_callback();
        request_meta_data_str = pybind11::str(request_meta_data_dict).cast<std::string>();
        std::replace(request_meta_data_str.begin(), request_meta_data_str.end(), '\'', '"');
    }

    void bind_response_meta_data() override
    {
        bind_response_meta_data_callback();
    }

    void bind_api_callbacks() override
    {
        if (!response_meta_data_dict.contains("api_callbacks"))
        {
            return;
        }
        pybind11::list api_callbacks_list = response_meta_data_dict["api_callbacks"].cast<pybind11::list>();
        for (size_t i = 0; i < pybind11::len(api_callbacks_list); i++)
        {
            const pybind11::dict api_callback_dict = api_callbacks_list[i].cast<pybind11::dict>();
            for (auto api_callback_pair : api_callback_dict)
            {
                const std::string api_callback_name = api_callback_pair.first.cast<std::string>();
                if (api_callbacks.find(api_callback_name) == api_callbacks.end())
                {
                    continue;
                }
                const pybind11::list api_callback_arguments = api_callback_pair.second.cast<pybind11::list>();
                api_callbacks[api_callback_name.c_str()](api_callback_arguments);
            }
        }
    }

    void bind_api_callbacks_response() override
    {
        if (!response_meta_data_dict.contains("api_callbacks"))
        {
            return;
        }
        request_meta_data_dict["api_callbacks_response"] = pybind11::list();
        pybind11::list api_callbacks_list = response_meta_data_dict["api_callbacks"].cast<pybind11::list>();
        for (size_t i = 0; i < pybind11::len(api_callbacks_list); i++)
        {
            const pybind11::dict api_callback_dict = api_callbacks_list[i].cast<pybind11::dict>();
            for (auto api_callback_pair : api_callback_dict)
            {
                const std::string api_callback_name = api_callback_pair.first.cast<std::string>();
                pybind11::dict api_callback_dict_request;
                if (api_callbacks_response.find(api_callback_name) != api_callbacks_response.end())
                {
                    const pybind11::list api_callback_arguments = api_callback_pair.second.cast<pybind11::list>();
                    api_callback_dict_request[api_callback_name.c_str()] = api_callbacks_response[api_callback_name.c_str()](api_callback_arguments);
                }
                else
                {
                    api_callback_dict_request[api_callback_name.c_str()] = pybind11::list();
                    api_callback_dict_request[api_callback_name.c_str()].cast<pybind11::list>().append("not implemented");
                }
                request_meta_data_dict["api_callbacks_response"].cast<pybind11::list>().append(api_callback_dict_request);
            }
        }
    }

    void clean_up() override
    {
        // TODO: Find a clean way to clear the data because it's unsure if the data is still in use.

        // send_data.clear();

        // receive_data.clear();
    }

    void reset() override
    {
        reset_callback();
    }

    void init_send_and_receive_data() override
    {
        if (send_buffer.buffer_double.size != send_data_double.size())
        {
            send_data_double = std::vector<double>(send_buffer.buffer_double.size, 0.0);
        }
        if (send_buffer.buffer_uint8_t.size != send_data_uint8_t.size())
        {
            send_data_uint8_t = std::vector<uint8_t>(send_buffer.buffer_uint8_t.size, 0);
        }
        if (send_buffer.buffer_uint16_t.size != send_data_uint16_t.size())
        {
            send_data_uint16_t = std::vector<uint16_t>(send_buffer.buffer_uint16_t.size, 0);
        }
        if (receive_buffer.buffer_double.size != receive_data_double.size())
        {
            receive_data_double = std::vector<double>(receive_buffer.buffer_double.size, 0.0);
        }
        if (receive_buffer.buffer_uint8_t.size != receive_data_uint8_t.size())
        {
            receive_data_uint8_t = std::vector<uint8_t>(receive_buffer.buffer_uint8_t.size, 0);
        }
        if (receive_buffer.buffer_uint16_t.size != receive_data_uint16_t.size())
        {
            receive_data_uint16_t = std::vector<uint16_t>(receive_buffer.buffer_uint16_t.size, 0);
        }
    }

    void bind_send_data() override
    {
        bind_send_data_callback();
        if (send_data_double.size() != send_buffer.buffer_double.size || send_data_uint8_t.size() != send_buffer.buffer_uint8_t.size)
        {
            printf("[Client %s] The size of in_send_data [%zu - %zu - %zu] does not match with send_buffer_size [%zu - %zu - %zu].\n",
                   client_port.c_str(),
                   send_data_double.size(),
                   send_data_uint8_t.size(),
                   send_data_uint16_t.size(),
                   send_buffer.buffer_double.size,
                   send_buffer.buffer_uint8_t.size,
                   send_buffer.buffer_uint16_t.size);
            return;
        }

        std::copy(send_data_double.begin(), send_data_double.end(), send_buffer.buffer_double.data);
        std::copy(send_data_uint8_t.begin(), send_data_uint8_t.end(), send_buffer.buffer_uint8_t.data);
        std::copy(send_data_uint16_t.begin(), send_data_uint16_t.end(), send_buffer.buffer_uint16_t.data);
    }

    void bind_receive_data() override
    {
        if (receive_data_double.size() != receive_buffer.buffer_double.size ||
            receive_data_uint8_t.size() != receive_buffer.buffer_uint8_t.size ||
            receive_data_uint16_t.size() != receive_buffer.buffer_uint16_t.size)
        {
            printf("[Client %s] The size of receive_data [%zu - %zu - %zu] does not match with receive_buffer_size [%zu - %zu - %zu].\n",
                   client_port.c_str(),
                   receive_data_double.size(),
                   receive_data_uint8_t.size(),
                   receive_data_uint16_t.size(),
                   receive_buffer.buffer_double.size,
                   receive_buffer.buffer_uint8_t.size,
                   receive_buffer.buffer_uint16_t.size);
            return;
        }

        std::copy(receive_buffer.buffer_double.data, receive_buffer.buffer_double.data + receive_buffer.buffer_double.size, receive_data_double.begin());
        std::copy(receive_buffer.buffer_uint8_t.data, receive_buffer.buffer_uint8_t.data + receive_buffer.buffer_uint8_t.size, receive_data_uint8_t.begin());
        std::copy(receive_buffer.buffer_uint16_t.data, receive_buffer.buffer_uint16_t.data + receive_buffer.buffer_uint16_t.size, receive_data_uint16_t.begin());
        bind_receive_data_callback();
    }
};

PYBIND11_MODULE(multiverse_client_pybind, handle)
{
    handle.doc() = "";

    pybind11::class_<MultiverseClient>(handle, "MultiverseClient")
        .def("connect", static_cast<void (MultiverseClient::*)(const std::string &, const std::string &, const std::string &)>(&MultiverseClient::connect))
        .def("start", &MultiverseClient::start)
        .def("communicate", &MultiverseClient::communicate)
        .def("disconnect", &MultiverseClient::disconnect)
        .def("get_time_now", &MultiverseClient::get_time_now);

    pybind11::class_<MultiverseClientPybind, MultiverseClient>(handle, "MultiverseClientPybind")
        .def(pybind11::init<>())
        .def("get_world_time", &MultiverseClientPybind::get_world_time)
        .def("set_request_meta_data", &MultiverseClientPybind::set_request_meta_data)
        .def("get_response_meta_data", &MultiverseClientPybind::get_response_meta_data)
        .def("set_send_data", &MultiverseClientPybind::set_send_data)
        .def("get_receive_data", &MultiverseClientPybind::get_receive_data)
        .def("set_api_callbacks", &MultiverseClientPybind::set_api_callbacks)
        .def("set_api_callbacks_response", &MultiverseClientPybind::set_api_callbacks_response)
        .def("set_bind_request_meta_data_callback", &MultiverseClientPybind::set_bind_request_meta_data_callback)
        .def("set_bind_response_meta_data_callback", &MultiverseClientPybind::set_bind_response_meta_data_callback)
        .def("set_bind_send_data_callback", &MultiverseClientPybind::set_bind_send_data_callback)
        .def("set_bind_receive_data_callback", &MultiverseClientPybind::set_bind_receive_data_callback)
        .def("set_init_objects_callback", &MultiverseClientPybind::set_init_objects_callback)
        .def("set_reset_callback", &MultiverseClientPybind::set_reset_callback);
}