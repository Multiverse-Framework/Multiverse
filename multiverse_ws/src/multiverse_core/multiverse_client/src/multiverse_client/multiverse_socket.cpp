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

#include "multiverse_client.h"

#include <thread>

#include <pybind11/chrono.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

class MultiverseSocket final : public MultiverseClient
{
public:
    MultiverseSocket(const bool use_thread) : use_thread(use_thread)
    {
    }

    ~MultiverseSocket()
    {
    }

    inline void set_send_meta_data(const pybind11::dict &in_send_meta_data_dict)
    {
        send_meta_data_dict = in_send_meta_data_dict;
    }

    inline pybind11::dict get_receive_meta_data() const
    {
        return receive_meta_data_dict;
    }

    inline void set_send_data(const pybind11::list &in_send_data)
    {
        if (in_send_data.size() != send_buffer_size)
        {
            printf("[Client %s] Size doesn't match with send_buffer_size = %ld", port.c_str(), send_buffer_size);
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
    pybind11::dict send_meta_data_dict;

    pybind11::dict receive_meta_data_dict;

    pybind11::list send_data;

    pybind11::list receive_data;

    std::thread meta_data_thread;

    bool use_thread = true;

private:
    void start_meta_data_thread() override
    {
        if (use_thread)
        {
            meta_data_thread = std::thread(&MultiverseClient::send_and_receive_meta_data, this);
        }
        else
        {
            MultiverseClient::send_and_receive_meta_data();
        }
    }

    void wait_for_meta_data_thread_finish() override
    {
        if (use_thread && meta_data_thread.joinable())
        {
            meta_data_thread.join();
        }
    }

    bool init_objects() override
    {
        return true;
    }

    void bind_send_meta_data() override
    {
        send_meta_data_json.clear();
        
        send_meta_data_json["world"] = send_meta_data_dict.contains("world") ? send_meta_data_dict["world"].cast<std::string>() : "world";
        send_meta_data_json["length_unit"] = send_meta_data_dict.contains("length_unit") ? send_meta_data_dict["length_unit"].cast<std::string>() : "N";
        send_meta_data_json["angle_unit"] = send_meta_data_dict.contains("angle_unit") ? send_meta_data_dict["angle_unit"].cast<std::string>() : "rad";
        send_meta_data_json["force_unit"] = send_meta_data_dict.contains("force_unit") ? send_meta_data_dict["force_unit"].cast<std::string>() : "N";
        send_meta_data_json["time_unit"] = send_meta_data_dict.contains("time_unit") ? send_meta_data_dict["time_unit"].cast<std::string>() : "s";
        send_meta_data_json["handedness"] = send_meta_data_dict.contains("handedness") ? send_meta_data_dict["handedness"].cast<std::string>() : "rhs";

        for (const std::string &send_receive : {"send", "receive"})
        {
            for (const std::pair<std::string, std::vector<std::string>> receive_objects : send_meta_data_dict[send_receive.c_str()].cast<std::map<std::string, std::vector<std::string>>>())
            {
                for (const std::string &object_attribute : receive_objects.second)
                {
                    send_meta_data_json[send_receive][receive_objects.first].append(object_attribute);
                }
            }
        }
    }

    void bind_receive_meta_data() override
    {
        if (receive_meta_data_json.empty())
        {
            return;
        }

        pybind11::gil_scoped_acquire acquire;

        receive_meta_data_dict["world"] = receive_meta_data_json["world"].asString();
        receive_meta_data_dict["length_unit"] = receive_meta_data_json["length_unit"].asString();
        receive_meta_data_dict["angle_unit"] = receive_meta_data_json["angle_unit"].asString();
        receive_meta_data_dict["force_unit"] = receive_meta_data_json["force_unit"].asString();
        receive_meta_data_dict["time_unit"] = receive_meta_data_json["time_unit"].asString();
        receive_meta_data_dict["handedness"] = receive_meta_data_json["handedness"].asString();

        for (const std::string &send_receive : {"send", "receive"})
        {
            receive_meta_data_dict[send_receive.c_str()] = pybind11::dict();
            const Json::Value objects_json = receive_meta_data_json[send_receive];
            for (const std::string &object_name : objects_json.getMemberNames())
            {
                receive_meta_data_dict[send_receive.c_str()][object_name.c_str()] = pybind11::dict();
                const Json::Value object_json = objects_json[object_name];
                for (const std::string &attribute_name : object_json.getMemberNames())
                {
                    pybind11::list data_list;
                    const Json::Value object_data_json = object_json[attribute_name];
                    for (int i = 0; i < object_data_json.size(); i++)
                    {
                        data_list.append(object_data_json[i].asDouble());
                    }
                    receive_meta_data_dict[send_receive.c_str()][object_name.c_str()][attribute_name.c_str()] = data_list;
                }
            }
        }

        pybind11::gil_scoped_release release;
    }

    void clean_up() override
    {
        send_data = pybind11::list();

        receive_data = pybind11::list();
    }

    void init_send_and_receive_data() override
    {
        pybind11::gil_scoped_acquire acquire;

        send_data = pybind11::list(send_buffer_size);

        receive_data = pybind11::list(receive_buffer_size);

        pybind11::gil_scoped_release release;
    }

    void bind_send_data() override
    {
        for (size_t i = 0; i < send_buffer_size; i++)
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

PYBIND11_MODULE(multiverse_socket, handle)
{
    handle.doc() = "";

    pybind11::class_<MultiverseClient>(handle, "MultiverseClient")
        .def("init", &MultiverseClient::init)
        .def("connect", &MultiverseClient::connect)
        .def("communicate", &MultiverseClient::communicate)
        .def("disconnect", &MultiverseClient::disconnect);

    pybind11::class_<MultiverseSocket, MultiverseClient>(handle, "MultiverseSocket", pybind11::is_final())
        .def(pybind11::init<bool>())
        .def("set_send_meta_data", &MultiverseSocket::set_send_meta_data)
        .def("get_receive_meta_data", &MultiverseSocket::get_receive_meta_data)
        .def("set_send_data", &MultiverseSocket::set_send_data)
        .def("get_receive_data", &MultiverseSocket::get_receive_data);
}