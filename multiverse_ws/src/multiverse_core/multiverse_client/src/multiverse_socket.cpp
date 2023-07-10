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

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/chrono.h>
#include <thread>

std::map<std::string, size_t> attribute_map = {
    {"", 0},
    {"position", 3},
    {"quaternion", 3},
    {"relative_velocity", 6},
    {"joint_rvalue", 1},
    {"joint_tvalue", 1},
    {"joint_position", 3},
    {"joint_quaternion", 4},
    {"force", 3},
    {"torque", 3}};

class MultiverseSocket final : public MultiverseClient
{
public:
    MultiverseSocket()
    {
    }

    ~MultiverseSocket()
    {
    }

    void set_meta_data(const pybind11::dict &in_meta_data_dict)
    {
        meta_data_dict = in_meta_data_dict;
    }

    pybind11::dict get_meta_data_response() const
    {
        pybind11::dict meta_data_res_dict;

        meta_data_res_dict["world"] = meta_data_res_json["world"].asString();
        meta_data_res_dict["length_unit"] = meta_data_res_json["length_unit"].asString();
        meta_data_res_dict["angle_unit"] = meta_data_res_json["angle_unit"].asString();
        meta_data_res_dict["force_unit"] = meta_data_res_json["force_unit"].asString();
        meta_data_res_dict["time_unit"] = meta_data_res_json["time_unit"].asString();
        meta_data_res_dict["handedness"] = meta_data_res_json["handedness"].asString();

        for (const std::string &send_receive : {"send", "receive"})
        {
            meta_data_res_dict[send_receive.c_str()] = pybind11::dict();
            const Json::Value objects_json = meta_data_res_json[send_receive];
            for (const std::string &object_name : objects_json.getMemberNames())
            {
                meta_data_res_dict[send_receive.c_str()][object_name.c_str()] = pybind11::dict();
                const Json::Value object_json = objects_json[object_name];
                for (const std::string &attribute_name : object_json.getMemberNames())
                {
                    pybind11::list data_list;
                    const Json::Value object_data_json = object_json[attribute_name];
                    for (int i = 0; i < object_data_json.size(); i++)
                    {
                        data_list.append(object_data_json[i].asDouble());
                    }
                    meta_data_res_dict[send_receive.c_str()][object_name.c_str()][attribute_name.c_str()] = data_list;
                }
            }
        }

        return meta_data_res_dict;
    }

    void set_send_data(const pybind11::list &in_send_data)
    {
        send_data = in_send_data;
    }

    pybind11::list get_receive_data() const
    {
        return receive_data;
    }

private:
    pybind11::list send_data;

    pybind11::list receive_data;

    pybind11::dict meta_data_dict;

    std::thread meta_data_thread;

private:
    void start_meta_data_thread() override
    {
        MultiverseSocket::send_and_receive_meta_data();
    }

    void stop_meta_data_thread() override
    {
        if (meta_data_thread.joinable())
        {
            meta_data_thread.join();
        }
    }

    void init_objects() override
    {
    }

    void validate_objects() override
    {
    }

    void construct_meta_data() override
    {
        meta_data_json.clear();
        meta_data_json["world"] = meta_data_dict["world"].cast<std::string>();
        meta_data_json["length_unit"] = meta_data_dict["length_unit"].cast<std::string>();
        meta_data_json["angle_unit"] = meta_data_dict["angle_unit"].cast<std::string>();
        meta_data_json["force_unit"] = meta_data_dict["force_unit"].cast<std::string>();
        meta_data_json["time_unit"] = meta_data_dict["time_unit"].cast<std::string>();
        meta_data_json["handedness"] = meta_data_dict["handedness"].cast<std::string>();

        for (const std::string &send_receive : {"send", "receive"})
        {
            for (const std::pair<std::string, std::vector<std::string>> receive_objects : meta_data_dict[send_receive.c_str()].cast<std::map<std::string, std::vector<std::string>>>())
            {
                for (const std::string &object_attribute : receive_objects.second)
                {
                    meta_data_json[send_receive][receive_objects.first].append(object_attribute);
                }
            }
        }
    }

    void bind_object_data() override
    {
        
    }

    void clean_up() override
    {
        meta_data_json.clear();

        send_data = pybind11::list();

        receive_data = pybind11::list();
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
        .def("disconnect", &MultiverseClient::disconnect)
        .def("send_and_receive_meta_data", &MultiverseClient::send_and_receive_meta_data);

    pybind11::class_<MultiverseSocket, MultiverseClient>(handle, "MultiverseSocket", pybind11::is_final())
        .def(pybind11::init<>())
        .def("set_meta_data", &MultiverseSocket::set_meta_data)
        .def("get_meta_data_response", &MultiverseSocket::get_meta_data_response)
        .def("set_send_data", &MultiverseSocket::set_send_data)
        .def("get_receive_data", &MultiverseSocket::get_receive_data);
}