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

class MultiverseQueryData final : public MultiverseClient
{
public:
    MultiverseQueryData()
    {
    }

    ~MultiverseQueryData()
    {
    }

    void set_meta_data(const pybind11::dict &in_meta_data_dict)
    {
        meta_data_dict = in_meta_data_dict;
    }

    pybind11::dict get_meta_data_response() const
    {
        pybind11::dict meta_data_res_dict;
        
        meta_data_res_dict["simulator"] = meta_data_res_json["simulator"].asString();
        meta_data_res_dict["length_unit"] = meta_data_res_json["length_unit"].asString();
        meta_data_res_dict["angle_unit"] = meta_data_res_json["angle_unit"].asString(); 
        meta_data_res_dict["force_unit"] = meta_data_res_json["force_unit"].asString(); 
        meta_data_res_dict["time_unit"] = meta_data_res_json["time_unit"].asString();
        meta_data_res_dict["handedness"] = meta_data_res_json["handedness"].asString(); 
        
        for (const std::string& object_name : meta_data_res_json["receive"].getMemberNames()) 
        {
            
            // for (const double object_data : meta_data_res_json["receive"][object_name])
            // {
            //     /* code */
            // }
            
            // meta_data_res_dict["receive"][object_name] = meta_data_res_json["receive"][object_name];
        }
    }

    pybind11::list get_receive_data() const
    {
        pybind11::list receive_data;
        for (size_t i = 0; i < receive_buffer_size; i++)
        {
            receive_data.append(receive_buffer[i]);
        }

        return receive_data;
    }

private:
    pybind11::dict meta_data_dict;

    std::thread meta_data_thread;

private:
    void start_meta_data_thread() override
    {
        MultiverseQueryData::send_and_receive_meta_data();
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
        meta_data_json["simulator"] = "query_data";
        meta_data_json["length_unit"] = meta_data_dict["length_unit"].cast<std::string>();
        meta_data_json["angle_unit"] = meta_data_dict["angle_unit"].cast<std::string>();
        meta_data_json["force_unit"] = meta_data_dict["force_unit"].cast<std::string>();
        meta_data_json["time_unit"] = meta_data_dict["time_unit"].cast<std::string>();
        meta_data_json["handedness"] = meta_data_dict["handedness"].cast<std::string>();

        for (const std::pair<std::string, std::vector<std::string>> receive_objects : meta_data_dict["receive"].cast<std::map<std::string, std::vector<std::string>>>())
        {
            for (const std::string &object_attribute : receive_objects.second)
            {
                meta_data_json["receive"][receive_objects.first].append(object_attribute);
            }
        }
    }

    void bind_object_data() override
    {
    }

    void clean_up() override
    {
    }

    double get_time_now() override
    {
        return 0.0;
    }

    void bind_send_data() override
    {
    }

    void bind_receive_data() override
    {
    }
};

PYBIND11_MODULE(multiverse_client, handle)
{
    handle.doc() = "";

    pybind11::class_<MultiverseClient>(handle, "MultiverseClient")
        .def("init", &MultiverseClient::init)
        .def("connect", &MultiverseClient::connect)
        .def("communicate", &MultiverseClient::communicate)
        .def("disconnect", &MultiverseClient::disconnect)
        .def("send_and_receive_meta_data", &MultiverseClient::send_and_receive_meta_data);

    pybind11::class_<MultiverseQueryData, MultiverseClient>(handle, "MultiverseQueryData", pybind11::is_final())
        .def(pybind11::init<>())
        .def("set_meta_data", &MultiverseQueryData::set_meta_data)
        .def("get_receive_data", &MultiverseQueryData::get_receive_data);
}