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

#include <ros/ros.h>

#include <chrono>
#include <csignal>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/reader.h>
#include <mutex>
#include <string>
#include <list>
#include <thread>
#include <zmq.hpp>

using namespace std::chrono_literals;

enum class EAttribute : unsigned char
{
    Position,
    Quaternion,
    JointRvalue,
    JointTvalue,
    JointPosition,
    JointQuaternion,
    Force,
    Torque
};

std::map<std::string, std::pair<EAttribute, std::list<double>>> attribute_map =
    {
        {"position", {EAttribute::Position, {0.0, 0.0, 0.0}}},
        {"quaternion", {EAttribute::Quaternion, {1.0, 0.0, 0.0, 0.0}}},
        {"joint_rvalue", {EAttribute::JointRvalue, {0.0}}},
        {"joint_tvalue", {EAttribute::JointTvalue, {0.0}}},
        {"joint_position", {EAttribute::JointPosition, {0.0, 0.0, 0.0}}},
        {"joint_quaternion", {EAttribute::JointQuaternion, {1.0, 0.0, 0.0, 0.0}}},
        {"force", {EAttribute::Force, {0.0, 0.0, 0.0}}},
        {"torque", {EAttribute::Torque, {0.0, 0.0, 0.0}}}};

std::map<std::string, std::list<double>> unit_scale =
    {
        {"quat_unit", {1.0, 1.0, 1.0, 1.0}},
        {"m", {1.0, 1.0, 1.0}},
        {"cm", {0.01, 0.01, 0.01}},
        {"rad", {1.0}},
        {"deg", {M_PI / 180.0}},
        {"N", {1.0, 1.0, 1.0}},
        {"Nm", {1.0, 1.0, 1.0}},
        {"Ncm", {0.01, 0.01, 0.01}}};

std::map<EAttribute, std::map<std::string, std::list<double>>> unit_handedness =
    {
        {EAttribute::Position,
         {{"rhs", {1.0, 1.0, 1.0}},
          {"lhs", {1.0, -1.0, 1.0}}}},
        {EAttribute::Quaternion,
         {{"rhs", {1.0, 1.0, 1.0, 1.0}},
          {"lhs", {-1.0, 1.0, -1.0, 1.0}}}},
        {EAttribute::JointRvalue,
         {{"rhs", {1.0}},
          {"lhs", {-1.0}}}},
        {EAttribute::JointTvalue,
         {{"rhs", {1.0}},
          {"lhs", {-1.0}}}},
        {EAttribute::JointPosition,
         {{"rhs", {1.0, 1.0, 1.0}},
          {"lhs", {1.0, -1.0, 1.0}}}},
        {EAttribute::JointQuaternion,
         {{"rhs", {1.0, 1.0, 1.0, 1.0}},
          {"lhs", {1.0, 1.0, -1.0, 1.0}}}},
        {EAttribute::Force,
         {{"rhs", {1.0, 1.0, 1.0}},
          {"lhs", {1.0, -1.0, 1.0}}}},
        {EAttribute::Torque,
         {{"rhs", {1.0, 1.0, 1.0}},
          {"lhs", {1.0, -1.0, 1.0}}}}};

std::vector<std::thread> workers;

std::mutex mtx;

std::map<std::string, std::map<std::string, std::pair<std::list<double>, bool>>> objects;

bool should_shut_down = false;

std::map<std::string, bool> sockets_need_clean_up;

zmq::context_t context{1};

class StateHandle
{
public:
    StateHandle(const std::string &socket_addr) : socket_addr(socket_addr)
    {
        socket_server = zmq::socket_t(context, zmq::socket_type::rep);
        socket_server.bind(socket_addr);
        sockets_need_clean_up[socket_addr] = false;
        ROS_INFO("Bind server socket to address %s", socket_addr.c_str());
    }

    ~StateHandle()
    {
        ROS_INFO("Close server socket %s", socket_addr.c_str());

        if (send_buffer != nullptr)
        {
            free(send_buffer);
        }

        if (receive_buffer != nullptr)
        {
            free(receive_buffer);
        }

        sockets_need_clean_up[socket_addr] = false;
    }

public:
    void communicate()
    {
        std::vector<std::pair<std::list<double>::iterator, double>> send_data_vec;
        std::vector<std::pair<std::list<double>::iterator, double>> receive_data_vec;
        std::map<EAttribute, std::list<double>> conversion_map;
        bool is_received_data_sent;
        bool continue_state = false;

    request_meta_data:
        is_received_data_sent = false;

        // Receive JSON string over ZMQ
        zmq::message_t request_meta_data;
        try
        {
            sockets_need_clean_up[socket_addr] = false;
            socket_server.recv(request_meta_data, zmq::recv_flags::none);
            sockets_need_clean_up[socket_addr] = true;
        }
        catch (const zmq::error_t &e)
        {
            ROS_INFO("%s, server socket %s prepares to close", e.what(), socket_addr.c_str());
            return;
        }

        Json::Reader reader;
        reader.parse(request_meta_data.to_string(), meta_data_json);
        ROS_INFO("%s", meta_data_json.toStyledString().c_str());

    receive_meta_data:
        const std::string length_unit = meta_data_json["length_unit"].asString();
        const std::string angle_unit = meta_data_json["angle_unit"].asString();
        const std::string handedness = meta_data_json["handedness"].asString();
        const std::string force_unit = meta_data_json["force_unit"].asString();
        const std::string torque_unit = force_unit + length_unit;

        conversion_map[EAttribute::Position] = unit_scale[length_unit];
        conversion_map[EAttribute::Quaternion] = unit_scale["quat_unit"];
        conversion_map[EAttribute::JointRvalue] = unit_scale[angle_unit];
        conversion_map[EAttribute::JointTvalue] = {unit_scale[length_unit].front()};
        conversion_map[EAttribute::JointPosition] = unit_scale[length_unit];
        conversion_map[EAttribute::JointQuaternion] = unit_scale["quat_unit"];
        conversion_map[EAttribute::Force] = unit_scale[force_unit];
        conversion_map[EAttribute::Torque] = unit_scale[torque_unit];

        for (std::pair<const EAttribute, std::list<double>> &conversion_element : conversion_map)
        {
            std::list<double>::iterator conversion_element_it = conversion_element.second.begin();
            std::list<double>::iterator handedness_it = unit_handedness[conversion_element.first][handedness].begin();
            for (size_t i = 0; i < conversion_element.second.size(); i++)
            {
                *(conversion_element_it++) *= *(handedness_it++);
            }
        }

        const Json::Value send_objects_json_tmp = meta_data_json["send"];
        const Json::Value send_objects_json = send_objects_json_tmp;

        std::map<std::string, std::map<std::string, std::list<double>::iterator>> effort_index_starts;

        for (auto send_object_it = send_objects_json.begin(); send_object_it != send_objects_json.end(); ++send_object_it)
        {
            const std::string object_name = send_object_it.key().asString();

            mtx.lock();
            if (objects.count(object_name) == 0)
            {
                objects[object_name] = {};
                objects[object_name]["force"] = {attribute_map["force"].second, false};
                objects[object_name]["torque"] = {attribute_map["force"].second, false};
            }

            for (const Json::Value &attribute_json : *send_object_it)
            {
                const std::string attribute_name = attribute_json.asString();
                if (objects[object_name].count(attribute_name) == 0)
                {
                    objects[object_name][attribute_name] = {attribute_map[attribute_name].second, false};

                    std::list<double>::iterator object_data_it = objects[object_name][attribute_name].first.begin();
                    std::list<double>::iterator conversion_it = conversion_map[attribute_map[attribute_name].first].begin();
                    for (size_t i = 0; i < objects[object_name][attribute_name].first.size(); i++)
                    {
                        send_data_vec.emplace_back(object_data_it++, *(conversion_it++));
                    }
                }
                else if (strcmp(attribute_name.c_str(), "force") == 0 || strcmp(attribute_name.c_str(), "torque") == 0)
                {
                    effort_index_starts[object_name][attribute_name] = objects[object_name][attribute_name].first.end();
                    objects[object_name][attribute_name].first.insert(effort_index_starts[object_name][attribute_name], attribute_map[attribute_name].second.begin(), attribute_map[attribute_name].second.end());
                    
                    std::list<double>::iterator object_data_it = std::next(objects[object_name][attribute_name].first.begin(), 3);

                    std::list<double>::iterator conversion_it = conversion_map[attribute_map[attribute_name].first].begin();
                    for (size_t i = 0; i < 3; i++)
                    {
                        send_data_vec.emplace_back(std::next(objects[object_name][attribute_name].first.begin(), i+3), *(conversion_it++));
                    }

                    for (std::list<double>::iterator object_data_it = objects[object_name][attribute_name].first.begin(); object_data_it != objects[object_name][attribute_name].first.end(); object_data_it++)
                    {
                        ROS_INFO("%s - %p - %f", socket_addr.c_str(), &*object_data_it, *object_data_it);
                    }
                    std::list<double>::iterator effort_it0 = objects[object_name][attribute_name].first.begin();
                    ROS_WARN("%s - [%p %p %p %p %p %p]", socket_addr.c_str(), &*effort_it0++, &*effort_it0++, &*effort_it0++, &*effort_it0++, &*effort_it0++, &*effort_it0++);
                }
                else
                {
                    ROS_INFO("Continue state [%s - %s] on socket %s", object_name.c_str(), attribute_name.c_str(), socket_addr.c_str());
                    continue_state = true;
                    objects[object_name][attribute_name].second = false;

                    std::list<double>::iterator object_data_it = objects[object_name][attribute_name].first.begin();
                    std::list<double>::iterator conversion_it = conversion_map[attribute_map[attribute_name].first].begin();
                    for (size_t i = 0; i < objects[object_name][attribute_name].first.size(); i++)
                    {
                        send_data_vec.emplace_back(object_data_it++, *(conversion_it++));
                    }
                }
            }
            mtx.unlock();
        }

        const Json::Value receive_objects_json_tmp = meta_data_json["receive"];
        const Json::Value receive_objects_json = receive_objects_json_tmp;

        for (auto it = receive_objects_json.begin(); it != receive_objects_json.end(); ++it)
        {
            const std::string object_name = it.key().asString();
            for (const Json::Value &attribute_json : *it)
            {
                const std::string attribute_name = attribute_json.asString();
                int start = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
                while ((objects.count(object_name) == 0 || objects[object_name].count(attribute_name) == 0) && !should_shut_down)
                {
                    const int now = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
                    if (now - start > 1)
                    {
                        ROS_INFO("[Socket %s]: Waiting for [%s][%s] to be declared", socket_addr.c_str(), object_name.c_str(), attribute_name.c_str());
                        start = now;
                    }
                }
            }

            for (const Json::Value &attribute_json : *it)
            {
                const std::string attribute_name = attribute_json.asString();
                mtx.lock();
                std::list<double>::iterator object_data_it = objects[object_name][attribute_name].first.begin();
                std::list<double>::iterator conversion_it = conversion_map[attribute_map[attribute_name].first].begin();
                if (strcmp(attribute_name.c_str(), "force") == 0 || strcmp(attribute_name.c_str(), "torque") == 0)
                {
                    for (size_t i = 0; i < 3; i++)
                    {
                        receive_data_vec.emplace_back(object_data_it++, 1.0 / *(conversion_it++));
                    }
                }
                else
                {
                    for (size_t i = 0; i < objects[object_name][attribute_name].first.size(); i++)
                    {
                        receive_data_vec.emplace_back(object_data_it++, 1.0 / *(conversion_it++));
                    }
                }
                mtx.unlock();
            }
        }

    send_meta_data:
        const size_t send_buffer_size = 1 + send_data_vec.size();
        const size_t receive_buffer_size = 1 + receive_data_vec.size();

        double *buffer = (double *)calloc(send_buffer_size + 2, sizeof(double));
        buffer[0] = send_buffer_size;
        buffer[1] = receive_buffer_size;

        if (should_shut_down)
        {
            buffer[0] = -1.0;
            buffer[1] = -1.0;
        }

        if (continue_state)
        {
            buffer[2] = -1.0;

            for (size_t i = 0; i < send_buffer_size - 1; i++)
            {
                buffer[i + 3] = *send_data_vec[i].first * send_data_vec[i].second;
            }

            continue_state = false;
        }
        else
        {
            buffer[2] = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        }

        // Send buffer sizes and send_data (if exists) over ZMQ
        zmq::message_t reply_data((send_buffer_size + 2) * sizeof(double));
        memcpy(reply_data.data(), buffer, (send_buffer_size + 2) * sizeof(double));
        socket_server.send(reply_data, zmq::send_flags::none);

        free(buffer);

        if (send_buffer_size == 1 && receive_buffer_size == 1)
        {
            goto request_meta_data;
        }

        send_buffer = (double *)calloc(send_buffer_size, sizeof(double));
        receive_buffer = (double *)calloc(receive_buffer_size, sizeof(double));

        sockets_need_clean_up[socket_addr] = true;
        while (!should_shut_down)
        {
            // Receive send_data over ZMQ
            zmq::message_t request_data;
            try
            {
                socket_server.recv(request_data, zmq::recv_flags::none);
            }
            catch (const zmq::error_t &e)
            {
                ROS_INFO("%s, server socket %s prepares to close", e.what(), socket_addr.c_str());
            }

            if (request_data.to_string()[0] == '{')
            {
                Json::Reader reader;
                reader.parse(request_data.to_string(), meta_data_json);
                std::cout << meta_data_json.toStyledString() << std::endl;

                send_data_vec.clear();
                receive_data_vec.clear();
                for (auto send_object_it = send_objects_json.begin(); send_object_it != send_objects_json.end(); ++send_object_it)
                {
                    const std::string object_name = send_object_it.key().asString();
                    if (effort_index_starts.count(object_name) != 0)
                    {
                        for (const Json::Value &attribute_json : *send_object_it)
                        {
                            const std::string attribute_name = attribute_json.asString();
                            if (strcmp(attribute_name.c_str(), "force") == 0 || strcmp(attribute_name.c_str(), "torque"))
                            {
                                // mtx.lock();
                                // objects[object_name][attribute_name].first.erase(effort_index_starts[object_name][attribute_name]);
                                // objects[object_name][attribute_name].first.erase(effort_index_starts[object_name][attribute_name]);
                                // objects[object_name][attribute_name].first.erase(effort_index_starts[object_name][attribute_name]);
                                // mtx.unlock();
                            }
                        }
                    }
                }

                if (request_data.to_string()[1] == '}')
                {
                    goto send_meta_data;
                }
                else
                {
                    goto receive_meta_data;
                }
            }

            memcpy(send_buffer, request_data.data(), send_buffer_size * sizeof(double));

            const double delay_ms = (std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() - send_buffer[0]) / 1000.0;

            mtx.lock();
            for (size_t i = 0; i < send_buffer_size - 1; i++)
            {
                *send_data_vec[i].first = send_buffer[i + 1] * send_data_vec[i].second;
                // ROS_WARN("%s, %ld - %p - %f", socket_addr.c_str(), i, &*send_data_vec[i].first, *send_data_vec[i].first);
            }
            mtx.unlock();

            if (!is_received_data_sent)
            {
                int start = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
                for (auto it = send_objects_json.begin(); it != send_objects_json.end(); ++it)
                {
                    const std::string object_name = it.key().asString();
                    for (const Json::Value &attribute_json : *it)
                    {
                        const std::string attribute_name = attribute_json.asString();
                        objects[object_name][attribute_name].second = true;
                    }
                }

                for (auto it = receive_objects_json.begin(); it != receive_objects_json.end(); ++it)
                {
                    const std::string object_name = it.key().asString();
                    for (const Json::Value &attribute_json : *it)
                    {
                        const std::string attribute_name = attribute_json.asString();
                        int start = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
                        while ((objects.count(object_name) == 0 || objects[object_name].count(attribute_name) == 0 || !objects[object_name][attribute_name].second) && !should_shut_down)
                        {
                            const int now = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
                            if (now - start > 1)
                            {
                                ROS_INFO("[Socket %s]: Waiting for data of [%s][%s] to be sent", socket_addr.c_str(), object_name.c_str(), attribute_name.c_str());
                                start = now;
                            }
                        }
                    }
                }

                is_received_data_sent = true;
            }
            *receive_buffer = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
            if (should_shut_down)
            {
                receive_buffer[0] = -1.0;
            }

            mtx.lock();
            for (std::pair<const std::string, std::map<std::string, std::pair<std::list<double>, bool>>> &object : objects)
            {
                for (const std::string &effort : {"force"})
                {
                    double a[3] = {0.0, 0.0, 0.0};
                    std::list<double>::iterator effort_it0 = object.second[effort].first.begin();
                    // ROS_WARN("%s - [%p %p %p %p %p %p]", socket_addr.c_str(), &*effort_it0++, &*effort_it0++, &*effort_it0++, &*effort_it0++, &*effort_it0++, &*effort_it0++);
                    effort_it0 = object.second[effort].first.begin();
                    // ROS_INFO("%s - [%f %f %f %f %f %f]", socket_addr.c_str(), *effort_it0++, *effort_it0++, *effort_it0++, *effort_it0++, *effort_it0++, *effort_it0++);
                    // for (std::list<double>::iterator effort_it = object.second[effort].first.begin(); effort_it != object.second[effort].first.end();)
                    // {
                    //     if (effort_it == object.second[effort].first.begin())
                    //     {
                    //         *(effort_it++) = 0.0;
                    //         *(effort_it++) = 0.0;
                    //         *(effort_it++) = 0.0;
                    //     }
                    //     else
                    //     {
                    //         std::list<double>::iterator start_effort_it = object.second[effort].first.begin();
                    //         a[0] = *(effort_it++);
                    //         a[1] = *(effort_it++);
                    //         a[2] = *(effort_it++);
                    //     }
                    // }
                    std::list<double>::iterator effort_it = object.second[effort].first.begin();
                    // ROS_INFO("%s - [%f %f %f]", socket_addr.c_str(), a[0], a[1], a[2]);
                }
            }

            for (size_t i = 0; i < receive_buffer_size - 1; i++)
            {
                receive_buffer[i + 1] = *receive_data_vec[i].first * receive_data_vec[i].second;
            }
            mtx.unlock();

            // Send receive_data over ZMQ
            zmq::message_t reply_data(receive_buffer_size * sizeof(double));
            memcpy(reply_data.data(), receive_buffer, receive_buffer_size * sizeof(double));
            socket_server.send(reply_data, zmq::send_flags::none);

            if (should_shut_down)
            {
                socket_server.unbind(socket_addr);
                ROS_INFO("Unbind server socket from address %s", socket_addr.c_str());
                return;
            }
        }
    }

private:
    std::string socket_addr;

    zmq::socket_t socket_server;

    Json::Value meta_data_json;

    double *send_buffer;

    double *receive_buffer;
};

void start_state_handle(int port)
{
    StateHandle state_handle("tcp://127.0.0.1:" + std::to_string(port));
    state_handle.communicate();
}

int main(int argc, char **argv)
{
    // register signal SIGINT and signal handler
    signal(SIGINT, [](int signum)
           {
        ROS_INFO("Interrupt signal (%d) received.", signum);
        should_shut_down = true; });

    ros::init(argc, argv, "state_server");

    for (size_t thread_num = 0; thread_num < argc - 1; thread_num++)
    {
        workers.emplace_back(start_state_handle, std::stoi(argv[thread_num + 1]));
    }

    while (!should_shut_down)
    {
    }

    bool can_shut_down = true;
    do
    {
        can_shut_down = true;
        for (const std::pair<std::string, bool> &socket_needs_clean_up : sockets_need_clean_up)
        {
            if (socket_needs_clean_up.second)
            {
                can_shut_down = false;
                break;
            }
        }
    } while (!can_shut_down);

    context.shutdown();

    for (std::thread &worker : workers)
    {
        worker.join();
    }
}
