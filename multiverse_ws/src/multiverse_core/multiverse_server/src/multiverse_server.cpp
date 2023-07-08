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
#include <thread>
#include <zmq.hpp>

using namespace std::chrono_literals;

enum class EAttribute : unsigned char
{
    Position,
    Quaternion,
    RelativeVelocity,
    JointRvalue,
    JointTvalue,
    JointPosition,
    JointQuaternion,
    Force,
    Torque
};

std::map<std::string, std::pair<EAttribute, std::vector<double>>> attribute_map =
    {
        {"position", {EAttribute::Position, {0.0, 0.0, 0.0}}},
        {"quaternion", {EAttribute::Quaternion, {1.0, 0.0, 0.0, 0.0}}},
        {"relative_velocity", {EAttribute::RelativeVelocity, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}},
        {"joint_rvalue", {EAttribute::JointRvalue, {0.0}}},
        {"joint_tvalue", {EAttribute::JointTvalue, {0.0}}},
        {"joint_position", {EAttribute::JointPosition, {0.0, 0.0, 0.0}}},
        {"joint_quaternion", {EAttribute::JointQuaternion, {1.0, 0.0, 0.0, 0.0}}},
        {"force", {EAttribute::Force, {0.0, 0.0, 0.0}}},
        {"torque", {EAttribute::Torque, {0.0, 0.0, 0.0}}}};

std::map<std::string, double> unit_scale =
    {
        {"s", 1.0},
        {"ms", 0.001},
        {"us", 0.00001},
        {"m", 1.0},
        {"cm", 0.01},
        {"rad", 1.0},
        {"deg", M_PI / 180.0},
        {"N", 1.0}};

std::map<EAttribute, std::map<std::string, std::vector<double>>> handedness_scale =
    {
        {EAttribute::Position,
         {{"rhs", {1.0, 1.0, 1.0}},
          {"lhs", {1.0, -1.0, 1.0}}}},
        {EAttribute::Quaternion,
         {{"rhs", {1.0, 1.0, 1.0, 1.0}},
          {"lhs", {-1.0, 1.0, -1.0, 1.0}}}},
        {EAttribute::RelativeVelocity,
         {{"rhs", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}},
          {"lhs", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}}}},
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

std::map<std::string, std::map<std::string, std::pair<std::vector<double>, bool>>> objects;

std::map<std::string, std::map<std::string, std::map<std::string, std::vector<double>>>> send_efforts;

bool should_shut_down = false;

std::map<std::string, bool> sockets_need_clean_up;

zmq::context_t context{1};

static double get_time_now()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000000.0;
}

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
        std::vector<std::pair<std::vector<double>::iterator, double>> send_data_vec;
        std::vector<std::pair<std::vector<double>::iterator, double>> receive_data_vec;
        std::map<EAttribute, std::vector<double>> conversion_map;
        bool is_received_data_sent;
        bool continue_state = false;

    request_meta_data:
        is_received_data_sent = false;

        // Receive JSON string over ZMQ
        try
        {
            sockets_need_clean_up[socket_addr] = false;
            socket_server.recv(message, zmq::recv_flags::none);
            sockets_need_clean_up[socket_addr] = true;
        }
        catch (const zmq::error_t &e)
        {
            ROS_INFO("%s, server socket %s prepares to close", e.what(), socket_addr.c_str());
            return;
        }

        Json::Reader reader;
        reader.parse(message.to_string(), meta_data_json);

    received_meta_data:
        ROS_INFO("%s", meta_data_json.toStyledString().c_str());

        const std::string length_unit = meta_data_json["length_unit"].asString();
        const std::string angle_unit = meta_data_json["angle_unit"].asString();
        const std::string handedness = meta_data_json["handedness"].asString();
        const std::string force_unit = meta_data_json["force_unit"].asString();
        const std::string time_unit = meta_data_json["time_unit"].asString();

        for (const std::pair<const std::string, std::pair<EAttribute, std::vector<double>>> &attribute : attribute_map)
        {
            conversion_map.emplace(attribute.second);
        }

        std::for_each(conversion_map[EAttribute::Position].begin(), conversion_map[EAttribute::Position].end(),
                      [length_unit](double &position)
                      { position = unit_scale[length_unit]; });

        std::for_each(conversion_map[EAttribute::Quaternion].begin(), conversion_map[EAttribute::Quaternion].end(),
                      [](double &quaternion)
                      { quaternion = 1.0; });

        std::for_each(conversion_map[EAttribute::JointRvalue].begin(), conversion_map[EAttribute::JointRvalue].end(),
                      [angle_unit](double &joint_rvalue)
                      { joint_rvalue = unit_scale[angle_unit]; });

        std::for_each(conversion_map[EAttribute::JointTvalue].begin(), conversion_map[EAttribute::JointTvalue].end(),
                      [length_unit](double &joint_tvalue)
                      { joint_tvalue = unit_scale[length_unit]; });

        std::for_each(conversion_map[EAttribute::JointPosition].begin(), conversion_map[EAttribute::JointPosition].end(),
                      [length_unit](double &joint_position)
                      { joint_position = unit_scale[length_unit]; });

        std::for_each(conversion_map[EAttribute::JointQuaternion].begin(), conversion_map[EAttribute::JointQuaternion].end(),
                      [](double &joint_quaternion)
                      { joint_quaternion = 1.0; });

        std::for_each(conversion_map[EAttribute::Force].begin(), conversion_map[EAttribute::Force].end(),
                      [force_unit](double &force)
                      { force = unit_scale[force_unit]; });

        std::for_each(conversion_map[EAttribute::Torque].begin(), conversion_map[EAttribute::Torque].end(),
                      [force_unit, length_unit](double &torque)
                      { torque = unit_scale[force_unit] * unit_scale[length_unit]; });

        for (size_t i = 0; i < 3; i++)
        {
            conversion_map[EAttribute::RelativeVelocity][i] = unit_scale[length_unit] / unit_scale[time_unit];
        }
        for (size_t i = 3; i < 6; i++)
        {
            conversion_map[EAttribute::RelativeVelocity][i] = unit_scale[angle_unit] / unit_scale[time_unit];
        }

        for (std::pair<const EAttribute, std::vector<double>> &conversion_scale : conversion_map)
        {
            std::vector<double>::iterator conversion_scale_it = conversion_scale.second.begin();
            std::vector<double>::iterator handedness_scale_it = handedness_scale[conversion_scale.first][handedness].begin();
            for (size_t i = 0; i < conversion_scale.second.size(); i++)
            {
                *(conversion_scale_it++) *= *(handedness_scale_it++);
            }
        }

        Json::Value response_json;
        response_json["time"] = get_time_now() * unit_scale[time_unit];
        response_json["angle_unit"] = angle_unit;
        response_json["force_unit"] = force_unit;
        response_json["time_unit"] = time_unit;
        response_json["handedness"] = handedness;

        Json::Value send_objects_json_tmp = meta_data_json["send"];
        const Json::Value send_objects_json = send_objects_json_tmp;

        for (auto send_object_it = send_objects_json.begin(); send_object_it != send_objects_json.end(); ++send_object_it)
        {
            const std::string object_name = send_object_it.key().asString();

            mtx.lock();
            if (objects.count(object_name) == 0)
            {
                objects[object_name] = {};
                objects[object_name]["force"] = {attribute_map["force"].second, false};
                objects[object_name]["torque"] = {attribute_map["torque"].second, false};
            }

            for (const Json::Value &attribute_json : *send_object_it)
            {
                const std::string attribute_name = attribute_json.asString();
                if (objects[object_name].count(attribute_name) == 0)
                {
                    objects[object_name][attribute_name] = {attribute_map[attribute_name].second, false};
                    
                    for (size_t i = 0; i < objects[object_name][attribute_name].first.size(); i++)
                    {
                        double *data = &objects[object_name][attribute_name].first[i];
                        const double conversion = conversion_map[attribute_map[attribute_name].first][i];
                        send_data_vec.emplace_back(data, conversion);
                        response_json["send"][object_name][attribute_name].append(std::numeric_limits<double>::quiet_NaN());
                    }
                }
                else if (strcmp(attribute_name.c_str(), "force") == 0 || strcmp(attribute_name.c_str(), "torque") == 0)
                {
                    send_efforts[object_name][socket_addr][attribute_name] = attribute_map[attribute_name].second;

                    for (size_t i = 0; i < attribute_map[attribute_name].second.size(); i++)
                    {
                        double *data = &send_efforts[object_name][socket_addr][attribute_name][i];
                        const double conversion = conversion_map[attribute_map[attribute_name].first][i];
                        send_data_vec.emplace_back(data, conversion);
                        response_json["send"][object_name][attribute_name].append(*data * conversion);
                    }
                }
                else
                {
                    ROS_INFO("Continue state [%s - %s] on socket %s", object_name.c_str(), attribute_name.c_str(), socket_addr.c_str());
                    continue_state = true;
                    objects[object_name][attribute_name].second = false;

                    for (size_t i = 0; i < objects[object_name][attribute_name].first.size(); i++)
                    {
                        double *data = &objects[object_name][attribute_name].first[i];
                        const double conversion = conversion_map[attribute_map[attribute_name].first][i];
                        send_data_vec.emplace_back(data, conversion);
                        response_json["send"][object_name][attribute_name].append(*data * conversion);
                    }
                }
            }
            mtx.unlock();
        }

        Json::Value receive_objects_json_tmp = meta_data_json["receive"];
        for (auto receive_object_it = meta_data_json["receive"].begin(); receive_object_it != meta_data_json["receive"].end(); ++receive_object_it)
        {
            const std::string object_name = receive_object_it.key().asString();
            if (!object_name.empty())
            {
                continue;
            }
            for (const Json::Value &attribute_json : *receive_object_it)
            {
                for (const std::pair<std::string, std::map<std::string, std::pair<std::vector<double>, bool>>> &object : objects)
                {
                    const std::string attribute_name = attribute_json.asString();
                    if (object.second.count(attribute_name) != 0)
                    {
                        receive_objects_json_tmp[object.first].append(attribute_name);
                    }
                }
            }
            receive_objects_json_tmp.removeMember("");
        }
        const Json::Value receive_objects_json = receive_objects_json_tmp;

        for (auto receive_object_it = receive_objects_json.begin(); receive_object_it != receive_objects_json.end(); ++receive_object_it)
        {
            const std::string object_name = receive_object_it.key().asString();
            for (const Json::Value &attribute_json : *receive_object_it)
            {
                const std::string attribute_name = attribute_json.asString();
                int start = get_time_now();
                while ((objects.count(object_name) == 0 || objects[object_name].count(attribute_name) == 0) && !should_shut_down)
                {
                    const int now = get_time_now();
                    if (now - start > 1)
                    {
                        ROS_INFO("[Socket %s]: Waiting for [%s][%s] to be declared", socket_addr.c_str(), object_name.c_str(), attribute_name.c_str());
                        start = now;
                    }
                }
            }

            for (const Json::Value &attribute_json : *receive_object_it)
            {
                const std::string attribute_name = attribute_json.asString();
                mtx.lock();
                if (strcmp(attribute_name.c_str(), "force") == 0 || strcmp(attribute_name.c_str(), "torque") == 0)
                {
                    for (size_t i = 0; i < 3; i++)
                    {
                        double *data = &objects[object_name][attribute_name].first[i];
                        const double conversion = 1.0 / conversion_map[attribute_map[attribute_name].first][i];
                        receive_data_vec.emplace_back(data, conversion);
                        response_json["receive"][object_name][attribute_name].append(*data * conversion);
                    }
                }
                else
                {
                    for (size_t i = 0; i < objects[object_name][attribute_name].first.size(); i++)
                    {
                        double *data = &objects[object_name][attribute_name].first[i];
                        const double conversion = 1.0 / conversion_map[attribute_map[attribute_name].first][i];
                        receive_data_vec.emplace_back(data, conversion);
                        response_json["receive"][object_name][attribute_name].append(*data * conversion);
                    }
                }
                mtx.unlock();
            }
        }

    send_meta_data:
        const size_t send_buffer_size = 1 + send_data_vec.size();
        const size_t receive_buffer_size = 1 + receive_data_vec.size();

        if (should_shut_down)
        {
            response_json["time"] = -1.0;
        }

        if (continue_state)
        {
            continue_state = false;
        }

        const std::string response_str = response_json.toStyledString();

        // Send buffer sizes and send_data (if exists) over ZMQ
        zmq::message_t response_message(response_str.size());
        memcpy(response_message.data(), response_str.data(), response_str.size());
        socket_server.send(response_message, zmq::send_flags::none);
        
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
            try
            {
                socket_server.recv(message, zmq::recv_flags::none);
            }
            catch (const zmq::error_t &e)
            {
                ROS_INFO("%s, server socket %s prepares to close", e.what(), socket_addr.c_str());
            }

            const std::string request_data_str = message.to_string();
            if (request_data_str[0] == '{')
            {
                reader.parse(request_data_str, meta_data_json);

                send_data_vec.clear();
                receive_data_vec.clear();

                if (request_data_str[1] == '}')
                {
                    goto send_meta_data;
                }
                else if (!meta_data_json.empty())
                {
                    goto received_meta_data;
                }
                else
                {
                    ROS_WARN("Received %s from %s", request_data_str.c_str(), socket_addr.c_str());
                }
            }
            else
            {
                memcpy(send_buffer, message.data(), send_buffer_size * sizeof(double));
            }

            mtx.lock();
            for (size_t i = 0; i < send_buffer_size - 1; i++)
            {
                *send_data_vec[i].first = send_buffer[i + 1] * send_data_vec[i].second;
            }
            mtx.unlock();

            if (!is_received_data_sent)
            {
                int start = get_time_now();
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
                        int start = get_time_now();
                        while ((objects.count(object_name) == 0 || objects[object_name].count(attribute_name) == 0 || !objects[object_name][attribute_name].second) && !should_shut_down)
                        {
                            const int now = get_time_now();
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
            *receive_buffer = get_time_now();
            if (should_shut_down)
            {
                receive_buffer[0] = -1.0;
            }

            mtx.lock();
            for (std::pair<const std::string, std::map<std::string, std::pair<std::vector<double>, bool>>> &object : objects)
            {
                for (const std::string &effort : {"force", "torque"})
                {
                    for (size_t i = 0; i < 3; i++)
                    {
                        for (std::pair<const std::string, std::map<std::string, std::vector<double>>> &data : send_efforts[object.first])
                        {
                            object.second[effort].first[i] = data.second[effort][i];
                        }
                    }
                }
            }
            mtx.unlock();

            for (size_t i = 0; i < receive_buffer_size - 1; i++)
            {
                receive_buffer[i + 1] = *receive_data_vec[i].first * receive_data_vec[i].second;
            }

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
    zmq::message_t message;

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
