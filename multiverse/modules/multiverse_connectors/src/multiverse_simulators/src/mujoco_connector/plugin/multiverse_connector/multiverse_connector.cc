// Copyright (c) 2024, Giang Hoang Nguyen - Institute for Artificial Intelligence, University Bremen

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

#include "multiverse_connector.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>

#ifdef __linux__
#include <bits/stdc++.h>
#include <jsoncpp/json/reader.h>
#elif _WIN32
#include <json/reader.h>
#endif

#include <mujoco/mujoco.h>

void replace_all(std::string& str, const std::string& from, const std::string& to) {
  if (from.empty()) {
    return; // Avoid infinite loop
  }

  size_t start_pos = 0;
  while ((start_pos = str.find(from, start_pos)) != std::string::npos) {
    str.replace(start_pos, from.length(), to);
    start_pos += to.length(); // Move past the last replaced segment
  }
}

Json::Value string_to_json(std::string &str)
{
  replace_all(str, "'", "\"");
  if (str.empty())
  {
    return Json::Value();
  }
  Json::Value json;
  Json::Reader reader;
  if (reader.parse(str, json) && !str.empty())
  {
    return json;
  }
  else
  {
    mju_warning_s("Cannot parse %s into a map\n", str.c_str());
    return Json::Value();
  }
}

enum class BodyState : uint8_t
{
  UNKNOWN = 0,
  STATIC = 1,
  FREE = 2,
  HANGING = 3,
};

BodyState get_body_state(const mjModel *m, const int body_id)
{
  if (m->body_mocapid[body_id] != -1)
  {
    return BodyState::STATIC;
  }
  if (m->body_dofnum[body_id] == 6 && m->body_jntadr[body_id] != -1 && m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE)
  {
    return BodyState::FREE;
  }
  if (m->body_dofnum[body_id] == 3 && m->body_jntadr[body_id] != -1 && m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_BALL)
  {
    return BodyState::HANGING;
  }
  return BodyState::UNKNOWN;
}

bool is_attribute_valid(const std::string &obj_name, const std::string &attr_name, const int obj_type, int &attr_size, const mjModel *m)
{
  attr_size = 0;
  switch (obj_type)
  {
  case mjOBJ_BODY:
  {
    const int body_id = mj_name2id(m, mjOBJ_BODY, obj_name.c_str());
    const BodyState body_state = get_body_state(m, body_id);
    if (strcmp(attr_name.c_str(), "position") == 0)
    {
      attr_size = 3;
      return true;
    }
    else if (strcmp(attr_name.c_str(), "quaternion") == 0)
    {
      attr_size = 4;
      return true;
    }
    else if (strcmp(attr_name.c_str(), "relative_velocity") == 0 && body_state == BodyState::FREE)
    {
      attr_size = 6;
      return true;
    }
    else if (strcmp(attr_name.c_str(), "odometric_velocity") == 0)
    {
      attr_size = 6;
      return true;
    }
    else if (strcmp(attr_name.c_str(), "force") == 0 && body_state == BodyState::FREE)
    {
      attr_size = 3;
      return true;
    }
    else if (strcmp(attr_name.c_str(), "torque") == 0 && body_state == BodyState::FREE)
    {
      attr_size = 3;
      return true;
    }
    return false;
  }
  case mjOBJ_JOINT:
  {
    const int joint_id = mj_name2id(m, mjOBJ_JOINT, obj_name.c_str());
    const int joint_type = m->jnt_type[joint_id];
    if (joint_type == mjJNT_HINGE)
    {
      const std::set<const char *> joint_attributes = {"joint_rvalue", "joint_angular_velocity", "joint_angular_acceleration", "joint_torque",};
      if (std::find(joint_attributes.begin(), joint_attributes.end(), attr_name) != joint_attributes.end())
      {
        attr_size = 1;
        return true;
      }
    }
    else if (joint_type == mjJNT_SLIDE)
    {
      const std::set<const char *> joint_attributes = {"joint_tvalue", "joint_linear_velocity", "joint_linear_acceleration", "joint_force"};
      if (std::find(joint_attributes.begin(), joint_attributes.end(), attr_name) != joint_attributes.end())
      {
        attr_size = 1;
        return true;
      }
    }
    return false;
  }
  case mjOBJ_ACTUATOR:
  {
    const std::set<const char *> actuator_attributes = {"cmd_joint_rvalue", "cmd_joint_tvalue", "cmd_joint_angular_velocity", "cmd_joint_linear_velocity", "cmd_joint_angular_acceleration", "cmd_joint_linear_acceleration", "cmd_joint_torque", "cmd_joint_force"};
    if (std::find(actuator_attributes.begin(), actuator_attributes.end(), attr_name) != actuator_attributes.end())
    {
      attr_size = 1;
      return true;
    }
    return false;
  }
  default:
    mju_warning("Object type %d is not supported\n", obj_type);
    return false;
  }
}

void calculate_contact_efforts(std::map<int, mjtNum *> &contact_efforts, const mjModel *m, mjData *d)
{
  mj_markStack(d);
  for (std::pair<const int, mjtNum *> &contact_effort : contact_efforts)
  {
    mjtNum *jac = mj_stackAllocNum(d, 6 * m->nv);
    mj_jacBodyCom(m, d, jac, jac + 3 * m->nv, contact_effort.first);
    mju_mulMatVec(contact_effort.second, jac, d->qfrc_constraint, 6, m->nv);
  }
  mj_freeStack(d);
}

void calculate_odom_velocities(std::map<int, mjtNum *> &odom_velocities, const mjModel *m, const mjData *d)
{
  for (std::pair<const int, mjtNum *> &odom_velocity : odom_velocities)
  {
    const int body_id = odom_velocity.first;
    const int dof_adr = m->body_dofadr[body_id];
    if (m->body_dofnum[body_id] == 6 &&
        m->body_jntadr[body_id] != -1 &&
        m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE)
    {
      const int joint_adr = m->body_jntadr[body_id];
      const int qpos_adr = m->jnt_qposadr[joint_adr];

      const mjtNum w = (d->qpos + qpos_adr)[3];
      const mjtNum x = (d->qpos + qpos_adr)[4];
      const mjtNum y = (d->qpos + qpos_adr)[5];
      const mjtNum z = (d->qpos + qpos_adr)[6];

      const mjtNum sinr_cosp = 2 * (w * x + y * z);
      const mjtNum cosr_cosp = 1 - 2 * (x * x + y * y);
      mjtNum odom_ang_x_joint_pos = std::atan2(sinr_cosp, cosr_cosp);

      const mjtNum sinp = 2 * (w * y - z * x);
      mjtNum odom_ang_y_joint_pos;
      if (mju_abs(sinp) >= 1)
      {
        odom_ang_y_joint_pos = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
      }
      else
      {
        odom_ang_y_joint_pos = std::asin(sinp);
      }

      const mjtNum siny_cosp = 2 * (w * z + x * y);
      const mjtNum cosy_cosp = 1 - 2 * (y * y + z * z);
      mjtNum odom_ang_z_joint_pos = std::atan2(siny_cosp, cosy_cosp);

      mjtNum *qvel = d->qvel + dof_adr;
      qvel[0] = odom_velocity.second[0] * mju_cos(odom_ang_y_joint_pos) * mju_cos(odom_ang_z_joint_pos) + odom_velocity.second[1] * (mju_sin(odom_ang_x_joint_pos) * mju_sin(odom_ang_y_joint_pos) * mju_cos(odom_ang_z_joint_pos) - mju_cos(odom_ang_x_joint_pos) * mju_sin(odom_ang_z_joint_pos)) + odom_velocity.second[2] * (mju_cos(odom_ang_x_joint_pos) * mju_sin(odom_ang_y_joint_pos) * mju_cos(odom_ang_z_joint_pos) + mju_sin(odom_ang_x_joint_pos) * mju_sin(odom_ang_z_joint_pos));
      qvel[1] = odom_velocity.second[0] * mju_cos(odom_ang_y_joint_pos) * mju_sin(odom_ang_z_joint_pos) + odom_velocity.second[1] * (mju_sin(odom_ang_x_joint_pos) * mju_sin(odom_ang_y_joint_pos) * mju_sin(odom_ang_z_joint_pos) + mju_cos(odom_ang_x_joint_pos) * mju_cos(odom_ang_z_joint_pos)) + odom_velocity.second[2] * (mju_cos(odom_ang_x_joint_pos) * mju_sin(odom_ang_y_joint_pos) * mju_sin(odom_ang_z_joint_pos) - mju_sin(odom_ang_x_joint_pos) * mju_cos(odom_ang_z_joint_pos));
      qvel[2] = odom_velocity.second[0] * mju_sin(odom_ang_y_joint_pos) + odom_velocity.second[1] * mju_sin(odom_ang_x_joint_pos) * mju_cos(odom_ang_y_joint_pos) + odom_velocity.second[2] * mju_cos(odom_ang_x_joint_pos) * mju_cos(odom_ang_y_joint_pos);
      qvel[3] = odom_velocity.second[3];
      qvel[4] = odom_velocity.second[4];
      qvel[5] = odom_velocity.second[5];
    }
    else if (m->body_dofnum[body_id] <= 6)
    {
      mjtNum odom_ang_x_joint_pos = 0.0;
      mjtNum odom_ang_y_joint_pos = 0.0;
      mjtNum odom_ang_z_joint_pos = 0.0;

      const int joint_adr = m->body_jntadr[body_id];
      for (int joint_id = joint_adr; joint_id < joint_adr + m->body_jntnum[body_id]; joint_id++)
      {
        const int qpos_adr = m->jnt_qposadr[joint_id];
        if (m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE)
        {
          if (mju_abs(m->jnt_axis[3 * joint_id] - 1.0) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id + 1]) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id + 2]) < mjMINVAL)
          {
            odom_ang_x_joint_pos = d->qpos[qpos_adr];
          }
          else if (mju_abs(m->jnt_axis[3 * joint_id + 1] - 1.0) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id + 2]) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id]) < mjMINVAL)
          {
            odom_ang_y_joint_pos = d->qpos[qpos_adr];
          }
          else if (mju_abs(m->jnt_axis[3 * joint_id + 2] - 1.0) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id]) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id + 1]) < mjMINVAL)
          {
            odom_ang_z_joint_pos = d->qpos[qpos_adr];
          }
        }
      }
      for (int joint_num = 0; joint_num < m->body_jntnum[body_id]; joint_num++)
      {
        const int joint_id = joint_adr + joint_num;
        if (m->jnt_type[joint_id] != mjtJoint::mjJNT_SLIDE && m->jnt_type[joint_id] != mjtJoint::mjJNT_HINGE)
        {
          continue;
        }
        const int dof_id = m->jnt_dofadr[joint_id];
        if (m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE)
        {
          if (mju_abs(m->jnt_axis[3 * joint_id] - 1.0) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id + 1]) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id + 2]) < mjMINVAL)
          {
            d->qvel[dof_id] = odom_velocity.second[0] * mju_cos(odom_ang_y_joint_pos) * mju_cos(odom_ang_z_joint_pos) + odom_velocity.second[1] * (mju_sin(odom_ang_x_joint_pos) * mju_sin(odom_ang_y_joint_pos) * mju_cos(odom_ang_z_joint_pos) - mju_cos(odom_ang_x_joint_pos) * mju_sin(odom_ang_z_joint_pos)) + odom_velocity.second[2] * (mju_cos(odom_ang_x_joint_pos) * mju_sin(odom_ang_y_joint_pos) * mju_cos(odom_ang_z_joint_pos) + mju_sin(odom_ang_x_joint_pos) * mju_sin(odom_ang_z_joint_pos));
          }
          else if (mju_abs(m->jnt_axis[3 * joint_id + 1] - 1.0) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id + 2]) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id]) < mjMINVAL)
          {
            d->qvel[dof_id] = odom_velocity.second[0] * mju_cos(odom_ang_y_joint_pos) * mju_sin(odom_ang_z_joint_pos) + odom_velocity.second[1] * (mju_sin(odom_ang_x_joint_pos) * mju_sin(odom_ang_y_joint_pos) * mju_sin(odom_ang_z_joint_pos) + mju_cos(odom_ang_x_joint_pos) * mju_cos(odom_ang_z_joint_pos)) + odom_velocity.second[2] * (mju_cos(odom_ang_x_joint_pos) * mju_sin(odom_ang_y_joint_pos) * mju_sin(odom_ang_z_joint_pos) - mju_sin(odom_ang_x_joint_pos) * mju_cos(odom_ang_z_joint_pos));
          }
          else if (mju_abs(m->jnt_axis[3 * joint_id + 2] - 1.0) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id]) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id + 1]) < mjMINVAL)
          {
            d->qvel[dof_id] = odom_velocity.second[0] * mju_sin(odom_ang_y_joint_pos) + odom_velocity.second[1] * mju_sin(odom_ang_x_joint_pos) * mju_cos(odom_ang_y_joint_pos) + odom_velocity.second[2] * mju_cos(odom_ang_x_joint_pos) * mju_cos(odom_ang_y_joint_pos);
          }
        }
        else if (m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE)
        {
          if (mju_abs(m->jnt_axis[3 * joint_id] - 1.0) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id + 1]) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id + 2]) < mjMINVAL)
          {
            d->qvel[dof_id] = odom_velocity.second[3];
          }
          else if (mju_abs(m->jnt_axis[3 * joint_id + 1] - 1.0) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id + 2]) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id]) < mjMINVAL)
          {
            d->qvel[dof_id] = odom_velocity.second[4];
          }
          else if (mju_abs(m->jnt_axis[3 * joint_id + 2] - 1.0) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id]) < mjMINVAL && mju_abs(m->jnt_axis[3 * joint_id + 1]) < mjMINVAL)
          {
            d->qvel[dof_id] = odom_velocity.second[5];
          }
        }
      }
    }
  }
}

namespace mujoco::plugin::multiverse_connector
{
  constexpr char host_str[] = "host";
  constexpr char server_port_str[] = "server_port";
  constexpr char client_port_str[] = "client_port";
  constexpr char world_name_str[] = "world_name";
  constexpr char simulation_name_str[] = "simulation_name";
  constexpr char send_str[] = "send";
  constexpr char receive_str[] = "receive";

  std::string GetStringAttr(const mjModel *m, int instance, const char *attr, const std::string &default_value = "")
  {
    const char *value = mj_getPluginConfig(m, instance, attr);
    return (value != nullptr && value[0] != '\0') ? value : default_value;
  }

  MultiverseConnector *MultiverseConnector::Create(const mjModel *m, mjData *d, int instance)
  {
    MultiverseConfig config;
    config.host = GetStringAttr(m, instance, host_str, config.host);
    config.server_port = GetStringAttr(m, instance, server_port_str, config.server_port);
    config.client_port = GetStringAttr(m, instance, client_port_str, config.client_port);
    config.world_name = GetStringAttr(m, instance, world_name_str, config.world_name);
    config.simulation_name = GetStringAttr(m, instance, simulation_name_str, config.simulation_name);

    Json::Reader reader;

    std::string send_json_str = GetStringAttr(m, instance, send_str);
    replace_all(send_json_str, "'", "\"");
    Json::Value send_json = string_to_json(send_json_str);
    const std::map<std::string, std::pair<int, int>> obj_type_map = 
    {
      {"body", {mjOBJ_BODY, m->nbody}},
      {"joint", {mjOBJ_JOINT, m->njnt}},
      {"actuator", {mjOBJ_ACTUATOR, m->na}}
    };
    for (const std::pair<const std::string, std::pair<int, int>> &object_type_pair : obj_type_map)
    {
      const std::string &obj_type_str = object_type_pair.first;
      const int object_type_int = object_type_pair.second.first;
      const int object_type_num = object_type_pair.second.second;
      if (send_json.isMember(obj_type_str))
      {
        for (int object_id = 0; object_id < object_type_num; object_id++)
        {
          if (object_type_int == mjOBJ_JOINT && m->jnt_type[object_id] == mjJNT_FREE)
          {
            continue;
          }
          const char *object_name = mj_id2name(m, object_type_int, object_id);
          if (!object_name)
          {
            mju_warning("%s with id %d does not have a name\n", obj_type_str.c_str(), object_id);
            continue;
          }
          config.send_objects[object_name] = {};
          for (const Json::Value &attribute_json : send_json[obj_type_str])
          {
            const std::string attribute_name = attribute_json.asString();
            int attr_size = 0;
            if (is_attribute_valid(object_name, attribute_name, object_type_int, attr_size, m))
            {
              config.send_objects[object_name].insert(attribute_name);
            }
          }
        }
        send_json.removeMember(obj_type_str);
      }
    }
    for (const std::string &object_name : send_json.getMemberNames())
    {
      config.send_objects[object_name] = {};
      for (const Json::Value &attribute_json : send_json[object_name])
      {
        const std::string attribute_name = attribute_json.asString();
        for (const int obj_type : {mjOBJ_BODY, mjOBJ_JOINT, mjOBJ_ACTUATOR})
        {
          int attr_size = 0;
          if (is_attribute_valid(object_name, attribute_name, obj_type, attr_size, m))
          {
            config.send_objects[object_name].insert(attribute_name);
          }
        }
      }
    }

    std::string receive_json_str = GetStringAttr(m, instance, receive_str);
    replace_all(receive_json_str, "'", "\"");
    Json::Value receive_json = string_to_json(receive_json_str);
    for (const std::string &object_name : receive_json.getMemberNames())
    {
      config.receive_objects[object_name] = {};
      for (const Json::Value &attribute_json : receive_json[object_name])
      {
        const std::string attribute_name = attribute_json.asString();
        for (const int obj_type : {mjOBJ_BODY, mjOBJ_JOINT, mjOBJ_ACTUATOR})
        {
          int attr_size = 0;
          if (is_attribute_valid(object_name, attribute_name, obj_type, attr_size, m))
          {
            config.receive_objects[object_name].insert(attribute_name);
            if (config.send_objects.find(object_name) != config.send_objects.end() && config.send_objects[object_name].find(attribute_name) != config.send_objects[object_name].end())
            {
              config.send_objects[object_name].erase(attribute_name);
              if (config.send_objects[object_name].empty())
              {
                config.send_objects.erase(object_name);
              }
            }
          }
        }
      }
    }

    return new MultiverseConnector(config, m, d);
  }

  void MultiverseConnector::Reset(mjtNum *plugin_state) {}

  void MultiverseConnector::Compute(const mjModel *m, mjData *d, int instance)
  {
    communicate();
    // const std::vector<int> sensor_ids = get_sensor_ids(m, instance);
  }

  void MultiverseConnector::Advance(const mjModel *m, mjData *d, int instance) const
  {
    // act variables already updated by MuJoCo integrating act_dot
  }

  int MultiverseConnector::StateSize(const mjModel *m, int instance)
  {
    return 0;
  }

  void MultiverseConnector::RegisterPlugin()
  {
    mjpPlugin plugin;
    mjp_defaultPlugin(&plugin);
    plugin.name = "mujoco.multiverse_connector";
    plugin.capabilityflags |= mjPLUGIN_PASSIVE;

    std::vector<const char *> attributes = {host_str, server_port_str, client_port_str, world_name_str, simulation_name_str, send_str, receive_str};
    plugin.nattribute = attributes.size();
    plugin.attributes = attributes.data();
    plugin.nstate = MultiverseConnector::StateSize;
    plugin.init = +[](const mjModel *m, mjData *d, int instance)
    {
      MultiverseConnector *multiverse_connector = MultiverseConnector::Create(m, d, instance);
      if (multiverse_connector == nullptr)
      {
        return -1;
      }
      d->plugin_data[instance] = reinterpret_cast<uintptr_t>(multiverse_connector);
      return 0;
    };
    plugin.destroy = +[](mjData *d, int instance)
    {
      delete reinterpret_cast<MultiverseConnector *>(d->plugin_data[instance]);
      d->plugin_data[instance] = 0;
    };
    plugin.reset = +[](const mjModel *m, mjtNum *plugin_state, void *plugin_data,
                       int instance)
    {
      auto *multiverse_connector = reinterpret_cast<MultiverseConnector *>(plugin_data);
      multiverse_connector->Reset(plugin_state);
    };
    plugin.compute =
        +[](const mjModel *m, mjData *d, int instance, int capability_bit)
    {
      auto *multiverse_connector = reinterpret_cast<MultiverseConnector *>(d->plugin_data[instance]);
      multiverse_connector->Compute(m, d, instance);
    };
    mjp_registerPlugin(&plugin);
  }

  MultiverseConnector::MultiverseConnector(MultiverseConfig config, const mjModel *m, mjData *d)
      : config_(std::move(config)), m_((mjModel *)m), d_(d)
  {
    host = config_.host;
    server_port = config_.server_port;
    client_port = config_.client_port;

    *world_time = 0.0;

    printf("Multiverse Server: %s:%s - Multiverse Client: %s:%s\n", host.c_str(), server_port.c_str(), host.c_str(), client_port.c_str());

    connect();

    communicate(true);
  }

  void MultiverseConnector::start_connect_to_server_thread()
  {
    connect_to_server();
  }

  void MultiverseConnector::wait_for_connect_to_server_thread_finish()
  {
  }

  void MultiverseConnector::start_meta_data_thread()
  {
    send_and_receive_meta_data();
  }

  void MultiverseConnector::wait_for_meta_data_thread_finish()
  {
  }

  bool MultiverseConnector::init_objects(bool from_request_meta_data)
  {
    if (from_request_meta_data)
    {
      if (request_meta_data_json["receive"].empty())
      {
        config_.receive_objects.clear();
      }
      if (request_meta_data_json["send"].empty())
      {
        config_.send_objects.clear();
      }
      for (const std::string &object_name : request_meta_data_json["receive"].getMemberNames())
      {
        for (const Json::Value &attribute_json : request_meta_data_json["receive"][object_name])
        {
          const std::string attribute_name = attribute_json.asString();
          config_.receive_objects[object_name].insert(attribute_name);
        }
      }
      for (const std::string &object_name : request_meta_data_json["send"].getMemberNames())
      {
        for (const Json::Value &attribute_json : request_meta_data_json["send"][object_name])
        {
          const std::string attribute_name = attribute_json.asString();
          config_.send_objects[object_name].insert(attribute_name);
        }
      }
    }
    return true;
  }

  void MultiverseConnector::bind_request_meta_data()
  {
    const Json::Value api_callbacks = request_meta_data_json["api_callbacks"];
    const Json::Value api_callbacks_response = request_meta_data_json["api_callbacks_response"];
    request_meta_data_json.clear();

    if (!api_callbacks.isNull())
    {
      request_meta_data_json["api_callbacks"] = api_callbacks;
    }
    if (!api_callbacks_response.isNull())
    {
      request_meta_data_json["api_callbacks_response"] = api_callbacks_response;
    }

    request_meta_data_json["meta_data"]["world_name"] = config_.world_name;
    request_meta_data_json["meta_data"]["simulation_name"] = config_.simulation_name;
    request_meta_data_json["meta_data"]["length_unit"] = "m";
    request_meta_data_json["meta_data"]["angle_unit"] = "rad";
    request_meta_data_json["meta_data"]["mass_unit"] = "kg";
    request_meta_data_json["meta_data"]["time_unit"] = "s";
    request_meta_data_json["meta_data"]["handedness"] = "rhs";

    for (const std::pair<const std::string, std::set<std::string>> &send_object : config_.send_objects)
    {
      const int body_id = mj_name2id(m_, mjtObj::mjOBJ_BODY, send_object.first.c_str());
      const int joint_id = mj_name2id(m_, mjtObj::mjOBJ_JOINT, send_object.first.c_str());
      const int actuator_id = mj_name2id(m_, mjtObj::mjOBJ_ACTUATOR, send_object.first.c_str());
      if (body_id != -1 || joint_id != -1 || actuator_id != -1)
      {
        const std::string object_name = send_object.first;
        for (const std::string &attribute_name : send_object.second)
        {
          request_meta_data_json["send"][object_name].append(attribute_name);
        }
      }
    }

    for (const std::pair<const std::string, std::set<std::string>> &receive_object : config_.receive_objects)
    {
      const int body_id = mj_name2id(m_, mjtObj::mjOBJ_BODY, receive_object.first.c_str());
      const int joint_id = mj_name2id(m_, mjtObj::mjOBJ_JOINT, receive_object.first.c_str());
      const int actuator_id = mj_name2id(m_, mjtObj::mjOBJ_ACTUATOR, receive_object.first.c_str());
      if (body_id != -1 || joint_id != -1 || actuator_id != -1)
      {
        const std::string object_name = receive_object.first;
        for (const std::string &attribute_name : receive_object.second)
        {
          request_meta_data_json["receive"][object_name].append(attribute_name);
        }
      }
    }

    request_meta_data_str = request_meta_data_json.toStyledString();
  }

  void MultiverseConnector::bind_response_meta_data()
  {
    for (const std::pair<const std::string, std::set<std::string>> &send_object : config_.send_objects)
    {
      const int body_id = mj_name2id(m_, mjtObj::mjOBJ_BODY, send_object.first.c_str());
      const int joint_id = mj_name2id(m_, mjtObj::mjOBJ_JOINT, send_object.first.c_str());
      const int mocap_id = m_->body_mocapid[body_id];
      const int actuator_id = mj_name2id(m_, mjtObj::mjOBJ_ACTUATOR, send_object.first.c_str());
      if (body_id != -1)
      {
        const BodyState body_state = get_body_state(m_, body_id);
        switch (body_state)
        {
        case BodyState::STATIC:
        {
          for (const std::string &attribute_name : send_object.second)
          {
            const Json::Value attribute_data = response_meta_data_json["send"][send_object.first][attribute_name];
            if (strcmp(attribute_name.c_str(), "position") == 0)
            {
              const Json::Value x_json = attribute_data[0];
              const Json::Value y_json = attribute_data[1];
              const Json::Value z_json = attribute_data[2];
              if (!x_json.isNull() && !y_json.isNull() && !z_json.isNull())
              {
                d_->mocap_pos[3 * mocap_id] = x_json.asDouble();
                d_->mocap_pos[3 * mocap_id + 1] = y_json.asDouble();
                d_->mocap_pos[3 * mocap_id + 2] = z_json.asDouble();
              }
            }
            else if (strcmp(attribute_name.c_str(), "quaternion") == 0)
            {
              const Json::Value w_json = attribute_data[0];
              const Json::Value x_json = attribute_data[1];
              const Json::Value y_json = attribute_data[2];
              const Json::Value z_json = attribute_data[3];
              if (!w_json.isNull() && !x_json.isNull() && !y_json.isNull() && !z_json.isNull())
              {
                d_->mocap_quat[4 * mocap_id] = w_json.asDouble();
                d_->mocap_quat[4 * mocap_id + 1] = x_json.asDouble();
                d_->mocap_quat[4 * mocap_id + 2] = y_json.asDouble();
                d_->mocap_quat[4 * mocap_id + 3] = z_json.asDouble();
              }
            }
          }
          break;
        }

        case BodyState::FREE:
        {
          mjtNum *xpos_desired = d_->xpos + 3 * body_id;
          mjtNum *xquat_desired = d_->xquat + 4 * body_id;

          for (const std::string &attribute_name : send_object.second)
          {
            const Json::Value attribute_data = response_meta_data_json["send"][send_object.first][attribute_name];
            if (strcmp(attribute_name.c_str(), "position") == 0)
            {
              const Json::Value x_json = attribute_data[0];
              const Json::Value y_json = attribute_data[1];
              const Json::Value z_json = attribute_data[2];
              if (!x_json.isNull() && !y_json.isNull() && !z_json.isNull())
              {
                xpos_desired[0] = x_json.asDouble();
                xpos_desired[1] = y_json.asDouble();
                xpos_desired[2] = z_json.asDouble();
              }
            }
            else if (strcmp(attribute_name.c_str(), "quaternion") == 0)
            {
              const Json::Value w_json = attribute_data[0];
              const Json::Value x_json = attribute_data[1];
              const Json::Value y_json = attribute_data[2];
              const Json::Value z_json = attribute_data[3];
              if (!w_json.isNull() && !x_json.isNull() && !y_json.isNull() && !z_json.isNull())
              {
                xquat_desired[0] = w_json.asDouble();
                xquat_desired[1] = x_json.asDouble();
                xquat_desired[2] = y_json.asDouble();
                xquat_desired[3] = z_json.asDouble();
              }
            }
            else if (strcmp(attribute_name.c_str(), "relative_velocity") == 0)
            {
              const Json::Value qvel_lin_x = attribute_data[0];
              const Json::Value qvel_lin_y = attribute_data[1];
              const Json::Value qvel_lin_z = attribute_data[2];
              const Json::Value qvel_ang_x = attribute_data[3];
              const Json::Value qvel_ang_y = attribute_data[4];
              const Json::Value qvel_ang_z = attribute_data[5];
              if (!qvel_lin_x.isNull() && !qvel_lin_y.isNull() && !qvel_lin_z.isNull() && !qvel_ang_x.isNull() && !qvel_ang_y.isNull() && !qvel_ang_z.isNull())
              {
                const int qvel_adr = m_->body_dofadr[body_id];
                d_->qvel[qvel_adr] = qvel_lin_x.asDouble();
                d_->qvel[qvel_adr + 1] = qvel_lin_y.asDouble();
                d_->qvel[qvel_adr + 2] = qvel_lin_z.asDouble();
                d_->qvel[qvel_adr + 3] = qvel_ang_x.asDouble();
                d_->qvel[qvel_adr + 4] = qvel_ang_y.asDouble();
                d_->qvel[qvel_adr + 5] = qvel_ang_z.asDouble();
              }
            }
            else if (strcmp(attribute_name.c_str(), "odometric_velocity") == 0)
            {
              const Json::Value odom_vel_lin_x = attribute_data[0];
              const Json::Value odom_vel_lin_y = attribute_data[1];
              const Json::Value odom_vel_lin_z = attribute_data[2];
              const Json::Value odom_vel_ang_x = attribute_data[3];
              const Json::Value odom_vel_ang_y = attribute_data[4];
              const Json::Value odom_vel_ang_z = attribute_data[5];
              if (!odom_vel_lin_x.isNull() && !odom_vel_lin_y.isNull() && !odom_vel_lin_z.isNull() && !odom_vel_ang_x.isNull() && !odom_vel_ang_y.isNull() && !odom_vel_ang_z.isNull())
              {
                odom_velocities[body_id][0] = odom_vel_lin_x.asDouble();
                odom_velocities[body_id][1] = odom_vel_lin_y.asDouble();
                odom_velocities[body_id][2] = odom_vel_lin_z.asDouble();
                odom_velocities[body_id][3] = odom_vel_ang_x.asDouble();
                odom_velocities[body_id][4] = odom_vel_ang_y.asDouble();
                odom_velocities[body_id][5] = odom_vel_ang_z.asDouble();
              }
            }
          }

          const int qpos_adr = m_->jnt_qposadr[m_->body_jntadr[body_id]];
          d_->qpos[qpos_adr] = xpos_desired[0];
          d_->qpos[qpos_adr + 1] = xpos_desired[1];
          d_->qpos[qpos_adr + 2] = xpos_desired[2];
          d_->qpos[qpos_adr + 3] = xquat_desired[0];
          d_->qpos[qpos_adr + 4] = xquat_desired[1];
          d_->qpos[qpos_adr + 5] = xquat_desired[2];
          d_->qpos[qpos_adr + 6] = xquat_desired[3];

          break;
        }

        case BodyState::HANGING:
        {
          for (const std::string &attribute_name : send_object.second)
          {
            const Json::Value attribute_data = response_meta_data_json["send"][send_object.first][attribute_name];
            if (strcmp(attribute_name.c_str(), "quaternion") == 0)
            {
              const Json::Value w_json = attribute_data[0];
              const Json::Value x_json = attribute_data[1];
              const Json::Value y_json = attribute_data[2];
              const Json::Value z_json = attribute_data[3];

              if (!w_json.isNull() && !x_json.isNull() && !y_json.isNull() && !z_json.isNull())
              {
                const mjtNum xquat_desired[4] = {w_json.asDouble(), x_json.asDouble(), y_json.asDouble(), z_json.asDouble()};
                mjtNum *xquat_current_neg = d_->xquat + 4 * body_id;
                mju_negQuat(xquat_current_neg, xquat_current_neg);

                const int qpos_id = m_->jnt_qposadr[m_->body_jntadr[body_id]];
                mju_mulQuat(d_->qpos + qpos_id, xquat_current_neg, xquat_desired);
              }
            }
          }
          break;
        }

        case BodyState::UNKNOWN:
        {
          break;
        }
        }

        if (body_state != BodyState::STATIC)
        {
          for (const std::string &attribute_name : send_object.second)
          {
            const Json::Value attribute_data = response_meta_data_json["send"][send_object.first][attribute_name];
            if (strcmp(attribute_name.c_str(), "force") == 0)
            {
              const Json::Value x_json = attribute_data[0];
              const Json::Value y_json = attribute_data[1];
              const Json::Value z_json = attribute_data[2];
              if (!x_json.isNull() && !y_json.isNull() && !z_json.isNull())
              {
                d_->xfrc_applied[6 * body_id] = x_json.asDouble();
                d_->xfrc_applied[6 * body_id + 1] = y_json.asDouble();
                d_->xfrc_applied[6 * body_id + 2] = z_json.asDouble();
              }
            }
            else if (strcmp(attribute_name.c_str(), "torque") == 0)
            {
              const Json::Value x_json = attribute_data[0];
              const Json::Value y_json = attribute_data[1];
              const Json::Value z_json = attribute_data[2];
              if (!x_json.isNull() && !y_json.isNull() && !z_json.isNull())
              {
                d_->xfrc_applied[6 * body_id + 3] = x_json.asDouble();
                d_->xfrc_applied[6 * body_id + 4] = y_json.asDouble();
                d_->xfrc_applied[6 * body_id + 5] = z_json.asDouble();
              }
            }
          }
        }
      }
      if (joint_id != -1)
      {
        const bool is_revolute_joint = m_->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE;
        const bool is_prismatic_joint = m_->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE;
        const bool is_ball_joint = m_->jnt_type[joint_id] == mjtJoint::mjJNT_BALL;
        for (const std::string &attribute_name : send_object.second)
        {
          const Json::Value attribute_data = response_meta_data_json["send"][send_object.first][attribute_name];
          if ((strcmp(attribute_name.c_str(), "joint_rvalue") == 0 && is_revolute_joint) ||
              (strcmp(attribute_name.c_str(), "joint_tvalue") == 0 && is_prismatic_joint))
          {
            const Json::Value v_json = attribute_data[0];
            if (!v_json.isNull())
            {
              const int qpos_id = m_->jnt_qposadr[joint_id];
              d_->qpos[qpos_id] = v_json.asDouble();
            }
          }
          else if ((strcmp(attribute_name.c_str(), "joint_angular_velocity") == 0 && is_revolute_joint) ||
                   (strcmp(attribute_name.c_str(), "joint_linear_velocity") == 0 && is_prismatic_joint))
          {
            const Json::Value v_json = attribute_data[0];
            if (!v_json.isNull())
            {
              const int dof_id = m_->jnt_dofadr[joint_id];
              d_->qvel[dof_id] = v_json.asDouble();
            }
          }
          else if ((strcmp(attribute_name.c_str(), "joint_angular_acceleration") == 0 && is_revolute_joint) ||
                   (strcmp(attribute_name.c_str(), "joint_linear_acceleration") == 0 && is_prismatic_joint))
          {
            const Json::Value v_json = attribute_data[0];
            if (!v_json.isNull())
            {
              const int dof_id = m_->jnt_dofadr[joint_id];
              d_->qacc[dof_id] = v_json.asDouble();
            }
          }
          else if ((strcmp(attribute_name.c_str(), "joint_torque") == 0 && is_revolute_joint) ||
                   (strcmp(attribute_name.c_str(), "joint_force") == 0 && is_prismatic_joint))
          {
            const Json::Value v_json = attribute_data[0];
            if (!v_json.isNull())
            {
              const int dof_id = m_->jnt_dofadr[joint_id];
              d_->qfrc_applied[dof_id] = v_json.asDouble();
            }
          }
          else if ((strcmp(attribute_name.c_str(), "joint_quaternion") == 0 && is_ball_joint))
          {
            const Json::Value w_json = attribute_data[0];
            const Json::Value x_json = attribute_data[1];
            const Json::Value y_json = attribute_data[2];
            const Json::Value z_json = attribute_data[3];

            if (!w_json.isNull() && !x_json.isNull() && !y_json.isNull() && !z_json.isNull())
            {
              const int qpos_adr = m_->jnt_qposadr[joint_id];
              d_->qpos[qpos_adr] = w_json.asDouble();
              d_->qpos[qpos_adr + 1] = x_json.asDouble();
              d_->qpos[qpos_adr + 2] = y_json.asDouble();
              d_->qpos[qpos_adr + 3] = z_json.asDouble();
            }
          }
        }
      }
      if (actuator_id != -1)
      {
        for (const std::string &attribute_name : send_object.second)
        {
          if (strcmp(attribute_name.c_str(), "cmd_joint_rvalue") == 0 ||
              strcmp(attribute_name.c_str(), "cmd_joint_tvalue") == 0 ||
              strcmp(attribute_name.c_str(), "cmd_joint_angular_velocity") == 0 ||
              strcmp(attribute_name.c_str(), "cmd_joint_linear_velocity") == 0 ||
              strcmp(attribute_name.c_str(), "cmd_joint_angular_acceleration") == 0 ||
              strcmp(attribute_name.c_str(), "cmd_joint_linear_acceleration") == 0 ||
              strcmp(attribute_name.c_str(), "cmd_joint_torque") == 0 ||
              strcmp(attribute_name.c_str(), "cmd_joint_force") == 0)
          {
            d_->ctrl[actuator_id] = response_meta_data_json["send"][send_object.first][attribute_name][0].asDouble();
          }
        }
      }
    }
  }

  void MultiverseConnector::bind_api_callbacks()
  {
  }

  void MultiverseConnector::bind_api_callbacks_response()
  {
  }

  void MultiverseConnector::clean_up()
  {
    send_data_vec.clear();

    for (std::pair<const int, mjtNum *> &contact_effort : contact_efforts)
    {
      if (contact_effort.second != nullptr)
      {
        free(contact_effort.second);
      }
    }
    contact_efforts.clear();
  }

  void MultiverseConnector::reset()
  {
    d_->time = 0.0;
  }

  void MultiverseConnector::init_send_and_receive_data()
  {
    for (const std::pair<const std::string, std::set<std::string>> &send_object : config_.send_objects)
    {
      const int body_id = mj_name2id(m_, mjtObj::mjOBJ_BODY, send_object.first.c_str());
      const int mocap_id = m_->body_mocapid[body_id];
      const int joint_id = mj_name2id(m_, mjtObj::mjOBJ_JOINT, send_object.first.c_str());
      const int actuator_id = mj_name2id(m_, mjtObj::mjOBJ_ACTUATOR, send_object.first.c_str());
      if (body_id != -1)
      {
        const std::string body_name = send_object.first;
        if (mocap_id != -1)
        {
          for (const std::string &attribute_name : send_object.second)
          {
            if (strcmp(attribute_name.c_str(), "position") == 0)
            {
              send_data_vec.emplace_back(&d_->mocap_pos[3 * mocap_id]);
              send_data_vec.emplace_back(&d_->mocap_pos[3 * mocap_id + 1]);
              send_data_vec.emplace_back(&d_->mocap_pos[3 * mocap_id + 2]);
            }
            else if (strcmp(attribute_name.c_str(), "quaternion") == 0)
            {
              send_data_vec.emplace_back(&d_->mocap_quat[4 * mocap_id]);
              send_data_vec.emplace_back(&d_->mocap_quat[4 * mocap_id + 1]);
              send_data_vec.emplace_back(&d_->mocap_quat[4 * mocap_id + 2]);
              send_data_vec.emplace_back(&d_->mocap_quat[4 * mocap_id + 3]);
            }
            else if (joint_id == -1 && actuator_id == -1)
            {
              mju_warning("Send %s for %s not supported\n", attribute_name.c_str(), body_name.c_str());
            }
          }
        }
        else
        {
          const int dof_id = m_->body_dofadr[body_id];
          const BodyState body_state = get_body_state(m_, body_id);
          for (const std::string &attribute_name : send_object.second)
          {
            if (strcmp(attribute_name.c_str(), "position") == 0)
            {
              send_data_vec.emplace_back(&d_->xpos[3 * body_id]);
              send_data_vec.emplace_back(&d_->xpos[3 * body_id + 1]);
              send_data_vec.emplace_back(&d_->xpos[3 * body_id + 2]);
            }
            else if (strcmp(attribute_name.c_str(), "quaternion") == 0)
            {
              send_data_vec.emplace_back(&d_->xquat[4 * body_id]);
              send_data_vec.emplace_back(&d_->xquat[4 * body_id + 1]);
              send_data_vec.emplace_back(&d_->xquat[4 * body_id + 2]);
              send_data_vec.emplace_back(&d_->xquat[4 * body_id + 3]);
            }
            else if (strcmp(attribute_name.c_str(), "force") == 0 && body_state == BodyState::FREE)
            {
              if (contact_efforts.count(body_id) == 0)
              {
                contact_efforts[body_id] = (mjtNum *)calloc(6, sizeof(mjtNum));
              }

              send_data_vec.emplace_back(&contact_efforts[body_id][0]);
              send_data_vec.emplace_back(&contact_efforts[body_id][1]);
              send_data_vec.emplace_back(&contact_efforts[body_id][2]);
            }
            else if (strcmp(attribute_name.c_str(), "torque") == 0 && body_state == BodyState::FREE)
            {
              if (contact_efforts.count(body_id) == 0)
              {
                contact_efforts[body_id] = (mjtNum *)calloc(6, sizeof(mjtNum));
              }

              send_data_vec.emplace_back(&contact_efforts[body_id][3]);
              send_data_vec.emplace_back(&contact_efforts[body_id][4]);
              send_data_vec.emplace_back(&contact_efforts[body_id][5]);
            }
            else if (strcmp(attribute_name.c_str(), "relative_velocity") == 0 && body_state == BodyState::FREE)
            {
              send_data_vec.emplace_back(&d_->qvel[dof_id]);
              send_data_vec.emplace_back(&d_->qvel[dof_id + 1]);
              send_data_vec.emplace_back(&d_->qvel[dof_id + 2]);
              send_data_vec.emplace_back(&d_->qvel[dof_id + 3]);
              send_data_vec.emplace_back(&d_->qvel[dof_id + 4]);
              send_data_vec.emplace_back(&d_->qvel[dof_id + 5]);
            }
            else if (strcmp(attribute_name.c_str(), "odometric_velocity") == 0 && m_->body_dofnum[body_id] <= 6 && m_->body_jntadr[body_id] != -1)
            {
              odom_velocities[body_id] = (mjtNum *)calloc(6, sizeof(mjtNum));
              send_data_vec.emplace_back(&odom_velocities[body_id][0]);
              send_data_vec.emplace_back(&odom_velocities[body_id][1]);
              send_data_vec.emplace_back(&odom_velocities[body_id][2]);
              send_data_vec.emplace_back(&odom_velocities[body_id][3]);
              send_data_vec.emplace_back(&odom_velocities[body_id][4]);
              send_data_vec.emplace_back(&odom_velocities[body_id][5]);
            }
          }
        }
      }
      if (joint_id != -1)
      {
        const std::string joint_name = send_object.first;
        const int qpos_id = m_->jnt_qposadr[joint_id];
        const int dof_id = m_->jnt_dofadr[joint_id];
        for (const std::string &attribute_name : send_object.second)
        {
          const bool is_revolute_joint = m_->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE;
          const bool is_prismatic_joint = m_->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE;
          const bool is_ball_joint = m_->jnt_type[joint_id] == mjtJoint::mjJNT_BALL;
          if ((strcmp(attribute_name.c_str(), "joint_rvalue") == 0 && is_revolute_joint) ||
              (strcmp(attribute_name.c_str(), "joint_tvalue") == 0 && is_prismatic_joint))
          {
            send_data_vec.emplace_back(&d_->qpos[qpos_id]);
          }
          else if ((strcmp(attribute_name.c_str(), "joint_angular_velocity") == 0 && is_revolute_joint) ||
                   (strcmp(attribute_name.c_str(), "joint_linear_velocity") == 0 && is_prismatic_joint))
          {
            send_data_vec.emplace_back(&d_->qvel[dof_id]);
          }
          else if ((strcmp(attribute_name.c_str(), "joint_angular_acceleration") == 0 && is_revolute_joint) ||
                   (strcmp(attribute_name.c_str(), "joint_linear_acceleration") == 0 && is_prismatic_joint))
          {
            send_data_vec.emplace_back(&d_->qacc[dof_id]);
          }
          else if ((strcmp(attribute_name.c_str(), "joint_torque") == 0 && is_revolute_joint) ||
                   (strcmp(attribute_name.c_str(), "joint_force") == 0 && is_prismatic_joint))
          {
            send_data_vec.emplace_back(&d_->qfrc_inverse[dof_id]);
          }
          else if (strcmp(attribute_name.c_str(), "joint_position") == 0)
          {
            mju_warning("Send %s for %s not supported yet\n", attribute_name.c_str(), joint_name.c_str());
          }
          else if (strcmp(attribute_name.c_str(), "joint_quaternion") == 0 && is_prismatic_joint && is_ball_joint)
          {
            send_data_vec.emplace_back(&d_->qpos[qpos_id]);
            send_data_vec.emplace_back(&d_->qpos[qpos_id + 1]);
            send_data_vec.emplace_back(&d_->qpos[qpos_id + 2]);
            send_data_vec.emplace_back(&d_->qpos[qpos_id + 3]);
          }
          else if (body_id == -1 && actuator_id == -1)
          {
            mju_warning("Send %s for %s not supported\n", attribute_name.c_str(), joint_name.c_str());
          }
        }
      }
      if (actuator_id != -1)
      {
        const std::string actuator_name = send_object.first;
        for (const std::string &attribute_name : send_object.second)
        {
          if (strcmp(attribute_name.c_str(), "cmd_joint_rvalue") == 0 ||
              strcmp(attribute_name.c_str(), "cmd_joint_tvalue") == 0 ||
              strcmp(attribute_name.c_str(), "cmd_joint_angular_velocity") == 0 ||
              strcmp(attribute_name.c_str(), "cmd_joint_linear_velocity") == 0 ||
              strcmp(attribute_name.c_str(), "cmd_joint_angular_acceleration") == 0 ||
              strcmp(attribute_name.c_str(), "cmd_joint_linear_acceleration") == 0 ||
              strcmp(attribute_name.c_str(), "cmd_joint_torque") == 0 ||
              strcmp(attribute_name.c_str(), "cmd_joint_force") == 0)
          {
            send_data_vec.emplace_back(&d_->ctrl[actuator_id]);
          }
          else if (body_id == -1 && joint_id == -1)
          {
            printf("Send %s for %s not supported\n", attribute_name.c_str(), actuator_name.c_str());
          }
        }
      }
    }

    for (const std::pair<const std::string, std::set<std::string>> &receive_object : config_.receive_objects)
    {
      const int body_id = mj_name2id(m_, mjtObj::mjOBJ_BODY, receive_object.first.c_str());
      const int mocap_id = m_->body_mocapid[body_id];
      const int joint_id = mj_name2id(m_, mjtObj::mjOBJ_JOINT, receive_object.first.c_str());
      const int actuator_id = mj_name2id(m_, mjtObj::mjOBJ_ACTUATOR, receive_object.first.c_str());
      if (body_id != -1)
      {
        const std::string body_name = receive_object.first;
        const BodyState body_state = get_body_state(m_, body_id);
        if (body_state == BodyState::STATIC)
        {
          for (const std::string &attribute_name : receive_object.second)
          {
            if (strcmp(attribute_name.c_str(), "position") == 0)
            {
              receive_data_vec.emplace_back(&d_->mocap_pos[3 * mocap_id]);
              receive_data_vec.emplace_back(&d_->mocap_pos[3 * mocap_id + 1]);
              receive_data_vec.emplace_back(&d_->mocap_pos[3 * mocap_id + 2]);
            }
            else if (strcmp(attribute_name.c_str(), "quaternion") == 0)
            {
              receive_data_vec.emplace_back(&d_->mocap_quat[4 * mocap_id]);
              receive_data_vec.emplace_back(&d_->mocap_quat[4 * mocap_id + 1]);
              receive_data_vec.emplace_back(&d_->mocap_quat[4 * mocap_id + 2]);
              receive_data_vec.emplace_back(&d_->mocap_quat[4 * mocap_id + 3]);
            }
          }
        }
        else
        {
          const int dof_id = m_->body_dofadr[body_id];
          for (const std::string &attribute_name : receive_object.second)
          {
            if (strcmp(attribute_name.c_str(), "position") == 0 && body_state == BodyState::FREE)
            {
              int qpos_id = m_->jnt_qposadr[m_->body_jntadr[body_id]];
              receive_data_vec.emplace_back(&d_->qpos[qpos_id]);
              receive_data_vec.emplace_back(&d_->qpos[qpos_id + 1]);
              receive_data_vec.emplace_back(&d_->qpos[qpos_id + 2]);
            }
            else if (strcmp(attribute_name.c_str(), "quaternion") == 0)
            {
              if (body_state == BodyState::FREE)
              {
                int qpos_id = m_->jnt_qposadr[m_->body_jntadr[body_id]];
                receive_data_vec.emplace_back(&d_->qpos[qpos_id + 3]);
                receive_data_vec.emplace_back(&d_->qpos[qpos_id + 4]);
                receive_data_vec.emplace_back(&d_->qpos[qpos_id + 5]);
                receive_data_vec.emplace_back(&d_->qpos[qpos_id + 6]);
              }
              else if (m_->body_dofnum[body_id] == 3 && body_state == BodyState::HANGING)
              {
                int qpos_id = m_->jnt_qposadr[m_->body_jntadr[body_id]];
                receive_data_vec.emplace_back(&d_->qpos[qpos_id]);
                receive_data_vec.emplace_back(&d_->qpos[qpos_id + 1]);
                receive_data_vec.emplace_back(&d_->qpos[qpos_id + 2]);
                receive_data_vec.emplace_back(&d_->qpos[qpos_id + 3]);
              }
            }
            else if (strcmp(attribute_name.c_str(), "force") == 0)
            {
              receive_data_vec.emplace_back(&d_->xfrc_applied[6 * body_id]);
              receive_data_vec.emplace_back(&d_->xfrc_applied[6 * body_id + 1]);
              receive_data_vec.emplace_back(&d_->xfrc_applied[6 * body_id + 2]);
            }
            else if (strcmp(attribute_name.c_str(), "torque") == 0)
            {
              receive_data_vec.emplace_back(&d_->xfrc_applied[6 * body_id + 3]);
              receive_data_vec.emplace_back(&d_->xfrc_applied[6 * body_id + 4]);
              receive_data_vec.emplace_back(&d_->xfrc_applied[6 * body_id + 5]);
            }
            else if (strcmp(attribute_name.c_str(), "relative_velocity") == 0 && body_state == BodyState::FREE && odom_velocities.count(body_id) == 0)
            {
              receive_data_vec.emplace_back(&d_->qvel[dof_id]);
              receive_data_vec.emplace_back(&d_->qvel[dof_id + 1]);
              receive_data_vec.emplace_back(&d_->qvel[dof_id + 2]);
              receive_data_vec.emplace_back(&d_->qvel[dof_id + 3]);
              receive_data_vec.emplace_back(&d_->qvel[dof_id + 4]);
              receive_data_vec.emplace_back(&d_->qvel[dof_id + 5]);
            }
            else if (strcmp(attribute_name.c_str(), "odometric_velocity") == 0 && m_->body_dofnum[body_id] <= 6 && m_->body_jntadr[body_id] != -1 && odom_velocities.count(body_id) == 0)
            {
              odom_velocities[body_id] = (mjtNum *)calloc(6, sizeof(mjtNum));
              receive_data_vec.emplace_back(&odom_velocities[body_id][0]);
              receive_data_vec.emplace_back(&odom_velocities[body_id][1]);
              receive_data_vec.emplace_back(&odom_velocities[body_id][2]);
              receive_data_vec.emplace_back(&odom_velocities[body_id][3]);
              receive_data_vec.emplace_back(&odom_velocities[body_id][4]);
              receive_data_vec.emplace_back(&odom_velocities[body_id][5]);
            }
          }
        }
      }
      if (joint_id != -1)
      {
        const std::string joint_name = receive_object.first;
        const int qpos_id = m_->jnt_qposadr[joint_id];
        const int dof_id = m_->jnt_dofadr[joint_id];
        const bool is_revolute_joint = m_->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE;
        const bool is_prismatic_joint = m_->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE;
        for (const std::string &attribute_name : receive_object.second)
        {
          if ((strcmp(attribute_name.c_str(), "joint_rvalue") == 0 && is_revolute_joint) ||
              (strcmp(attribute_name.c_str(), "joint_tvalue") == 0 && is_prismatic_joint))
          {
            receive_data_vec.emplace_back(&d_->qpos[qpos_id]);
          }
          else if ((strcmp(attribute_name.c_str(), "joint_angular_velocity") == 0 && is_revolute_joint) ||
                   (strcmp(attribute_name.c_str(), "joint_linear_velocity") == 0 && is_prismatic_joint))
          {
            receive_data_vec.emplace_back(&d_->qvel[dof_id]);
          }
          else if ((strcmp(attribute_name.c_str(), "joint_angular_acceleration") == 0 && is_revolute_joint) ||
                   (strcmp(attribute_name.c_str(), "joint_linear_acceleration") == 0 && is_prismatic_joint))
          {
            receive_data_vec.emplace_back(&d_->qvel[dof_id]);
          }
          else if ((strcmp(attribute_name.c_str(), "joint_torque") == 0 && is_revolute_joint) ||
                   (strcmp(attribute_name.c_str(), "joint_force") == 0 && is_prismatic_joint))
          {
            receive_data_vec.emplace_back(&d_->qfrc_applied[dof_id]);
          }
        }
      }
      if (actuator_id != -1)
      {
        const std::string actuator_name = receive_object.first;
        for (const std::string &attribute_name : receive_object.second)
        {
          if (strcmp(attribute_name.c_str(), "cmd_joint_rvalue") == 0 ||
              strcmp(attribute_name.c_str(), "cmd_joint_tvalue") == 0 ||
              strcmp(attribute_name.c_str(), "cmd_joint_angular_velocity") == 0 ||
              strcmp(attribute_name.c_str(), "cmd_joint_linear_velocity") == 0 ||
              strcmp(attribute_name.c_str(), "cmd_joint_angular_acceleration") == 0 ||
              strcmp(attribute_name.c_str(), "cmd_joint_linear_acceleration") == 0 ||
              strcmp(attribute_name.c_str(), "cmd_joint_torque") == 0 ||
              strcmp(attribute_name.c_str(), "cmd_joint_force") == 0)
          {
            receive_data_vec.emplace_back(&d_->ctrl[actuator_id]);
          }
          else if (body_id == -1 && joint_id == -1)
          {
            printf("Receive %s for %s not supported\n", attribute_name.c_str(), actuator_name.c_str());
          }
        }
      }
    }
  }

  void MultiverseConnector::bind_send_data()
  {
    *world_time = d_->time;
    if (send_data_vec.size() != send_buffer.buffer_double.size)
    {
      mju_warning("Mismatch between send_data_vec [%zd] and send_buffer.buffer_double.size [%zd]\n", send_data_vec.size(), send_buffer.buffer_double.size);
      return;
    }

    calculate_contact_efforts(contact_efforts, m_, d_);

    for (size_t i = 0; i < send_buffer.buffer_double.size; i++)
    {
      send_buffer.buffer_double.data[i] = *send_data_vec[i];
    }
  }

  void MultiverseConnector::bind_receive_data()
  {
    if (receive_data_vec.size() != receive_buffer.buffer_double.size)
    {
      mju_warning("Mismatch between receive_data_vec [%zd] and receive_buffer.buffer_double.size [%zd]\n", receive_data_vec.size(), receive_buffer.buffer_double.size);
      return;
    }

    calculate_odom_velocities(odom_velocities, m_, d_);

    for (size_t i = 0; i < receive_buffer.buffer_double.size; i++)
    {
      *receive_data_vec[i] = receive_buffer.buffer_double.data[i];
    }
  }

} // namespace mujoco::plugin::multiverse_connector
