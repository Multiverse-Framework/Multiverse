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

#ifndef MUJOCO_PLUGIN_MULTIVERSE_CONNECTOR_H_
#define MUJOCO_PLUGIN_MULTIVERSE_CONNECTOR_H_

#include "multiverse_client_json.h"

#include <set>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>

namespace mujoco::plugin::multiverse_connector
{
  struct MultiverseConfig
  {
    std::string host = "tcp://127.0.0.1";
    std::string server_port = "7000";
    std::string client_port = "7500";
    std::string world_name = "world";
    std::string simulation_name = "mujoco_simulation";
    std::map<std::string, std::set<std::string>> send_objects = {};
    std::map<std::string, std::set<std::string>> receive_objects = {};
  };

  // An multiverse_connector plugin which implements configurable MULTIVERSE_CONNECTOR control.
  class MultiverseConnector : public MultiverseClientJson
  {
  public:
    // Returns an instance of MultiverseConnector. The result can be null in case of
    // misconfiguration.
    static MultiverseConnector *Create(const mjModel *m, mjData *d, int instance);

    // Returns the number of state variables for the plugin instance
    static int StateSize(const mjModel *m, int instance);

    // Resets the C++ MultiverseConnector instance's state.
    // plugin_state is a C array pointer into mjData->plugin_state, with a size
    // equal to the value returned from StateSize.
    void Reset(mjtNum *plugin_state);

    // Idempotent computation which updates d->actuator_force and the internal
    // state of the class. Called after ActDot.
    void Compute(const mjModel *m, mjData *d, int instance);

    // Updates plugin state.
    void Advance(const mjModel *m, mjData *d, int instance) const;

    // Adds the MULTIVERSE_CONNECTOR plugin to the global registry of MuJoCo plugins.
    static void RegisterPlugin();

  private:
    MultiverseConnector(MultiverseConfig config, const mjModel *m, mjData *d);

    MultiverseConfig config_;

  private:
    mjModel *m_ = nullptr;

    mjData *d_ = nullptr;

    std::vector<mjtNum*> send_data_vec;

    std::vector<mjtNum*> receive_data_vec;

    std::map<int, mjtNum *> contact_efforts;

    std::map<int, mjtNum *> odom_velocities;

  private:
    void start_connect_to_server_thread() override;

    void wait_for_connect_to_server_thread_finish() override;

    void start_meta_data_thread() override;

    void wait_for_meta_data_thread_finish() override;

    bool init_objects(bool from_request_meta_data = false) override;

    void bind_request_meta_data() override;

    void bind_api_callbacks() override;

    void bind_api_callbacks_response() override;

    void bind_response_meta_data() override;

    void init_send_and_receive_data() override;

    void bind_send_data() override;

    void bind_receive_data() override;

    void clean_up() override;

    void reset() override;
  };

} // namespace mujoco::plugin::multiverse_connector

#endif // MUJOCO_PLUGIN_MULTIVERSE_CONNECTOR_H_
