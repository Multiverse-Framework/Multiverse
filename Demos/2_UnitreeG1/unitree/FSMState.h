#pragma once

#include "Types.h"
#include "param.h"
#include "FSM/BaseState.h"
#include "isaaclab/devices/keyboard/keyboard.h"
#include "unitree_joystick_dsl.hpp"

#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

extern "C" {
#include "dds/dds.h"
#include "Twist.h"   // generated from the working ROS2-compatible DDS IDL
}

class FSMState : public BaseState {
 public:
  FSMState(int state, std::string state_string) : BaseState(state, state_string) {
    spdlog::info("Initializing State_{} ...", state_string);

    _ensureCmdVelSubscriber();

    auto transitions = param::config["FSM"][state_string]["transitions"];
    if (transitions) {
      auto transition_map =
          transitions.as<std::unordered_map<std::string, std::string>>();
      for (auto it = transition_map.begin(); it != transition_map.end(); ++it) {
        std::string target_fsm = it->first;
        if (!FSMStringMap.right.count(target_fsm)) {
          spdlog::warn("FSM State_'{}' not found in FSMStringMap!", target_fsm);
          continue;
        }

        int fsm_id = FSMStringMap.right.at(target_fsm);
        std::string condition = it->second;

        unitree::common::dsl::Parser p(condition);
        auto ast = p.Parse();
        auto func = unitree::common::dsl::Compile(*ast);

        registered_checks.emplace_back(
            std::make_pair(
                [func]() -> bool { return func(FSMState::lowstate->joystick); },
                fsm_id));
      }
    }

    registered_checks.emplace_back(
        std::make_pair(
            []() -> bool { return lowstate->isTimeout(); },
            FSMStringMap.right.at("Passive")));
  }

  void pre_run() override {
    lowstate->update();

    if (keyboard) {
      keyboard->update();
    }

    _updateJoystickInputs();
  }

  void post_run() override {
    lowcmd->unlockAndPublish();
  }

  // Keep the exact repo type names
  static std::unique_ptr<LowCmd_t> lowcmd;
  static std::shared_ptr<LowState_t> lowstate;
  static std::shared_ptr<Keyboard> keyboard;

 private:
  template <typename TKey>
  static void _applyKey(TKey& key, bool pressed, bool on_pressed,
                        bool on_released, float dt) {
    key.pressed = pressed;
    key.on_pressed = on_pressed;
    key.on_released = on_released;

    if (pressed) {
      key.pressed_time += dt;
    } else {
      key.pressed_time = 0.0f;
    }
  }

  static float _clamp(float v, float lo = -1.0f, float hi = 1.0f) {
    return std::max(lo, std::min(hi, v));
  }

  static void _ensureCmdVelSubscriber() {
    bool expected = false;
    if (!cmdvel_started.compare_exchange_strong(expected, true)) {
      return;
    }

    cmdvel_thread = std::thread([]() {
      participant = dds_create_participant(DDS_DOMAIN_DEFAULT, nullptr, nullptr);
      if (participant < 0) {
        spdlog::error("Failed to create DDS participant for /cmd_vel: {}",
                      dds_strretcode(-participant));
        return;
      }

      topic = dds_create_topic(
          participant,
          &geometry_msgs_msg_dds__Twist__desc,
          "rt/cmd_vel",
          nullptr,
          nullptr);
      if (topic < 0) {
        spdlog::error("Failed to create DDS topic rt/cmd_vel: {}",
                      dds_strretcode(-topic));
        dds_delete(participant);
        participant = DDS_RETCODE_ERROR;
        return;
      }

      dds_qos_t* qos = dds_create_qos();
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(10));
      dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 10);

      reader = dds_create_reader(participant, topic, qos, nullptr);
      dds_delete_qos(qos);

      if (reader < 0) {
        spdlog::error("Failed to create DDS reader for rt/cmd_vel: {}",
                      dds_strretcode(-reader));
        dds_delete(participant);
        participant = DDS_RETCODE_ERROR;
        return;
      }

      spdlog::info("Subscribed to DDS topic rt/cmd_vel");

      while (!cmdvel_stop.load()) {
        void* samples[1] = {nullptr};
        dds_sample_info_t infos[1];

        const int rc = dds_take(reader, samples, infos, 1, 1);
        if (rc < 0) {
          spdlog::warn("dds_take(rt/cmd_vel) failed: {}", dds_strretcode(-rc));
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
          continue;
        }

        if (rc > 0 && infos[0].valid_data && samples[0] != nullptr) {
          auto* msg = static_cast<geometry_msgs_msg_dds__Twist_*>(samples[0]);

          cmd_vel_lx.store(_clamp(static_cast<float>(msg->linear.y)));
          cmd_vel_ly.store(_clamp(static_cast<float>(msg->linear.x)));
          cmd_vel_rx.store(_clamp(static_cast<float>(-msg->angular.z)));
          cmd_vel_stamp.store(std::chrono::steady_clock::now().time_since_epoch().count());
          
          geometry_msgs_msg_dds__Twist__free(samples[0], DDS_FREE_ALL);
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      }

      if (participant >= 0) {
        dds_delete(participant);
      }
    });

    cmdvel_thread.detach();
  }

  static bool _cmdVelFresh() {
    constexpr auto timeout_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::milliseconds(300))
            .count();

    const auto now_ns =
        std::chrono::steady_clock::now().time_since_epoch().count();
    const auto stamp = cmd_vel_stamp.load();
    return stamp > 0 && (now_ns - stamp) <= timeout_ns;
  }

  static void _updateJoystickInputs() {
    auto& joy = lowstate->joystick;
    constexpr float dt = 0.02f;

    if (keyboard) {
      _applyKey(joy.LT,    keyboard->pressed("q"),     keyboard->on_pressed_key("q"),     keyboard->on_released_key("q"),     dt);
      _applyKey(joy.RT,    keyboard->pressed("e"),     keyboard->on_pressed_key("e"),     keyboard->on_released_key("e"),     dt);
      _applyKey(joy.RB,    keyboard->pressed("r"),     keyboard->on_pressed_key("r"),     keyboard->on_released_key("r"),     dt);

      _applyKey(joy.A,     keyboard->pressed("j"),     keyboard->on_pressed_key("j"),     keyboard->on_released_key("j"),     dt);
      _applyKey(joy.B,     keyboard->pressed("k"),     keyboard->on_pressed_key("k"),     keyboard->on_released_key("k"),     dt);
      _applyKey(joy.X,     keyboard->pressed("u"),     keyboard->on_pressed_key("u"),     keyboard->on_released_key("u"),     dt);
      _applyKey(joy.Y,     keyboard->pressed("i"),     keyboard->on_pressed_key("i"),     keyboard->on_released_key("i"),     dt);

      _applyKey(joy.up,    keyboard->pressed("up"),    keyboard->on_pressed_key("up"),    keyboard->on_released_key("up"),    dt);
      _applyKey(joy.down,  keyboard->pressed("down"),  keyboard->on_pressed_key("down"),  keyboard->on_released_key("down"),  dt);
      _applyKey(joy.left,  keyboard->pressed("left"),  keyboard->on_pressed_key("left"),  keyboard->on_released_key("left"),  dt);
      _applyKey(joy.right, keyboard->pressed("right"), keyboard->on_pressed_key("right"), keyboard->on_released_key("right"), dt);

      _applyKey(joy.start, keyboard->pressed("1"),     keyboard->on_pressed_key("1"),     keyboard->on_released_key("1"),     dt);
      _applyKey(joy.back,  keyboard->pressed("2"),     keyboard->on_pressed_key("2"),     keyboard->on_released_key("2"),     dt);
    }

    if (_cmdVelFresh()) {
      joy.lx(cmd_vel_lx.load());  // linear.x
      joy.ly(cmd_vel_ly.load());  // linear.y
      joy.rx(cmd_vel_rx.load());  // angular.z
      return;
    }

    // float ly_cmd = 0.0f;
    // float lx_cmd = 0.0f;
    // float rx_cmd = 0.0f;

    // if (keyboard) {
    //   if (keyboard->pressed("w")) ly_cmd += 1.0f;
    //   if (keyboard->pressed("s")) ly_cmd -= 1.0f;

    //   if (keyboard->pressed("d")) lx_cmd += 1.0f;
    //   if (keyboard->pressed("a")) lx_cmd -= 1.0f;

    //   if (keyboard->pressed("left"))  rx_cmd -= 1.0f;
    //   if (keyboard->pressed("right")) rx_cmd += 1.0f;
    // }

    // joy.ly(ly_cmd);
    // joy.lx(lx_cmd);
    // joy.rx(rx_cmd);
  }

 private:
  inline static std::atomic<bool> cmdvel_started{false};
  inline static std::atomic<bool> cmdvel_stop{false};

  inline static std::atomic<float> cmd_vel_lx{0.0f};
  inline static std::atomic<float> cmd_vel_ly{0.0f};
  inline static std::atomic<float> cmd_vel_rx{0.0f};
  inline static std::atomic<long long> cmd_vel_stamp{0};

  inline static std::thread cmdvel_thread{};

  inline static dds_entity_t participant{DDS_RETCODE_ERROR};
  inline static dds_entity_t topic{DDS_RETCODE_ERROR};
  inline static dds_entity_t reader{DDS_RETCODE_ERROR};
};