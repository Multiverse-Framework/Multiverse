#pragma once

#include "Types.h"
#include "param.h"
#include "FSM/BaseState.h"
#include "isaaclab/devices/keyboard/keyboard.h"
#include "unitree_joystick_dsl.hpp"

class FSMState : public BaseState {
 public:
  FSMState(int state, std::string state_string) : BaseState(state, state_string) {
    spdlog::info("Initializing State_{} ...", state_string);

    auto transitions = param::config["FSM"][state_string]["transitions"];
    if (transitions) {
      auto transition_map = transitions.as<std::unordered_map<std::string, std::string>>();
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
      _updateJoystickFromKeyboard();
    }
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

  static void _updateJoystickFromKeyboard() {
    auto& joy = lowstate->joystick;

    // Adjust if your real control loop dt is different
    constexpr float dt = 0.02f;

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

    // ===== velocity control (analog axes) =====
    // W/S -> lx, A/D -> ly, Left/Right -> rx

    float ly_cmd = 0.0f;
    if (keyboard->pressed("w")) ly_cmd += 1.0f;
    if (keyboard->pressed("s")) ly_cmd -= 1.0f;
    joy.ly(ly_cmd);

    float lx_cmd = 0.0f;
    if (keyboard->pressed("d")) lx_cmd += 1.0f;
    if (keyboard->pressed("a")) lx_cmd -= 1.0f;
    joy.lx(lx_cmd);

    float rx_cmd = 0.0f;
    if (keyboard->pressed("left"))  rx_cmd -= 1.0f;
    if (keyboard->pressed("right")) rx_cmd += 1.0f;
    joy.rx(rx_cmd);
  }
};