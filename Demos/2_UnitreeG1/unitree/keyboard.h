#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <iostream>
#include <linux/input.h>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <unistd.h>

/**
 * @brief Maintain a keyboard reading thread using Linux evdev.
 * Tracks multiple simultaneous keys and exposes pressed/on_pressed/on_released.
 */
class Keyboard {
 public:
  Keyboard() {
    _fd = _openKeyboardDevice();
    if (_fd < 0) {
      throw std::runtime_error(
          "Keyboard: failed to open any /dev/input/event* keyboard device. "
          "Run with proper permissions or set KEYBOARD_EVENT_DEVICE.");
    }

    _thread_running = true;
    _readThread = std::thread([this] {
      while (_thread_running) {
        _read();
      }
    });
  }

  ~Keyboard() {
    _thread_running = false;
    if (_readThread.joinable()) {
      _readThread.join();
    }
    if (_fd >= 0) {
      close(_fd);
      _fd = -1;
    }
  }

  void update() {
    std::lock_guard<std::mutex> lock(_mutex);

    _on_pressed.clear();
    _on_released.clear();

    // compute edge events
    for (const auto& k : _pressed_now) {
      if (_pressed_last.count(k) == 0) {
        _on_pressed.insert(k);
      }
    }
    for (const auto& k : _pressed_last) {
      if (_pressed_now.count(k) == 0) {
        _on_released.insert(k);
      }
    }

    _pressed_last = _pressed_now;

    // preserve old interface behavior as "latest active key"
    if (_pressed_now.empty()) {
      _key.clear();
    } else {
      _key = *_pressed_now.begin();
    }

    on_pressed = !_on_pressed.empty();
    on_released = !_on_released.empty();
  }

  /**
   * @brief Get one active key, mainly for backward compatibility.
   */
  std::string key() const {
    std::lock_guard<std::mutex> lock(_mutex);
    return _key;
  }

  /**
   * @brief True while the named key is held.
   */
  bool pressed(const std::string& name) const {
    std::lock_guard<std::mutex> lock(_mutex);
    return _pressed_now.count(_normalize(name)) > 0;
  }

  /**
   * @brief True only on the frame the named key is pressed.
   */
  bool on_pressed_key(const std::string& name) const {
    std::lock_guard<std::mutex> lock(_mutex);
    return _on_pressed.count(_normalize(name)) > 0;
  }

  /**
   * @brief True only on the frame the named key is released.
   */
  bool on_released_key(const std::string& name) const {
    std::lock_guard<std::mutex> lock(_mutex);
    return _on_released.count(_normalize(name)) > 0;
  }

  /**
   * @brief Get string input from stdin, preserved for compatibility.
   */
  std::string getString(std::string slogan) {
    std::cout << slogan << std::endl;
    std::string stringtemp;
    std::getline(std::cin, stringtemp);
    return stringtemp;
  }

  /**
   * flags; available after update()
   */
  bool on_pressed = false;
  bool on_released = false;

 private:
  static std::string _normalize(const std::string& s) {
    std::string out = s;
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
  }

  static std::string _keyCodeToName(__u16 code) {
    switch (code) {
      case KEY_W: return "w";
      case KEY_A: return "a";
      case KEY_S: return "s";
      case KEY_D: return "d";
      case KEY_Q: return "q";
      case KEY_E: return "e";
      case KEY_R: return "r";
      case KEY_J: return "j";
      case KEY_K: return "k";
      case KEY_U: return "u";
      case KEY_I: return "i";
      case KEY_1: return "1";
      case KEY_2: return "2";
      case KEY_UP: return "up";
      case KEY_DOWN: return "down";
      case KEY_LEFT: return "left";
      case KEY_RIGHT: return "right";
      default: return "";
    }
  }

  static bool _hasKeyboardNameHint(const std::string& name) {
    std::string n = _normalize(name);
    return n.find("keyboard") != std::string::npos ||
           n.find("usb keyboard") != std::string::npos ||
           n.find("logitech") != std::string::npos ||
           n.find("razer") != std::string::npos;
  }

  static int _openKeyboardDevice() {
    const char* env = std::getenv("KEYBOARD_EVENT_DEVICE");
    if (env && env[0] != '\0') {
      int fd = open(env, O_RDONLY | O_NONBLOCK);
      if (fd >= 0) return fd;
    }

    DIR* dir = opendir("/dev/input");
    if (!dir) return -1;

    std::vector<std::string> candidates;
    struct dirent* ent = nullptr;
    while ((ent = readdir(dir)) != nullptr) {
      if (std::strncmp(ent->d_name, "event", 5) == 0) {
        candidates.emplace_back(std::string("/dev/input/") + ent->d_name);
      }
    }
    closedir(dir);
    std::sort(candidates.begin(), candidates.end());

    for (const auto& path : candidates) {
      int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK);
      if (fd < 0) continue;

      char name[256] = {0};
      if (ioctl(fd, EVIOCGNAME(sizeof(name)), name) >= 0) {
        if (_hasKeyboardNameHint(name)) {
          return fd;
        }
      }
      close(fd);
    }

    return -1;
  }

  void _read() {
    if (_fd < 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      return;
    }

    input_event ev;
    ssize_t n = read(_fd, &ev, sizeof(ev));
    if (n == static_cast<ssize_t>(sizeof(ev))) {
      if (ev.type == EV_KEY) {
        std::string name = _keyCodeToName(ev.code);
        if (!name.empty()) {
          std::lock_guard<std::mutex> lock(_mutex);
          if (ev.value == 1 || ev.value == 2) {
            _pressed_now.insert(name);
          } else if (ev.value == 0) {
            _pressed_now.erase(name);
          }
        }
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  mutable std::mutex _mutex;
  bool _thread_running = false;
  std::thread _readThread;
  int _fd = -1;

  std::set<std::string> _pressed_now;
  std::set<std::string> _pressed_last;
  std::set<std::string> _on_pressed;
  std::set<std::string> _on_released;

  std::string _key;
};