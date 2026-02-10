#include <sys/wait.h>
#include <spawn.h>
#include <filesystem>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "vr_teleop_interfaces/action/teleop.hpp"
#include "vr_teleop_action/visibility_control.h"
#include "multiverse_client/multiverse_client_json.h"

namespace vr_teleop_action
{
  using Publisher = rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr;

  struct MultiverseConfig
  {
    std::string transport = "zmq";
    std::string host = "127.0.0.1";
    std::string server_port = "7000";
    std::string client_port = "2000";
    std::string world_name = "world";
    std::string simulation_name = "vr_teleop";
    std::map<std::string, std::string> send_objects = {};
    std::map<std::string, std::string> receive_objects = {};
  };

  struct JointState
  {
    double *position;
    double *velocity;
  };

  struct JointCommand
  {
    double *position;
    double *velocity;
  };

  struct PublisherCfg
  {
    Publisher publisher;
    int rate = 100;
    std::vector<std::string> joints;
  };

  class VrTeleopActionServer : public rclcpp::Node, public MultiverseClientJson
  {
  private:
    void start_connect_to_server_thread() override
    {
      connect_to_server();
    }

    void wait_for_connect_to_server_thread_finish() override
    {
    }

    void start_meta_data_thread() override
    {
      send_and_receive_meta_data();
    }

    void wait_for_meta_data_thread_finish()
    {
    }

    bool init_objects(bool)
    {
      return true;
    }

    void bind_request_meta_data() override
    {
      request_meta_data_str = request_meta_data_json.toStyledString();
    }

    void bind_api_callbacks() override
    {
    }

    void bind_api_callbacks_response() override
    {
    }

    void bind_response_meta_data() override
    {
    }

    void init_send_and_receive_data() override
    {
    }

    void bind_send_data() override
    {
      *world_time = get_time_now() - start_time;
    }

    void bind_receive_data() override
    {
    }

    void clean_up() override
    {
    }

    void reset() override
    {
    }

  private:
    void parse_config()
    {
      struct TmpPublisherCfg
      {
        std::string topic;
        int rate = 100;
        std::vector<std::string> joints;
      };

      publishers.clear();
      config.send_objects.clear();
      config.receive_objects.clear();
      std::map<std::string, TmpPublisherCfg> tmp;

      rcl_interfaces::msg::ListParametersResult listed = list_parameters({"publishers"}, 64);
      for (const std::string &name : listed.names)
      {
        if (name.rfind("publishers.", 0) != 0)
        {
          continue;
        }

        // Remove "publishers."
        std::string rest = name.substr(11);
        size_t dot = rest.find('.');
        if (dot == std::string::npos)
        {
          continue;
        }

        std::string pub = rest.substr(0, dot);
        std::string tail = rest.substr(dot + 1);

        if (tail == "topic")
        {
          get_parameter(name, tmp[pub].topic);
        }
        else if (tail == "rate")
        {
          get_parameter(name, tmp[pub].rate);
        }
        else if (tail.rfind("joints.", 0) == 0 &&
                 tail.size() > 13 &&
                 tail.substr(tail.size() - 5) == ".from")
        {
          // tail = joints.<joint>.from
          const std::string ros_joint = tail.substr(7, tail.size() - 12);

          std::string mujoco_joint;
          if (!get_parameter(name, mujoco_joint) || mujoco_joint.empty())
          {
            throw std::runtime_error("Missing parameter: " + name);
          }

          tmp[pub].joints.push_back(ros_joint);
          joint_state_map[ros_joint] = mujoco_joint;

          if (joint_commands.find(ros_joint) == joint_commands.end())
          {
            joint_commands[ros_joint] = JointCommand{new double[1]{0.0}, new double[1]{0.0}};
          }
          if (joint_positions.find(ros_joint) == joint_positions.end())
          {
            joint_positions[ros_joint] = JointState{new double[1]{0.0}, new double[1]{0.0}};
          }

          std::string type;
          if (!get_parameter("publishers." + pub + ".joints." + ros_joint + ".type", type) || type.empty())
          {
            throw std::runtime_error("Missing parameter: publishers." + pub + ".joints." + ros_joint + ".type");
          }
          if (type != "cmd_joint_angular_position" && type != "cmd_joint_linear_position")
          {
            throw std::runtime_error("Invalid joint command type (must be 'cmd_joint_angular_position' or 'cmd_joint_linear_position'): publishers." + pub + ".joints." + ros_joint + ".type");
          }
          config.send_objects[ros_joint] = type.substr(4);  // remove "cmd_"
          config.receive_objects[mujoco_joint] = type;
        }
      }

      // Finalize publishers
      publishers.reserve(tmp.size());
      for (auto &[pub_name, t] : tmp)
      {
        if (t.topic.empty())
        {
          throw std::runtime_error("Missing publishers." + pub_name + ".topic");
        }

        if (t.rate <= 0)
        {
          throw std::runtime_error("Missing/invalid publishers." + pub_name + ".rate");
        }

        if (t.joints.empty())
        {
          throw std::runtime_error("No joints for publisher: " + pub_name);
        }

        PublisherCfg cfg;
        cfg.publisher = this->template create_publisher<std_msgs::msg::Float64MultiArray>(t.topic, 10);
        cfg.rate = t.rate;
        cfg.joints = std::move(t.joints);

        publishers.push_back(std::move(cfg));
      }

      if (publishers.empty())
      {
        RCLCPP_WARN(get_logger(), "No publishers found in configuration under 'publishers'.");
      }
    }

  private:
    MultiverseConfig config;
    double start_time = 0.0;
    std::vector<PublisherCfg> publishers;
    std::map<std::string, JointState> joint_positions;
    std::map<std::string, JointCommand> joint_commands;
    std::map<std::string, std::string> joint_state_map;

  public:
    using Teleop = vr_teleop_interfaces::action::Teleop;
    using TeleopGoal = Teleop::Goal;
    using TeleopFeedback = Teleop::Feedback;
    using TeleopResult = Teleop::Result;
    using TeleopGoalHandle = rclcpp_action::ServerGoalHandle<Teleop>;

    VR_TELEOP_ACTION_PUBLIC
    explicit VrTeleopActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : rclcpp::Node(
              "vr_teleop_action_server",
              rclcpp::NodeOptions(options)
                  .allow_undeclared_parameters(true)
                  .automatically_declare_parameters_from_overrides(true))
    {
      using namespace std::placeholders;

      // Declare parameters
      if (!this->get_parameter("sim_path", sim_path) || sim_path.empty())
      {
        throw std::runtime_error("Missing required parameter: sim_path");
      }

      if (!this->get_parameter("model_path", model_path) || model_path.empty())
      {
        throw std::runtime_error("Missing required parameter: model_path");
      }

      // Convert to absolute paths
      std::filesystem::path sim_fs = std::filesystem::absolute(sim_path);
      std::filesystem::path model_fs = std::filesystem::absolute(model_path);

      // Optional: resolve symlinks and normalize
      try
      {
        sim_fs = std::filesystem::canonical(sim_fs);
      }
      catch (...)
      {
      }

      try
      {
        model_fs = std::filesystem::canonical(model_fs);
      }
      catch (...)
      {
      }

      // Store back as strings
      sim_path = sim_fs.string();
      model_path = model_fs.string();
      if (!std::filesystem::exists(sim_path))
      {
        throw std::runtime_error("sim_path does not exist: " + sim_path);
      }
      if (!std::filesystem::exists(model_path))
      {
        throw std::runtime_error("model_path does not exist: " + model_path);
      }
      RCLCPP_INFO(get_logger(), "Sim path: %s", sim_path.c_str());
      RCLCPP_INFO(get_logger(), "Model path: %s", model_path.c_str());

      parse_config();

      action_server_ = rclcpp_action::create_server<Teleop>(
          this,
          "teleop",
          std::bind(&VrTeleopActionServer::handle_goal, this, _1, _2),
          std::bind(&VrTeleopActionServer::handle_cancel, this, _1),
          std::bind(&VrTeleopActionServer::handle_accepted, this, _1));

      rcl_interfaces::msg::ListParametersResult listed = list_parameters({"meta_data"}, 64);
      for (const std::string &name : listed.names)
      {
        if (name.rfind("meta_data.", 0) != 0)
        {
          continue;
        }

        // Remove "meta_data."
        std::string key = name.substr(10);
        std::string value;
        if (!get_parameter(name, value) || value.empty())
        {
          throw std::runtime_error("Missing parameter: " + name);
        }
        if (key == "transport")
        {
          if (value == "tcp")
          {
            set_transport(ClientTransportType::Tcp);
          }
          else if (value == "udp")
          {
            set_transport(ClientTransportType::Udp);
          }
          else if (value == "zmq")
          {
            set_transport(ClientTransportType::Zmq);
          }
          else
          {
            throw std::runtime_error("Invalid transport type: " + value);
          }
          config.transport = value;
        }
        else if (key == "host")
        {
          config.host = value;
        }
        else if (key == "server_port")
        {
          config.server_port = value;
        }
        else if (key == "client_port")
        {
          config.client_port = value;
        }
        else if (key == "world_name")
        {
          config.world_name = value;
        }
        else if (key == "simulation_name")
        {
          config.simulation_name = value;
        }
      }

      host = config.host;
      server_port = config.server_port;
      client_port = config.client_port;

      request_meta_data_json.clear();
      request_meta_data_json["meta_data"]["world_name"] = config.world_name;
      request_meta_data_json["meta_data"]["simulation_name"] = config.simulation_name;
      request_meta_data_json["meta_data"]["length_unit"] = "m";
      request_meta_data_json["meta_data"]["angle_unit"] = "rad";
      request_meta_data_json["meta_data"]["mass_unit"] = "kg";
      request_meta_data_json["meta_data"]["time_unit"] = "s";
      request_meta_data_json["meta_data"]["handedness"] = "rhs";
      for (const auto &ros_joint : config.send_objects)
      {
        request_meta_data_json["send"][ros_joint.first] = ros_joint.second;
      }

      *world_time = 0.0;

      start_time = get_time_now();

      RCLCPP_INFO(get_logger(), "Multiverse Server: %s:%s - Multiverse Client: %s:%s\n", host.c_str(), server_port.c_str(), host.c_str(),
                  client_port.c_str());

      connect();

      communicate(true);

      communicate();

      RCLCPP_INFO(get_logger(), "Action server ready");
    }

  private:
    rclcpp_action::Server<Teleop>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const TeleopGoal> goal)
    {
      if (goal_running.load())
      {
        RCLCPP_WARN(get_logger(), "Rejecting goal: another goal is running");
        return rclcpp_action::GoalResponse::REJECT;
      }
      RCLCPP_INFO(get_logger(), "Received goal request with duration %.3f sec", (static_cast<double>(goal->timeout.sec) + goal->timeout.nanosec * 1e-9));
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<TeleopGoalHandle> goal_handle)
    {
      RCLCPP_INFO(get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<TeleopGoalHandle> goal_handle)
    {
      goal_running.store(true);
      using namespace std::placeholders;
      // this needs to return quickly to avoid blocking the executor, so spin up a new thread
      std::thread{std::bind(&VrTeleopActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<TeleopGoalHandle> goal_handle)
    {
      RCLCPP_INFO(get_logger(), "Executing goal");
      pid = -1;
      message.clear();
      sim_starting.store(false);
      sim_started.store(false);
      sim_start_failed.store(false);

      rclcpp::Rate loop_rate(1000);
      const std::shared_ptr<const TeleopGoal> goal = goal_handle->get_goal();
      const std::shared_ptr<TeleopFeedback> feedback = std::make_shared<TeleopFeedback>();
      feedback->state = TeleopFeedback::SYNCHRONIZING;
      std::shared_ptr<TeleopResult> result;

      while (rclcpp::ok())
      {
        if (is_process_dead())
        {
          feedback->state = TeleopFeedback::ERROR;
          message = "Simulator process died unexpectedly";
        }
        if (goal_handle->is_canceling())
        {
          RCLCPP_WARN(get_logger(), "Goal is canceling");
          result = std::make_shared<TeleopResult>();
          result->return_code = TeleopResult::CANCELED;
          break;
        }

        if (feedback->state == TeleopFeedback::ERROR)
        {
          handling_error();
          result = std::make_shared<TeleopResult>();
          result->return_code = TeleopResult::FAILED;
          result->message = message;
        }

        if (!rclcpp::ok() || result)
        {
          break;
        }

        goal_handle->publish_feedback(feedback);

        switch (feedback->state)
        {
        case TeleopFeedback::SYNCHRONIZING:
          feedback->state = synchronizing();
          if (feedback->state != TeleopFeedback::SYNCHRONIZING &&
              feedback->state != TeleopFeedback::WAITING_FOR_OPERATOR &&
              feedback->state != TeleopFeedback::ERROR)
          {
            feedback->state = TeleopFeedback::ERROR;
            message = "[SYNCHRONIZING] Wrong state";
          }
          break;

        case TeleopFeedback::WAITING_FOR_OPERATOR:
          if (is_time_out())
          {
            result = std::make_shared<TeleopResult>();
            result->return_code = TeleopResult::TIMEOUT;
          }
          else if (is_aborted())
          {
            result = std::make_shared<TeleopResult>();
            result->return_code = TeleopResult::CANCELED;
          }
          else
          {
            feedback->state = waiting_for_operator();
          }
          if (feedback->state != TeleopFeedback::WAITING_FOR_OPERATOR &&
              feedback->state != TeleopFeedback::COMPLETING &&
              feedback->state != TeleopFeedback::ACTIVE &&
              feedback->state != TeleopFeedback::ERROR)
          {
            feedback->state = TeleopFeedback::ERROR;
            message = "[WAITING_FOR_OPERATOR] Wrong state";
          }
          break;

        case TeleopFeedback::COMPLETING:
          feedback->state = completing();
          result = std::make_shared<TeleopResult>();
          result->return_code = TeleopResult::DONE;
          if (feedback->state != TeleopFeedback::COMPLETING &&
              feedback->state != TeleopFeedback::ERROR)
          {
            feedback->state = TeleopFeedback::ERROR;
            message = "[COMPLETING] Wrong state";
          }
          break;

        case TeleopFeedback::ACTIVE:
          feedback->state = active();
          if (is_time_out())
          {
            result = std::make_shared<TeleopResult>();
            result->return_code = TeleopResult::TIMEOUT;
          }
          if (is_aborted())
          {
            result = std::make_shared<TeleopResult>();
            result->return_code = TeleopResult::CANCELED;
          }
          if (feedback->state != TeleopFeedback::ACTIVE &&
              feedback->state != TeleopFeedback::COMPLETING &&
              feedback->state != TeleopFeedback::WAITING_FOR_OPERATOR &&
              feedback->state != TeleopFeedback::ERROR)
          {
            feedback->state = TeleopFeedback::ERROR;
            message = "[ACTIVE] Wrong state";
          }
          break;
        }
        loop_rate.sleep();
      }

      // If ROS is not OK, force failure
      if (!rclcpp::ok())
      {
        result = std::make_shared<TeleopResult>();
        result->return_code = TeleopResult::FAILED;
        result->message = "ROS shutdown detected";
        pid = -1;
      }
      if (pid > 0)
      {
        kill(pid, SIGTERM);
        waitpid(pid, nullptr, 0);
        pid = -1;
      }

      // Ensure result exists
      if (!result)
      {
        result = std::make_shared<TeleopResult>();
        result->return_code = TeleopResult::FAILED;
        result->message = "result is nullptr";
      }

      if (rclcpp::ok())
      {
        // Send final action result
        switch (result->return_code)
        {
        case TeleopResult::DONE:
          goal_handle->succeed(result);
          RCLCPP_INFO(get_logger(), "Goal succeeded");
          break;

        case TeleopResult::CANCELED:
          goal_handle->canceled(result);
          RCLCPP_WARN(get_logger(), "Goal canceled");
          break;

        default:
          goal_handle->abort(result);
          RCLCPP_ERROR(get_logger(), "Goal aborted");
          break;
        }
      }

      goal_running.store(false);
    }

    uint8_t synchronizing()
    {
      // 1) If we already know startup failed, report ERROR
      if (sim_start_failed.load())
      {
        if (message.empty())
        {
          message = "[SYNCHRONIZING] simulate start failed";
        }
        return TeleopFeedback::ERROR;
      }

      // 2) Start the process ONCE in a background thread
      //    - Return SYNCHRONIZING while it is starting / not yet started.
      if (!sim_starting.exchange(true)) // first caller flips false->true
      {
        sim_start_time = std::chrono::steady_clock::now();
        // Launch thread only once
        sim_thread = std::thread([this]()
                                 {
          // argv for posix_spawn
          char *argv[] = {
            const_cast<char *>(sim_path.c_str()),
            const_cast<char *>(model_path.c_str()),
            nullptr
          };

          pid_t child_pid = -1;
          int rc = posix_spawn(&child_pid, sim_path.c_str(), nullptr, nullptr, argv, environ);

          if (rc != 0)
          {
            // posix_spawn failed
            {
              std::lock_guard<std::mutex> lk(sim_mutex);
              message = std::string("[SYNCHRONIZING] posix_spawn failed: ") + std::strerror(rc);
            }
            sim_start_failed.store(true);
            sim_started.store(false);
            return;
          }

          // Process started successfully
          pid = child_pid;               // store PID for later kill/waitpid
          sim_started.store(true); });

        sim_thread.detach();
      }

      // 3) While process is not confirmed started, stay in SYNCHRONIZING
      if (!sim_started.load())
      {
        return TeleopFeedback::SYNCHRONIZING;
      }

      // 4) Process started. If your criteria is met, move on.
      if (is_synchronized())
      {
        start_time = get_time_now();
        return TeleopFeedback::WAITING_FOR_OPERATOR;
      }

      // Otherwise keep synchronizing until criteria becomes true
      return TeleopFeedback::SYNCHRONIZING;
    }

    uint8_t waiting_for_operator()
    {
      if (is_done())
      {
        return TeleopFeedback::COMPLETING;
      }
      if (is_trigger())
      {
        return TeleopFeedback::ACTIVE;
      }
      return TeleopFeedback::WAITING_FOR_OPERATOR;
    }

    uint8_t completing()
    {
      return TeleopFeedback::COMPLETING;
    }

    uint8_t active()
    {
      if (is_trigger())
      {
        for (const auto &pub_cfg : publishers)
        {
          std_msgs::msg::Float64MultiArray msg;
          msg.data.reserve(pub_cfg.joints.size());
          for (const std::string &ros_joint : pub_cfg.joints)
          {
            const JointCommand &state = joint_commands[ros_joint];
            msg.data.push_back(*state.position);
          }
          pub_cfg.publisher->publish(msg);
        }
        return TeleopFeedback::ACTIVE;
      }
      else
      {
        return TeleopFeedback::WAITING_FOR_OPERATOR;
      }
    }

    void handling_error()
    {
      RCLCPP_ERROR(get_logger(), message.c_str());
    }

    bool is_time_out()
    {
      return false;
    }

    bool is_aborted()
    {
      return false;
    }

    bool is_synchronized() const
    {
      return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - sim_start_time).count() >= 3;
    }

    bool is_trigger() const
    {
      int count = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - sim_start_time).count();
      return (count >= 6 && count <= 7) || (count >= 9 && count <= 10) || (count >= 12 && count <= 13);
    }

    bool is_done() const
    {
      return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - sim_start_time).count() >= 20;
    }

    bool is_process_dead()
    {
      if (pid <= 0)
      {
        return false;
      }

      int status = 0;
      pid_t result = waitpid(pid, &status, WNOHANG);

      if (result == 0)
      {
        // still running
        return false;
      }
      else if (result == pid)
      {
        // process exited
        pid = -1;
        return true;
      }
      else
      {
        // error
        return false;
      }
    }

  private:
    std::string sim_path;
    std::string model_path;

    pid_t pid = -1;
    std::string message;

    std::atomic_bool goal_running{false};
    std::atomic_bool sim_starting{false};
    std::atomic_bool sim_started{false};
    std::atomic_bool sim_start_failed{false};

    std::mutex sim_mutex;
    std::thread sim_thread;

    std::chrono::steady_clock::time_point sim_start_time;
  }; // class VrTeleopActionServer

} // namespace vr_teleop_action

RCLCPP_COMPONENTS_REGISTER_NODE(vr_teleop_action::VrTeleopActionServer)