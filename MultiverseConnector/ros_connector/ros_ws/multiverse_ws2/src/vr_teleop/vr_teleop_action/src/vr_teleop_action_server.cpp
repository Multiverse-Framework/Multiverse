#include <sys/wait.h>
#include <spawn.h>
#include <filesystem>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "vr_teleop_interfaces/action/teleop.hpp"
#include "vr_teleop_action/visibility_control.h"
#include "multiverse_client/multiverse_client_json.h"

extern char **environ;

namespace vr_teleop_action
{
  using PublisherF64 = rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr;
  using PublisherJT = rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr;
  using Subscriber = rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr;

  static inline bool starts_with(const std::string &s, const std::string &p) { return s.rfind(p, 0) == 0; }
  static inline bool ends_with(const std::string &s, const std::string &p)
  {
    return s.size() >= p.size() && s.compare(s.size() - p.size(), p.size(), p) == 0;
  }

  enum class SignalType
  {
    JointAngularPos,
    JointLinearPos,
    Position,
    Quaternion,
    Unknown
  };

  static inline SignalType parse_signal_type(const std::string &s)
  {
    if (s == "joint_angular_position")
      return SignalType::JointAngularPos;
    if (s == "joint_linear_position")
      return SignalType::JointLinearPos;
    if (s == "position")
      return SignalType::Position;
    if (s == "quaternion")
      return SignalType::Quaternion;
    return SignalType::Unknown;
  }

  static inline const char *to_string(SignalType t)
  {
    switch (t)
    {
    case SignalType::JointAngularPos:
      return "joint_angular_position";
    case SignalType::JointLinearPos:
      return "joint_linear_position";
    case SignalType::Position:
      return "position";
    case SignalType::Quaternion:
      return "quaternion";
    default:
      return "unknown";
    }
  }

  struct SignalRef
  {
    double value = 0.0;
    double *ptr = &value;
    double get() const { return *ptr; }
    void set(double v) { *ptr = v; }
    void bind(double *p) { ptr = (p ? p : &value); }
  };

  // Resolve JointState indices once, then apply fast.
  struct JointStateIndexCache
  {
    bool resolved = false;
    std::vector<std::string> names;
    std::vector<int> indices;

    explicit JointStateIndexCache(std::vector<std::string> requested = {})
        : names(std::move(requested)), indices(names.size(), -1) {}

    void resolve(const sensor_msgs::msg::JointState &msg, rclcpp::Logger logger, const std::string &topic)
    {
      std::unordered_map<std::string, int> idx;
      idx.reserve(msg.name.size());
      for (int i = 0; i < (int)msg.name.size(); ++i)
        idx.emplace(msg.name[i], i);

      for (size_t k = 0; k < names.size(); ++k)
      {
        auto it = idx.find(names[k]);
        if (it != idx.end())
          indices[k] = it->second;
        else
          RCLCPP_WARN(logger, "JointState on '%s' missing joint '%s'", topic.c_str(), names[k].c_str());
      }
      resolved = true;
    }

    template <class F>
    void apply(const sensor_msgs::msg::JointState &, F &&f) const
    {
      for (size_t k = 0; k < names.size(); ++k)
      {
        const int i = indices[k];
        if (i < 0)
          continue;
        f(k, i);
      }
    }
  };

  struct MultiverseConfig
  {
    std::string transport = "zmq";
    std::string host = "127.0.0.1";
    std::string server_port = "7000";
    std::string client_port = "2000";
    std::string world_name = "world";
    std::string simulation_name = "vr_teleop";

    std::unordered_map<std::string, SignalType> send_objects;                   // ros_joint -> type
    std::unordered_map<std::string, SignalType> receive_objects;                // mujoco_joint -> type
    std::unordered_map<std::string, geometry_msgs::msg::Transform> init_frames; // ref_frame -> transform
    std::unordered_map<std::string, SignalType> init_joints;                    // teleop_joint -> type (init_phase only)
  };

  struct JointState
  {
    SignalRef position;
    SignalRef velocity;
  };

  struct JointCommand
  {
    SignalRef position;
    SignalRef velocity;

    SignalRef filtered_position;
  };

  enum class PubMsgType
  {
    Float64MultiArray,
    JointTrajectory
  };

  struct PublisherCfg
  {
    PubMsgType msg_type = PubMsgType::Float64MultiArray;

    // one of these is used
    PublisherF64 pub_f64;
    PublisherJT pub_jt;

    std::string topic;
    int rate = 100;
    std::vector<std::string> actuators;       // already reordered to controller joint order
    std::vector<std::string> ros_joint_order; // controller "joints" (ROS joint names in order)

    // messages
    std_msgs::msg::Float64MultiArray f64_msg;
    trajectory_msgs::msg::JointTrajectory jt_msg;
    trajectory_msgs::msg::JointTrajectoryPoint jt_point;

    // rate-guard state
    double period_s = 0.01;        // 1/rate
    double next_pub_time_s = 0.0;  // schedule based on "now"
    double last_warn_time_s = 0.0; // throttle warnings
    double max_lag_s = 0.25;       // if we lag more than this, warn
  };

  struct SubscriberCfg
  {
    Subscriber subscriber;
    std::string topic;
    std::vector<std::string> joints;
  };

  static inline bool is_joint_traj_stream_topic(const std::string &topic)
  {
    // Common ROS2 joint_trajectory_controller topic interface:
    if (ends_with(topic, "/joint_trajectory"))
      return true;

    // Some people still use /command naming:
    if (ends_with(topic, "/command") || ends_with(topic, "/commands"))
      return true;

    // Nonstandard but requested:
    if (ends_with(topic, "/follow_joint_trajectory/command"))
      return true;

    return false;
  }

  static inline std::string controller_ns_from_topic(const std::string &topic)
  {
    const std::string s1 = "/follow_joint_trajectory/command";
    const std::string s2 = "/follow_joint_trajectory/goal";
    const std::string s3 = "/follow_joint_trajectory";
    const std::string s4 = "/joint_trajectory";
    const std::string s5 = "/commands";
    const std::string s6 = "/command";

    if (ends_with(topic, s1))
      return topic.substr(0, topic.size() - s1.size());
    if (ends_with(topic, s2))
      return topic.substr(0, topic.size() - s2.size());
    if (ends_with(topic, s3))
      return topic.substr(0, topic.size() - s3.size());
    if (ends_with(topic, s4))
      return topic.substr(0, topic.size() - s4.size());
    if (ends_with(topic, s5))
      return topic.substr(0, topic.size() - s5.size());
    if (ends_with(topic, s6))
      return topic.substr(0, topic.size() - s6.size());
    return topic;
  }

  class VrTeleopActionServer : public rclcpp::Node, public MultiverseClientJson
  {
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
      parse_config();

      using namespace std::placeholders;
      action_server_ = rclcpp_action::create_server<Teleop>(
          this,
          "teleop",
          std::bind(&VrTeleopActionServer::handle_goal, this, _1, _2),
          std::bind(&VrTeleopActionServer::handle_cancel, this, _1),
          std::bind(&VrTeleopActionServer::handle_accepted, this, _1));

      init_multiverse_client();

      RCLCPP_INFO(get_logger(), "Action server ready");
    }

    ~VrTeleopActionServer() override
    {
      if (js_init_timer_)
        js_init_timer_->cancel();
      js_init_sub_.reset();
      init_js_cache_sub_.reset();

      if (goal_thread_.joinable())
        goal_thread_.join();

      if (sim_thread.joinable())
        sim_thread.join();

      const pid_t p = pid.load(std::memory_order_acquire);
      if (p > 0)
      {
        ::kill(p, SIGTERM);
        ::waitpid(p, nullptr, 0);
        pid.store(-1, std::memory_order_release);
      }
    }

  private:
    // -------------------- MultiverseClientJson overrides --------------------
    void start_connect_to_server_thread() override { connect_to_server(); }
    void wait_for_connect_to_server_thread_finish() override {}
    void start_meta_data_thread() override { send_and_receive_meta_data(); }
    void wait_for_meta_data_thread_finish() override {}

    bool init_objects(bool) { return true; }

    void bind_request_meta_data() override
    {
      request_meta_data_json.clear();
      request_meta_data_json["meta_data"]["world_name"] = config.world_name;
      request_meta_data_json["meta_data"]["simulation_name"] = config.simulation_name;
      request_meta_data_json["meta_data"]["length_unit"] = "m";
      request_meta_data_json["meta_data"]["angle_unit"] = "rad";
      request_meta_data_json["meta_data"]["mass_unit"] = "kg";
      request_meta_data_json["meta_data"]["time_unit"] = "s";
      request_meta_data_json["meta_data"]["handedness"] = "rhs";

      for (const auto &kv : config.send_objects)
        request_meta_data_json["send"][kv.first].append(to_string(kv.second));

      for (const auto &kv : config.receive_objects)
        request_meta_data_json[init_phase ? "send" : "receive"][kv.first].append(to_string(kv.second));

      if (init_phase)
      {
        for (const auto &kv : config.init_joints)
        {
          const std::string &mujoco_jointuator = kv.first;
          const std::string type = to_string(kv.second);

          auto &a = request_meta_data_json["send"][mujoco_jointuator];
          if (std::none_of(a.begin(), a.end(), [&](const Json::Value &v)
                           { return v.asString() == type; }))
            a.append(type);
        }

        for (const auto &kv : config.init_frames)
        {
          const std::string &frame = kv.first;
          request_meta_data_json["send"][frame].append("position");
          request_meta_data_json["send"][frame].append("quaternion");
        }
      }

      request_meta_data_str = request_meta_data_json.toStyledString();
    }

    void bind_api_callbacks() override {}
    void bind_api_callbacks_response() override {}

    void bind_response_meta_data() override
    {
      ros_joint_pos_to_index.clear();
      mujoco_joint_pos_to_index.clear();
      mujoco_body_pos_to_index.clear();
      mujoco_body_quat_to_index.clear();

      if (init_phase)
      {
        size_t idx = 0;
        for (const std::string &send_object : response_meta_data_json["send"].getMemberNames())
        {
          for (const std::string &type : response_meta_data_json["send"][send_object].getMemberNames())
          {
            if (type == "position")
            {
              mujoco_body_pos_to_index[send_object] = idx;
              idx += 3;
            }
            else if (type == "quaternion")
            {
              mujoco_body_quat_to_index[send_object] = idx;
              idx += 4;
            }
            else if (type == "joint_angular_position" || type == "joint_linear_position")
            {
              mujoco_joint_pos_to_index[send_object] = idx;
              idx += 1;
            }
            else
            {
              throw std::runtime_error("Unsupported type for send object '" + send_object + "': " + type);
            }
          }
        }
      }
      else
      {
        size_t ros_idx = 0;
        for (const std::string &ros_joint : response_meta_data_json["send"].getMemberNames())
          ros_joint_pos_to_index[ros_joint] = ros_idx++;

        size_t act_idx = 0;
        for (const std::string &mujoco_jointuator : response_meta_data_json["receive"].getMemberNames())
          mujoco_joint_pos_to_index[mujoco_jointuator] = act_idx++;
      }
    }

    void init_send_and_receive_data() override
    {
      if (init_phase)
      {
        auto bind_act_to_send = [this](const std::string &act)
        {
          auto it = mujoco_joint_pos_to_index.find(act);
          if (it == mujoco_joint_pos_to_index.end())
            return;
          const size_t send_index = it->second;
          send_buffer.buffer_double.data[send_index] = joint_commands[act].position.get();
          joint_commands[act].position.bind(&send_buffer.buffer_double.data[send_index]);
        };

        for (auto &pub_cfg : publishers)
        {
          pub_cfg.next_pub_time_s = 0.0; // will be initialized on first publish tick
          pub_cfg.last_warn_time_s = 0.0;
          for (const auto &act : pub_cfg.actuators)
            bind_act_to_send(act);
        }

        for (const auto &kv : config.init_joints)
          bind_act_to_send(kv.first);

        auto bind_frame_to_send = [this](const std::string &frame)
        {
          auto it_pos = mujoco_body_pos_to_index.find(frame);
          auto it_quat = mujoco_body_quat_to_index.find(frame);
          if (it_pos == mujoco_body_pos_to_index.end() || it_quat == mujoco_body_quat_to_index.end())
            return;
          const size_t pos_index = it_pos->second;
          const size_t quat_index = it_quat->second;
          const auto &tr = config.init_frames[frame].translation;
          const auto &q = config.init_frames[frame].rotation;
          send_buffer.buffer_double.data[pos_index + 0] = tr.x;
          send_buffer.buffer_double.data[pos_index + 1] = tr.y;
          send_buffer.buffer_double.data[pos_index + 2] = tr.z;
          send_buffer.buffer_double.data[quat_index + 0] = q.w;
          send_buffer.buffer_double.data[quat_index + 1] = q.x;
          send_buffer.buffer_double.data[quat_index + 2] = q.y;
          send_buffer.buffer_double.data[quat_index + 3] = q.z;
        };

        for (const auto &kv : config.init_frames)
          bind_frame_to_send(kv.first);
      }
      else
      {
        for (auto &pub_cfg : publishers)
        {
          for (const auto &act : pub_cfg.actuators)
          {
            const size_t i = mujoco_joint_pos_to_index[act];
            joint_commands[act].position.bind(&receive_buffer.buffer_double.data[i]);
          }
        }

        for (auto &sub_cfg : subscribers)
        {
          for (const auto &mujoco_joint : sub_cfg.joints)
          {
            const auto it = mujoco_joint_to_ros_joint.find(mujoco_joint);
            if (it == mujoco_joint_to_ros_joint.end())
              continue;
            const std::string &ros_joint = it->second;

            const auto it2 = ros_joint_pos_to_index.find(ros_joint);
            if (it2 == ros_joint_pos_to_index.end())
              continue;
            const size_t i = it2->second;

            joint_states[ros_joint].position.bind(&send_buffer.buffer_double.data[i]);
          }
        }
      }
    }

    void bind_send_data() override { *world_time = get_time_now() - start_time; }
    void bind_receive_data() override {}
    void clean_up() override {}
    void reset() override {}

  private:
    // ---------------------------- TF refresh per execute() ----------------------------
    void refresh_marker_init_frames()
    {
      if (marker_ref_to_source_.empty())
        return;

      if (!tf_buffer_)
      {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      }

      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
      auto updated = config.init_frames; // keep previous unless successful

      while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
      {
        bool all_ok = true;

        for (const auto &kv : marker_ref_to_source_)
        {
          const std::string &ref_frame = kv.first;
          const std::string &source_frame = kv.second;

          try
          {
            const auto t = tf_buffer_->lookupTransform(marker_root_frame_, source_frame, tf2::TimePointZero);
            updated[ref_frame] = t.transform;
          }
          catch (const tf2::TransformException &)
          {
            all_ok = false;
          }
        }

        if (all_ok)
          break;

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }

      config.init_frames = std::move(updated);
      RCLCPP_INFO(get_logger(), "Refreshed init_frames: %zu frames", config.init_frames.size());
    }

    // Cached last JointState on js_init_topic_ (no temporary subscription).
    void refresh_joint_states()
    {
      if (init_teleop_to_source_ros_.empty())
        return;

      sensor_msgs::msg::JointState::SharedPtr last;
      {
        std::lock_guard<std::mutex> lk(init_js_cache_m_);
        last = init_js_cache_last_;
      }

      if (!last)
      {
        RCLCPP_WARN(get_logger(),
                    "refresh_joint_states: no cached JointState on %s (keeping previous)",
                    js_init_topic_.c_str());
        return;
      }

      std::vector<std::string> teleop;
      std::vector<std::string> source;
      teleop.reserve(init_teleop_to_source_ros_.size());
      source.reserve(init_teleop_to_source_ros_.size());
      for (const auto &kv : init_teleop_to_source_ros_)
      {
        teleop.push_back(kv.first);
        source.push_back(kv.second);
      }

      JointStateIndexCache cache(source);
      cache.resolve(*last, get_logger(), js_init_topic_);

      size_t updated = 0;
      cache.apply(*last, [&](size_t k, int i)
                  {
                    if (i >= 0 && i < (int)last->position.size())
                    {
                      joint_commands[teleop[k]].position.set(last->position[i]);
                      updated++;
                    } });

      RCLCPP_INFO(get_logger(), "Refreshed init actuators from cached %s: updated=%zu",
                  js_init_topic_.c_str(), updated);
    }

  private:
    // ---------------------------- JointState one-shot init (kept) ----------------------------
    // Keeps your old behavior: quickly initializes teleop joints once after startup (or on reload).
    void start_joint_state_initializer()
    {
      if (init_teleop_to_source_ros_.empty())
        return;

      if (js_init_timer_)
        js_init_timer_->cancel();
      js_init_sub_.reset();

      const auto start = std::chrono::steady_clock::now();
      const double timeout_sec = 2.0;

      auto done = std::make_shared<std::atomic_bool>(false);

      std::vector<std::string> teleop;
      std::vector<std::string> source;
      teleop.reserve(init_teleop_to_source_ros_.size());
      source.reserve(init_teleop_to_source_ros_.size());
      for (const auto &kv : init_teleop_to_source_ros_)
      {
        teleop.push_back(kv.first);
        source.push_back(kv.second);
      }

      auto teleop_list = std::make_shared<std::vector<std::string>>(std::move(teleop));
      auto cache = std::make_shared<JointStateIndexCache>(std::move(source));

      js_init_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(50),
          [this, start, timeout_sec, done]()
          {
            if (done->load())
            {
              if (js_init_timer_)
                js_init_timer_->cancel();
              return;
            }

            const double elapsed =
                std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - start).count();
            if (elapsed >= timeout_sec)
            {
              RCLCPP_WARN(get_logger(), "joint_state_initializer timeout after %.2fs, continuing", elapsed);
              done->store(true);
              js_init_sub_.reset();
              if (js_init_timer_)
                js_init_timer_->cancel();
            }
          });

      RCLCPP_INFO(get_logger(), "Creating joint_state_initializer subscriber on %s", js_init_topic_.c_str());

      js_init_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
          js_init_topic_, 10,
          [this, done, teleop_list, cache, topic = js_init_topic_](const sensor_msgs::msg::JointState::SharedPtr msg)
          {
            if (done->load())
              return;

            if (!cache->resolved)
              cache->resolve(*msg, get_logger(), topic);

            bool any = false;
            cache->apply(*msg, [&](size_t k, int i)
                         {
                           if (i >= 0 && i < (int)msg->position.size())
                           {
                             joint_commands[(*teleop_list)[k]].position.set(msg->position[i]);
                             any = true;
                           } });

            if (any)
            {
              done->store(true);
              js_init_sub_.reset();
              if (js_init_timer_)
                js_init_timer_->cancel();
              RCLCPP_INFO(get_logger(), "joint_state_initializer finished");
            }
          });
    }

  private:
    // ---------------------------- config parse ----------------------------
    void parse_config()
    {
      if (!this->get_parameter("sim_path", sim_path) || sim_path.empty())
        throw std::runtime_error("Missing required parameter: sim_path");
      if (!this->get_parameter("model_path", model_path) || model_path.empty())
        throw std::runtime_error("Missing required parameter: model_path");

      std::filesystem::path sim_fs = std::filesystem::absolute(sim_path);
      std::filesystem::path model_fs = std::filesystem::absolute(model_path);
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
      sim_path = sim_fs.string();
      model_path = model_fs.string();

      if (!std::filesystem::exists(sim_path))
        throw std::runtime_error("sim_path does not exist: " + sim_path);
      if (!std::filesystem::exists(model_path))
        throw std::runtime_error("model_path does not exist: " + model_path);

      RCLCPP_INFO(get_logger(), "Sim path: %s", sim_path.c_str());
      RCLCPP_INFO(get_logger(), "Model path: %s", model_path.c_str());

      {
        this->get_parameter_or("filter.time_constant", filter_time_constant_, filter_time_constant_);
        this->get_parameter_or("filter.damping", filter_damping_, filter_damping_);

        if (!(filter_time_constant_ >= 0.0))
        {
          throw std::runtime_error("filter.time_constant should be >= 0, got " + std::to_string(filter_time_constant_));
        }

        if (!(filter_damping_ >= 0.0))
        {
          throw std::runtime_error("filter.damping should be >= 0, got " + std::to_string(filter_damping_));
        }

        RCLCPP_INFO(get_logger(), "Filter params: time_constant=%.6f, damping=%.6f", filter_time_constant_, filter_damping_);
      }

      // reset
      publishers.clear();
      subscribers.clear();
      config.send_objects.clear();
      config.receive_objects.clear();
      config.init_frames.clear();
      config.init_joints.clear();

      marker_ref_to_source_.clear();
      init_teleop_to_source_ros_.clear();
      ros_joint_to_mujoco_joint.clear();
      mujoco_joint_to_ros_joint.clear();

      // -------------------- markers_initializer (parse only) --------------------
      {
        if (!get_parameter("initializers.markers_initializer.frame_id", marker_root_frame_) || marker_root_frame_.empty())
          marker_root_frame_ = "pelvis";

        auto listed = list_parameters({"initializers.markers_initializer.frames"}, 512);
        const std::string prefix = "initializers.markers_initializer.frames.";
        for (const auto &pname : listed.names)
        {
          if (!starts_with(pname, prefix))
            continue;

          const std::string ref_frame = pname.substr(prefix.size());
          std::string source_frame;
          if (!get_parameter(pname, source_frame) || source_frame.empty())
            continue;

          marker_ref_to_source_[ref_frame] = source_frame;
        }
      }

      // -------------------- joint_state_initializer (parse + start one-shot sub) --------------------
      {
        std::string topic;
        if (get_parameter("initializers.joint_state_initializer.topic", topic) && !topic.empty())
          js_init_topic_ = topic;
        else
          js_init_topic_ = "/joint_states";

        auto listed = list_parameters({"initializers.joint_state_initializer.joints"}, 2048);
        const std::string prefix = "initializers.joint_state_initializer.joints.";
        for (const auto &pname : listed.names)
        {
          if (!starts_with(pname, prefix))
            continue;

          const std::string teleop_joint = pname.substr(prefix.size());
          std::string source_ros_joint;
          if (!get_parameter(pname, source_ros_joint) || source_ros_joint.empty())
            continue;

          init_teleop_to_source_ros_[teleop_joint] = source_ros_joint;

          (void)joint_commands[teleop_joint];

          // send it during init_phase
          config.init_joints[teleop_joint] = SignalType::JointAngularPos;
        }

        // Cache subscriber (persistent) for later refresh in execute()
        init_js_cache_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            js_init_topic_, 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg)
            {
              std::lock_guard<std::mutex> lk(init_js_cache_m_);
              init_js_cache_last_ = msg;
            });

        start_joint_state_initializer();
      }

      // -------------------- parse publishers.* --------------------
      struct ActuatorPublisherCfg
      {
        std::string topic;
        int rate = 100;
        std::vector<std::string> actuators;
      };
      struct TR
      {
        std::string topic;
        int rate = 100;
      };

      std::unordered_map<std::string, ActuatorPublisherCfg> actuator_pub;
      std::unordered_map<std::string, TR> pub_tr;

      {
        auto listed = list_parameters({"publishers"}, 1024);

        for (const std::string &full : listed.names)
        {
          if (!starts_with(full, "publishers."))
            continue;

          const std::string rest = full.substr(11);
          const size_t dot = rest.find('.');
          if (dot == std::string::npos)
            continue;

          const std::string pub = rest.substr(0, dot);
          const std::string tail = rest.substr(dot + 1);

          if (tail == "topic")
          {
            std::string t;
            if (get_parameter(full, t) && !t.empty())
              pub_tr[pub].topic = t;
            continue;
          }
          if (tail == "rate")
          {
            int r = 0;
            if (get_parameter(full, r))
              pub_tr[pub].rate = r;
            continue;
          }

          if (starts_with(tail, "joints.") && ends_with(tail, ".from"))
          {
            const std::string ros_joint = tail.substr(7, tail.size() - 7 - 5);

            std::string mujoco_joint;
            if (!get_parameter(full, mujoco_joint) || mujoco_joint.empty())
              continue;

            actuator_pub[pub].actuators.push_back(mujoco_joint);
            ros_joint_to_mujoco_joint[ros_joint] = mujoco_joint;
            (void)joint_commands[mujoco_joint];

            std::string type_str;
            const std::string type_param = "publishers." + pub + ".joints." + ros_joint + ".type";
            if (!get_parameter(type_param, type_str) || type_str.empty())
              continue;

            const SignalType st = parse_signal_type(type_str);
            if (st != SignalType::JointAngularPos && st != SignalType::JointLinearPos)
              continue;

            config.receive_objects[mujoco_joint] = st;
          }
        }

        for (auto &kv : pub_tr)
        {
          const std::string &pub_name = kv.first;
          const TR &tr = kv.second;
          actuator_pub[pub_name].topic = tr.topic;
          actuator_pub[pub_name].rate = tr.rate;
        }
      }

      // -------------------- finalize publishers --------------------
      {
        publishers.reserve(actuator_pub.size());

        for (auto &kv : actuator_pub)
        {
          auto &t = kv.second;
          if (t.topic.empty() || t.actuators.empty())
            continue;

          const std::string controller_ns = controller_ns_from_topic(t.topic);
          const std::string get_parameters_topic = controller_ns + "/get_parameters";

          PublisherCfg cfg;
          cfg.topic = t.topic;
          cfg.rate = t.rate;
          cfg.actuators = t.actuators;

          cfg.msg_type = is_joint_traj_stream_topic(t.topic) ? PubMsgType::JointTrajectory : PubMsgType::Float64MultiArray;

          if (cfg.msg_type == PubMsgType::JointTrajectory)
            cfg.pub_jt = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(t.topic, 10);
          else
            cfg.pub_f64 = this->create_publisher<std_msgs::msg::Float64MultiArray>(t.topic, 10);

          // Try to reorder actuators according to controller's "joints" parameter order.
          auto client = create_client<rcl_interfaces::srv::GetParameters>(get_parameters_topic);
          while (!client->wait_for_service(std::chrono::seconds(2)))
          {
            RCLCPP_WARN(get_logger(),
                        "Waiting for service %s ...",
                        get_parameters_topic.c_str());
          }

          auto req = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
          req->names = {"joints"};
          auto future = client->async_send_request(req);
          while (rclcpp::ok())
          {
            const auto rc = rclcpp::spin_until_future_complete(
                get_node_base_interface(), future, std::chrono::seconds(2));

            if (rc == rclcpp::FutureReturnCode::SUCCESS)
              break;

            if (rc == rclcpp::FutureReturnCode::INTERRUPTED)
              return; // executor interrupted → early stop

            RCLCPP_WARN(get_logger(),
                        "Waiting for response from %s ...",
                        get_parameters_topic.c_str());
          }
          auto resp = future.get();
          if (!resp->values.empty() &&
              resp->values[0].type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY)
          {
            cfg.ros_joint_order = resp->values[0].string_array_value; // store controller order (ROS joint names)
            std::vector<std::string> ordered;
            ordered.reserve(t.actuators.size());

            for (const auto &ros_joint : cfg.ros_joint_order)
            {
              auto it = ros_joint_to_mujoco_joint.find(ros_joint);
              if (it == ros_joint_to_mujoco_joint.end())
                throw std::runtime_error("Controller " + get_parameters_topic + " returned unknown joint '" + ros_joint + "'");

              const std::string &act = it->second;
              if (std::find(t.actuators.begin(), t.actuators.end(), act) != t.actuators.end())
                ordered.push_back(act);
            }

            if (ordered.empty())
              throw std::runtime_error("Controller " + get_parameters_topic + " returned empty or incompatible joints list");
            cfg.actuators = std::move(ordered);
          }

          cfg.period_s = (cfg.rate > 0) ? (1.0 / static_cast<double>(cfg.rate)) : 0.0;
          cfg.next_pub_time_s = 0.0;
          cfg.last_warn_time_s = 0.0;
          cfg.max_lag_s = 0.25;
          if (cfg.msg_type == PubMsgType::Float64MultiArray)
          {
            cfg.f64_msg.data.resize(cfg.actuators.size(), 0.0);
          }
          else
          {
            // JointTrajectory stream: joint_names must be controller order.
            // actuators vector is already reordered to match this order.
            cfg.jt_msg.joint_names = cfg.ros_joint_order;

            cfg.jt_point.positions.resize(cfg.ros_joint_order.size(), 0.0);
            cfg.jt_point.time_from_start = rclcpp::Duration::from_seconds(std::max(0.02, cfg.period_s));
          }
          cfg.next_pub_time_s = 0.0; // will be initialized on first publish tick
          cfg.last_warn_time_s = 0.0;
          cfg.max_lag_s = 0.25; // you can tune this (e.g. 0.1)
          publishers.push_back(std::move(cfg));
        }
      }

      // -------------------- parse subscribers.* --------------------
      struct JointSubscriberCfg
      {
        std::string topic;
        std::vector<std::string> mujoco_joints;
        std::vector<std::string> ros_joints;
      };

      std::unordered_map<std::string, JointSubscriberCfg> joint_sub;

      {
        auto listed = list_parameters({"subscribers"}, 2048);

        for (const auto &full : listed.names)
        {
          if (!starts_with(full, "subscribers."))
            continue;

          const std::string rest = full.substr(12);
          const size_t dot = rest.find('.');
          if (dot == std::string::npos)
            continue;

          const std::string sub = rest.substr(0, dot);
          const std::string tail = rest.substr(dot + 1);

          if (tail == "topic")
          {
            get_parameter(full, joint_sub[sub].topic);
            continue;
          }

          if (starts_with(tail, "joints.") && ends_with(tail, ".to"))
          {
            const std::string ros_joint = tail.substr(7, tail.size() - 7 - 3);

            std::string mujoco_joint;
            if (!get_parameter(full, mujoco_joint) || mujoco_joint.empty())
              continue;

            joint_sub[sub].ros_joints.push_back(ros_joint);
            joint_sub[sub].mujoco_joints.push_back(mujoco_joint);

            mujoco_joint_to_ros_joint[mujoco_joint] = ros_joint;
            (void)joint_states[ros_joint];

            std::string type_str;
            const std::string type_param = "subscribers." + sub + ".joints." + ros_joint + ".type";
            if (!get_parameter(type_param, type_str) || type_str.empty())
              continue;

            const SignalType st = parse_signal_type(type_str);
            if (st != SignalType::JointAngularPos && st != SignalType::JointLinearPos)
              continue;

            config.send_objects[mujoco_joint] = st;
          }
        }
      }

      // -------------------- finalize subscribers (subscribe to topic, resolve indices once) --------------------
      {
        for (auto &kv : joint_sub)
        {
          auto &t = kv.second;
          if (t.topic.empty() || t.ros_joints.empty() || t.mujoco_joints.empty())
            continue;

          SubscriberCfg cfg;
          cfg.topic = t.topic;
          cfg.joints = t.mujoco_joints;

          auto ros_joints = std::make_shared<std::vector<std::string>>(t.ros_joints);
          auto cache = std::make_shared<JointStateIndexCache>(*ros_joints);

          cfg.subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
              t.topic, 10,
              [this, ros_joints, cache, topic = t.topic](const sensor_msgs::msg::JointState::SharedPtr msg)
              {
                if (!cache->resolved)
                  cache->resolve(*msg, get_logger(), topic);

                cache->apply(*msg, [&](size_t k, int i)
                             {
                               const auto &ros_joint = (*ros_joints)[k];
                               if (i < (int)msg->position.size())
                                 joint_states[ros_joint].position.set(msg->position[i]);
                               if (i < (int)msg->velocity.size())
                                 joint_states[ros_joint].velocity.set(msg->velocity[i]); });
              });

          subscribers.push_back(std::move(cfg));
        }
      }

      build_filter_actuator_list();
    }

    void init_multiverse_client()
    {
      auto listed = list_parameters({"meta_data"}, 64);
      for (const auto &name : listed.names)
      {
        if (!starts_with(name, "meta_data."))
          continue;

        const std::string key = name.substr(10);
        std::string value;
        if (!get_parameter(name, value) || value.empty())
          throw std::runtime_error("Missing parameter: " + name);

        if (key == "transport")
        {
          if (value == "tcp")
            set_transport(ClientTransportType::Tcp);
          else if (value == "udp")
            set_transport(ClientTransportType::Udp);
          else if (value == "zmq")
            set_transport(ClientTransportType::Zmq);
          else
            throw std::runtime_error("Invalid transport type: " + value);
          config.transport = value;
        }
        else if (key == "host")
          config.host = value;
        else if (key == "server_port")
          config.server_port = value;
        else if (key == "client_port")
          config.client_port = value;
        else if (key == "world_name")
          config.world_name = value;
        else if (key == "simulation_name")
          config.simulation_name = value;
      }

      host = config.host;
      server_port = config.server_port;
      client_port = config.client_port;

      *world_time = 0.0;
      start_time = get_time_now();

      RCLCPP_INFO(get_logger(), "Multiverse Server: %s:%s - Multiverse Client: %s:%s",
                  host.c_str(), server_port.c_str(), host.c_str(), client_port.c_str());

      connect();
    }

    void build_filter_actuator_list()
    {
      std::unordered_map<std::string, bool> seen;
      filter_actuators_.clear();

      for (const auto &pub_cfg : publishers)
      {
        for (const auto &act : pub_cfg.actuators)
        {
          if (seen.emplace(act, true).second)
            filter_actuators_.push_back(act);
        }
      }

      // allocate vectors to correct size
      y1_.assign(filter_actuators_.size(), 0.0);
      y2_.assign(filter_actuators_.size(), 0.0);
      out_.assign(filter_actuators_.size(), 0.0);

      filter_ready_ = false;

      RCLCPP_INFO(get_logger(), "Filter actuator list: %zu actuators", filter_actuators_.size());
    }

    void update_filtered_commands_second_order()
    {
      if (filter_actuators_.empty())
        return;

      const double T = filter_time_constant_;
      const double D = filter_damping_;
      const double T1 = T * T;
      const double T2 = 2.0 * D * T;

      const double current_time = get_time_now() - start_time;

      std::vector<double> u(filter_actuators_.size(), 0.0);
      for (size_t k = 0; k < filter_actuators_.size(); ++k)
        u[k] = joint_commands[filter_actuators_[k]].position.get();

      // First-time init: y1=y2=send_data=u, and set times
      if (!filter_ready_)
      {
        out_ = u;
        y1_ = out_;
        y2_ = out_;

        t2_ = current_time;
        t1_ = current_time;

        // write filtered_position output
        for (size_t k = 0; k < filter_actuators_.size(); ++k)
          joint_commands[filter_actuators_[k]].filtered_position.set(out_[k]);

        filter_ready_ = true;
        return;
      }

      const double delta_T_1 = current_time - t1_;
      const double delta_T_2 = t1_ - t2_;

      // Safety guards (avoid division by zero / negative dt)
      if (delta_T_1 <= 1e-9 || delta_T_2 <= 1e-9 || T <= 1e-9)
      {
        // fall back to pass-through
        out_ = u;
      }
      else
      {
        const double a = (delta_T_1 * delta_T_2) / T1;

        const double b1 = (-T2 / delta_T_1 + T1 / (delta_T_1 * delta_T_1) + T1 / (delta_T_1 * delta_T_2));
        const double b2 = (-1.0 + T2 / delta_T_1 - T1 / (delta_T_1 * delta_T_1));

        for (size_t k = 0; k < out_.size(); ++k)
        {
          out_[k] = a * (u[k] + b1 * y1_[k] + b2 * y2_[k]);
        }
      }

      // Output to filtered_position
      for (size_t k = 0; k < filter_actuators_.size(); ++k)
        joint_commands[filter_actuators_[k]].filtered_position.set(out_[k]);

      t2_ = t1_;
      t1_ = current_time;
      y2_ = y1_;
      y1_ = out_;
    }

  private:
    rclcpp_action::Server<Teleop>::SharedPtr action_server_;

    // Avoid detach: keep a joinable goal thread.
    std::thread goal_thread_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const TeleopGoal> goal)
    {
      (void)uuid;
      if (goal_running.load())
      {
        RCLCPP_WARN(get_logger(), "Rejecting goal: another goal is running");
        return rclcpp_action::GoalResponse::REJECT;
      }
      timeout_s = static_cast<double>(goal->timeout.sec) + static_cast<double>(goal->timeout.nanosec) * 1e-9;
      RCLCPP_INFO(get_logger(), "Received goal request with duration %.3f sec", timeout_s);
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<TeleopGoalHandle> goal_handle)
    {
      (void)goal_handle;
      RCLCPP_INFO(get_logger(), "Received request to cancel goal");
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<TeleopGoalHandle> goal_handle)
    {
      goal_running.store(true);

      if (goal_thread_.joinable())
        goal_thread_.join();

      goal_thread_ = std::thread([this, goal_handle]()
                                 { execute(goal_handle); });
    }

    void run_phase(bool phase)
    {
      init_phase = phase;
      communicate(true);
      communicate();
    }

    void publish_filtered_with_rate_guard(double now_s)
    {
      for (auto &pub_cfg : publishers)
      {
        if (pub_cfg.rate <= 0 || pub_cfg.period_s <= 0.0)
          continue;

        // first time: start schedule from now
        if (pub_cfg.next_pub_time_s <= 0.0)
          pub_cfg.next_pub_time_s = now_s;

        // not time yet
        if (now_s < pub_cfg.next_pub_time_s)
          continue;

        // We are due (or late). Detect lag and warn if we can't keep up.
        const double lag_s = now_s - pub_cfg.next_pub_time_s;

        if (lag_s > pub_cfg.max_lag_s)
        {
          // throttle warnings (once per second per publisher)
          if ((now_s - pub_cfg.last_warn_time_s) > 1.0)
          {
            const double eff_hz = (lag_s > 1e-9) ? (1.0 / lag_s) : 0.0;
            RCLCPP_WARN(get_logger(),
                        "Publisher '%s' cannot keep up: target=%d Hz (period=%.4fs), lag=%.4fs (effective ~%.1f Hz).",
                        pub_cfg.topic.c_str(),
                        pub_cfg.rate,
                        pub_cfg.period_s,
                        lag_s,
                        eff_hz);
            pub_cfg.last_warn_time_s = now_s;
          }

          // If we are very late, resync schedule to now to avoid publishing bursts.
          pub_cfg.next_pub_time_s = now_s;
        }

        // Fill & publish once
        if (pub_cfg.msg_type == PubMsgType::Float64MultiArray)
        {
          auto &data = pub_cfg.f64_msg.data;
          for (size_t i = 0; i < pub_cfg.actuators.size(); ++i)
          {
            const std::string &act = pub_cfg.actuators[i];
            data[i] = joint_commands[act].filtered_position.get();
          }
          pub_cfg.pub_f64->publish(pub_cfg.f64_msg);
        }
        else
        {
          // Fill one point in controller order.
          // pub_cfg.actuators is already reordered to match pub_cfg.ros_joint_order.
          for (size_t i = 0; i < pub_cfg.actuators.size(); ++i)
          {
            const std::string &act = pub_cfg.actuators[i];
            pub_cfg.jt_point.positions[i] = joint_commands[act].filtered_position.get();
          }

          pub_cfg.jt_point.time_from_start =
              rclcpp::Duration::from_seconds(std::max(0.02, pub_cfg.period_s));

          pub_cfg.jt_msg.header.stamp = now(); // node clock
          pub_cfg.jt_msg.points.clear();
          pub_cfg.jt_msg.points.push_back(pub_cfg.jt_point);

          pub_cfg.pub_jt->publish(pub_cfg.jt_msg);
        }

        // Advance schedule by exactly 1 period (no drift accumulation)
        pub_cfg.next_pub_time_s += pub_cfg.period_s;

        // If loop is extremely slow and we are still behind, skip forward (no burst)
        if (pub_cfg.next_pub_time_s < now_s - pub_cfg.period_s)
          pub_cfg.next_pub_time_s = now_s;
      }
    }

    void execute(const std::shared_ptr<TeleopGoalHandle> goal_handle)
    {
      refresh_marker_init_frames();
      refresh_joint_states();

      run_phase(true);
      run_phase(false);

      RCLCPP_INFO(get_logger(), "Executing goal");

      pid.store(-1, std::memory_order_release);
      sim_starting.store(false);
      sim_started.store(false);
      sim_start_failed.store(false);

      rclcpp::Rate loop_rate(1000);

      const auto feedback = std::make_shared<TeleopFeedback>();
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
          result = std::make_shared<TeleopResult>();
          result->return_code = TeleopResult::CANCELED;
          break;
        }

        if (feedback->state == TeleopFeedback::ERROR)
        {
          result = std::make_shared<TeleopResult>();
          result->return_code = TeleopResult::FAILED;
          result->message = message;
        }

        if (!rclcpp::ok() || result)
          break;

        goal_handle->publish_feedback(feedback);

        switch (feedback->state)
        {
        case TeleopFeedback::SYNCHRONIZING:
          feedback->state = synchronizing();
          break;
        case TeleopFeedback::WAITING_FOR_OPERATOR:
          if (is_timeout())
          {
            result = std::make_shared<TeleopResult>();
            result->return_code = TeleopResult::TIMEOUT;
            result->message = "Timeout reached";
            break;
          }
          feedback->state = waiting_for_operator();
          break;
        case TeleopFeedback::ACTIVE:
          if (is_timeout())
          {
            result = std::make_shared<TeleopResult>();
            result->return_code = TeleopResult::TIMEOUT;
            result->message = "Timeout reached";
            break;
          }
          feedback->state = active();
          break;
        case TeleopFeedback::COMPLETING:
          result = std::make_shared<TeleopResult>();
          result->return_code = TeleopResult::DONE;
          break;
        default:
          feedback->state = TeleopFeedback::ERROR;
          message = "Unknown state";
          break;
        }

        communicate();
        update_filtered_commands_second_order();
        loop_rate.sleep();
      }

      const pid_t p = pid.load(std::memory_order_acquire);
      if (p > 0)
      {
        ::kill(p, SIGTERM);
        ::waitpid(p, nullptr, 0);
        pid.store(-1, std::memory_order_release);
      }

      if (!result)
      {
        result = std::make_shared<TeleopResult>();
        result->return_code = TeleopResult::FAILED;
        result->message = "result is nullptr";
      }

      result->duration = rclcpp::Duration::from_seconds(std::chrono::duration<double>(get_time_now() - start_time).count());

      if (rclcpp::ok())
      {
        if (result->return_code == TeleopResult::DONE)
          goal_handle->succeed(result);
        else if (result->return_code == TeleopResult::CANCELED)
          goal_handle->canceled(result);
        else
          goal_handle->abort(result);
      }

      goal_running.store(false);
    }

    uint8_t synchronizing()
    {
      if (!sim_starting.exchange(true))
      {
        if (sim_thread.joinable())
          sim_thread.join();

        sim_thread = std::thread([this]()
                                 {
                                   char *argv[] = {
                                       const_cast<char *>(sim_path.c_str()),
                                       const_cast<char *>(model_path.c_str()),
                                       nullptr};

                                   pid_t child_pid = -1;
                                   const int rc = ::posix_spawn(&child_pid, sim_path.c_str(), nullptr, nullptr, argv, environ);
                                   if (rc != 0)
                                   {
                                     message = std::string("posix_spawn failed: ") + std::strerror(rc);
                                     sim_start_failed.store(true);
                                     sim_started.store(false);
                                     return;
                                   }
                                   pid.store(child_pid, std::memory_order_release);
                                   sim_started.store(true); });
      }

      if (sim_start_failed.load())
        return TeleopFeedback::ERROR;

      if (!sim_started.load())
        return TeleopFeedback::SYNCHRONIZING;

      if (is_synchronized())
      {
        start_time = get_time_now();
        RCLCPP_INFO(get_logger(), "Synchronized with simulator, starting teleoperation at t=%.3f", start_time);
        return TeleopFeedback::WAITING_FOR_OPERATOR;
      }

      return TeleopFeedback::SYNCHRONIZING;
    }

    uint8_t waiting_for_operator()
    {
      if (is_done())
        return TeleopFeedback::COMPLETING;
      if (is_trigger())
        return TeleopFeedback::ACTIVE;
      return TeleopFeedback::WAITING_FOR_OPERATOR;
    }

    uint8_t active()
    {
      if (!is_trigger())
        return TeleopFeedback::WAITING_FOR_OPERATOR;

      publish_filtered_with_rate_guard(get_time_now() - start_time);

      return TeleopFeedback::ACTIVE;
    }

    bool is_timeout() const
    {
      return timeout_s < 0.0 ? false : get_time_now() - start_time >= timeout_s;
    }

    bool is_synchronized() const
    {
      constexpr double tol = 1e-3;

      for (const auto &kv : mujoco_joint_to_ros_joint)
      {
        const std::string &ros_joint = kv.second;
        if (ros_joint_to_mujoco_joint.find(ros_joint) == ros_joint_to_mujoco_joint.end())
        {
          continue;
        }
        const std::string mujoco_joint = ros_joint_to_mujoco_joint.at(ros_joint);

        auto it_cmd = joint_commands.find(mujoco_joint);
        auto it_state = joint_states.find(ros_joint);

        if (it_cmd == joint_commands.end() || it_state == joint_states.end())
        {
          RCLCPP_WARN(get_logger(),
                      "Not synchronized: missing joint command or state for %s (ROS joint: %s)",
                      mujoco_joint.c_str(),
                      ros_joint.c_str());
          return false;
        }

        const double cmd = it_cmd->second.position.get();
        const double state = it_state->second.position.get();
        const double err = std::abs(cmd - state);

        if (err > tol)
        {
          RCLCPP_WARN(get_logger(),
                      "Not synchronized: %s vs %s, err=%.6f",
                      mujoco_joint.c_str(),
                      ros_joint.c_str(),
                      err);
          return false;
        }
      }

      return true;
    }

    bool is_trigger() const { return true; }
    bool is_done() const { return false; }

    bool is_process_dead()
    {
      const pid_t p = pid.load(std::memory_order_acquire);
      if (p <= 0)
        return false;

      int status = 0;
      const pid_t r = ::waitpid(p, &status, WNOHANG);
      if (r == 0)
        return false;
      if (r == p)
      {
        pid.store(-1, std::memory_order_release);
        return true;
      }
      return false;
    }

  private:
    // ---------------------------- filter params/state ----------------------------
    double filter_time_constant_ = 0.02;
    double filter_damping_ = 1.0;

    std::vector<std::string> filter_actuators_;

    std::vector<double> y1_;
    std::vector<double> y2_;
    std::vector<double> out_;
    double t1_ = 0.0;
    double t2_ = 0.0;
    bool filter_ready_ = false;

    // ---------------------------- TF state ----------------------------
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unordered_map<std::string, std::string> marker_ref_to_source_; // ref_frame -> source_frame
    std::string marker_root_frame_{"pelvis"};

    // ---------------------------- joint_state_initializer state ----------------------------
    rclcpp::TimerBase::SharedPtr js_init_timer_;
    Subscriber js_init_sub_;
    std::string js_init_topic_{"/joint_states"};
    std::unordered_map<std::string, std::string> init_teleop_to_source_ros_; // teleop -> source

    // persistent cache for later refresh in execute()
    Subscriber init_js_cache_sub_;
    std::mutex init_js_cache_m_;
    sensor_msgs::msg::JointState::SharedPtr init_js_cache_last_;

    // ---------------------------- core state ----------------------------
    MultiverseConfig config;

    double start_time = 0.0;
    double timeout_s = -1.0;

    std::vector<PublisherCfg> publishers;
    std::vector<SubscriberCfg> subscribers;

    std::unordered_map<std::string, JointState> joint_states;
    std::unordered_map<std::string, JointCommand> joint_commands;

    std::unordered_map<std::string, std::string> ros_joint_to_mujoco_joint;
    std::unordered_map<std::string, std::string> mujoco_joint_to_ros_joint;

    std::unordered_map<std::string, size_t> ros_joint_pos_to_index;
    std::unordered_map<std::string, size_t> mujoco_joint_pos_to_index;
    std::unordered_map<std::string, size_t> mujoco_body_pos_to_index;
    std::unordered_map<std::string, size_t> mujoco_body_quat_to_index;

    std::string sim_path;
    std::string model_path;
    bool init_phase = true;

    std::atomic<pid_t> pid{-1};
    std::thread sim_thread;

    std::atomic_bool goal_running{false};
    std::atomic_bool sim_starting{false};
    std::atomic_bool sim_started{false};
    std::atomic_bool sim_start_failed{false};

    std::string message;
  };

} // namespace vr_teleop_action

RCLCPP_COMPONENTS_REGISTER_NODE(vr_teleop_action::VrTeleopActionServer)
