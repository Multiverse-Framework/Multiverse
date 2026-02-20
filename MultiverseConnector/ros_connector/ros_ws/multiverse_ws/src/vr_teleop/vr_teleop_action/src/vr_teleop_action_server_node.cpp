#include <sys/wait.h>
#include <spawn.h>
#include <filesystem>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/Transform.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>

#include <thread>

#include "vr_teleop_msgs/TeleopAction.h"
#include "multiverse_client/multiverse_client_json.h"

extern char **environ;

namespace vr_teleop_action
{
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

    void resolve(const sensor_msgs::JointState &msg, const std::string &topic)
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
          ROS_WARN("JointState on '%s' missing joint '%s'", topic.c_str(), names[k].c_str());
      }
      resolved = true;
    }

    template <class F>
    void apply(const sensor_msgs::JointState &, F &&f) const
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

    std::unordered_map<std::string, SignalType> send_objects;              // ros_joint -> type
    std::unordered_map<std::string, SignalType> receive_objects;           // mujoco_joint -> type
    std::unordered_map<std::string, geometry_msgs::Transform> init_frames; // ref_frame -> transform
    std::unordered_map<std::string, SignalType> init_joints;               // teleop_joint -> type (init_phase only)
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

  struct PublisherCfg
  {
    ros::Publisher publisher;
    std::string topic;
    int rate = 100;
    std::vector<std::string> actuators;
    std_msgs::Float64MultiArray msg;

    // rate-guard state
    double period_s = 0.01;        // 1/rate
    double next_pub_time_s = 0.0;  // schedule based on "now"
    double last_warn_time_s = 0.0; // throttle warnings
    double max_lag_s = 0.25;       // if we lag more than this, warn
  };

  struct SubscriberCfg
  {
    ros::Subscriber subscriber;
    std::string topic;
    std::vector<std::string> joints;
  };

  // ---------------------------- ROS1 param helpers (XmlRpc traversal) ----------------------------
  static bool xmlrpc_has_member(const XmlRpc::XmlRpcValue &v, const std::string &k)
  {
    return v.getType() == XmlRpc::XmlRpcValue::TypeStruct && v.hasMember(k);
  }

  static std::vector<std::string> xmlrpc_struct_keys(const XmlRpc::XmlRpcValue &v)
  {
    std::vector<std::string> out;
    if (v.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      return out;
    out.reserve((size_t)v.size());
    for (auto it = v.begin(); it != v.end(); ++it)
      out.push_back(it->first);
    return out;
  }

  static bool xmlrpc_get_string(const XmlRpc::XmlRpcValue &v, std::string &out)
  {
    if (v.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      out = static_cast<std::string>(v);
      return true;
    }
    return false;
  }

  static bool xmlrpc_get_int(const XmlRpc::XmlRpcValue &v, int &out)
  {
    if (v.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      out = (int)v;
      return true;
    }
    return false;
  }

  static bool xmlrpc_get_string_array(const XmlRpc::XmlRpcValue &v, std::vector<std::string> &out)
  {
    if (v.getType() != XmlRpc::XmlRpcValue::TypeArray)
      return false;
    out.clear();
    out.reserve((size_t)v.size());
    for (int i = 0; i < v.size(); ++i)
    {
      if (v[i].getType() != XmlRpc::XmlRpcValue::TypeString)
        return false;
      out.push_back(static_cast<std::string>(v[i]));
    }
    return true;
  }

  class VrTeleopActionServer : public MultiverseClientJson
  {
  public:
    using TeleopAction = vr_teleop_msgs::TeleopAction;
    using TeleopGoal = vr_teleop_msgs::TeleopGoal;
    using TeleopFeedback = vr_teleop_msgs::TeleopFeedback;
    using TeleopResult = vr_teleop_msgs::TeleopResult;

    explicit VrTeleopActionServer(ros::NodeHandle nh, ros::NodeHandle pnh)
        : nh_(std::move(nh)),
          pnh_(std::move(pnh)),
          as_(nh_, "vr_teleop_action_server", false)
    {
      parse_config();
      init_multiverse_client();

      as_.registerGoalCallback(std::bind(&VrTeleopActionServer::on_goal, this));
      as_.registerPreemptCallback(std::bind(&VrTeleopActionServer::on_preempt, this));
      as_.start();

      ROS_INFO("Action server ready");
    }

    ~VrTeleopActionServer() override
    {
      if (js_init_timer_.hasStarted())
        js_init_timer_.stop();
      js_init_sub_.shutdown();
      init_js_cache_sub_.shutdown();

      if (goal_thread_.joinable())
        goal_thread_.join();

      if (sim_thread_.joinable())
        sim_thread_.join();

      const pid_t p = pid_.load(std::memory_order_acquire);
      if (p > 0)
      {
        ::kill(p, SIGTERM);
        ::waitpid(p, nullptr, 0);
        pid_.store(-1, std::memory_order_release);
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
      request_meta_data_json["meta_data"]["world_name"] = config_.world_name;
      request_meta_data_json["meta_data"]["simulation_name"] = config_.simulation_name;
      request_meta_data_json["meta_data"]["length_unit"] = "m";
      request_meta_data_json["meta_data"]["angle_unit"] = "rad";
      request_meta_data_json["meta_data"]["mass_unit"] = "kg";
      request_meta_data_json["meta_data"]["time_unit"] = "s";
      request_meta_data_json["meta_data"]["handedness"] = "rhs";

      for (const auto &kv : config_.send_objects)
        request_meta_data_json["send"][kv.first].append(to_string(kv.second));

      for (const auto &kv : config_.receive_objects)
        request_meta_data_json[init_phase_ ? "send" : "receive"][kv.first].append(to_string(kv.second));

      if (init_phase_)
      {
        for (const auto &kv : config_.init_joints)
        {
          const std::string &mujoco_jointuator = kv.first;
          const std::string type = to_string(kv.second);

          auto &a = request_meta_data_json["send"][mujoco_jointuator];
          if (std::none_of(a.begin(), a.end(), [&](const Json::Value &v)
                           { return v.asString() == type; }))
            a.append(type);
        }
        
        for (const auto &kv : config_.init_frames)
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
      ros_joint_pos_to_index_.clear();
      mujoco_joint_pos_to_index_.clear();
      mujoco_body_pos_to_index_.clear();
      mujoco_body_quat_to_index_.clear();

      if (init_phase_)
      {
        size_t idx = 0;
        for (const std::string &send_object : response_meta_data_json["send"].getMemberNames())
        {
          for (const std::string &type : response_meta_data_json["send"][send_object].getMemberNames())
          {
            if (type == "position")
            {
              mujoco_body_pos_to_index_[send_object] = idx;
              idx += 3;
            }
            else if (type == "quaternion")
            {
              mujoco_body_quat_to_index_[send_object] = idx;
              idx += 4;
            }
            else if (type == "joint_angular_position" || type == "joint_linear_position")
            {
              mujoco_joint_pos_to_index_[send_object] = idx;
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
          ros_joint_pos_to_index_[ros_joint] = ros_idx++;

        size_t act_idx = 0;
        for (const std::string &mujoco_jointuator : response_meta_data_json["receive"].getMemberNames())
          mujoco_joint_pos_to_index_[mujoco_jointuator] = act_idx++;
      }
    }

    void init_send_and_receive_data() override
    {
      if (init_phase_)
      {
        auto bind_act_to_send = [this](const std::string &act)
        {
          auto it = mujoco_joint_pos_to_index_.find(act);
          if (it == mujoco_joint_pos_to_index_.end())
            return;
          const size_t send_index = it->second;
          send_buffer.buffer_double.data[send_index] = joint_commands_[act].position.get();
          joint_commands_[act].position.bind(&send_buffer.buffer_double.data[send_index]);
        };

        for (auto &pub_cfg : publishers_)
        {
          pub_cfg.next_pub_time_s = 0.0;
          pub_cfg.last_warn_time_s = 0.0;
          for (const auto &act : pub_cfg.actuators)
            bind_act_to_send(act);
        }

        for (const auto &kv : config_.init_joints)
          bind_act_to_send(kv.first);

        auto bind_frame_to_send = [this](const std::string &frame)
        {
          auto it_pos = mujoco_body_pos_to_index_.find(frame);
          auto it_quat = mujoco_body_quat_to_index_.find(frame);
          if (it_pos == mujoco_body_pos_to_index_.end() || it_quat == mujoco_body_quat_to_index_.end())
            return;
          const size_t pos_index = it_pos->second;
          const size_t quat_index = it_quat->second;
          const auto &tr = config_.init_frames[frame].translation;
          const auto &q = config_.init_frames[frame].rotation;
          send_buffer.buffer_double.data[pos_index + 0] = tr.x;
          send_buffer.buffer_double.data[pos_index + 1] = tr.y;
          send_buffer.buffer_double.data[pos_index + 2] = tr.z;
          send_buffer.buffer_double.data[quat_index + 0] = q.w;
          send_buffer.buffer_double.data[quat_index + 1] = q.x;
          send_buffer.buffer_double.data[quat_index + 2] = q.y;
          send_buffer.buffer_double.data[quat_index + 3] = q.z;
        };

        for (const auto &kv : config_.init_frames)
          bind_frame_to_send(kv.first);
      }
      else
      {
        for (auto &pub_cfg : publishers_)
        {
          for (const auto &act : pub_cfg.actuators)
          {
            const size_t i = mujoco_joint_pos_to_index_[act];
            joint_commands_[act].position.bind(&receive_buffer.buffer_double.data[i]);
          }
        }

        for (auto &sub_cfg : subscribers_)
        {
          for (const auto &mujoco_joint : sub_cfg.joints)
          {
            const auto it = mujoco_joint_to_ros_joint_.find(mujoco_joint);
            if (it == mujoco_joint_to_ros_joint_.end())
              continue;
            const std::string &ros_joint = it->second;

            const auto it2 = ros_joint_pos_to_index_.find(ros_joint);
            if (it2 == ros_joint_pos_to_index_.end())
              continue;
            const size_t i = it2->second;

            joint_states_[ros_joint].position.bind(&send_buffer.buffer_double.data[i]);
          }
        }
      }
    }

    void bind_send_data() override { *world_time = get_time_now() - start_time_; }
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
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      }

      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
      auto updated = config_.init_frames; // keep previous unless successful

      while (ros::ok() && std::chrono::steady_clock::now() < deadline)
      {
        bool all_ok = true;

        for (const auto &kv : marker_ref_to_source_)
        {
          const std::string &ref_frame = kv.first;
          const std::string &source_frame = kv.second;

          try
          {
            geometry_msgs::TransformStamped t = tf_buffer_->lookupTransform(marker_root_frame_, source_frame, ros::Time(0));
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

      config_.init_frames = std::move(updated);
      ROS_INFO("Refreshed init_frames: %zu frames", config_.init_frames.size());
    }

    // Cached last JointState on js_init_topic_
    void refresh_joint_states()
    {
      if (init_teleop_to_source_ros_.empty())
        return;

      sensor_msgs::JointStateConstPtr last;
      {
        std::lock_guard<std::mutex> lk(init_js_cache_m_);
        last = init_js_cache_last_;
      }

      if (!last)
      {
        ROS_WARN("refresh_joint_states: no cached JointState on %s (keeping previous)", js_init_topic_.c_str());
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
      cache.resolve(*last, js_init_topic_);

      size_t updated = 0;
      cache.apply(*last, [&](size_t k, int i)
                  {
                  if (i >= 0 && i < (int)last->position.size())
                  {
                    joint_commands_[teleop[k]].position.set(last->position[i]);
                    updated++;
                  } });

      ROS_INFO("Refreshed init actuators from cached %s: updated=%zu", js_init_topic_.c_str(), updated);
    }

    // ---------------------------- JointState one-shot init ----------------------------
    // Keeps your old behavior: quickly initializes teleop joints once after startup (or on reload).
    void start_joint_state_initializer()
    {
      if (init_teleop_to_source_ros_.empty())
        return;

      js_init_timer_.stop();
      js_init_sub_.shutdown();

      const ros::Time start = ros::Time::now();
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

      js_init_timer_ = nh_.createTimer(
          ros::Duration(0.05),
          [this, start, timeout_sec, done](const ros::TimerEvent &)
          {
            if (done->load())
            {
              js_init_timer_.stop();
              return;
            }

            const double elapsed = (ros::Time::now() - start).toSec();
            if (elapsed >= timeout_sec)
            {
              ROS_WARN("joint_state_initializer timeout after %.2fs, continuing", elapsed);
              done->store(true);
              js_init_sub_.shutdown();
              js_init_timer_.stop();
            }
          });

      ROS_INFO("Creating joint_state_initializer subscriber on %s", js_init_topic_.c_str());

      js_init_sub_ = nh_.subscribe<sensor_msgs::JointState>(
          js_init_topic_, 10,
          [this, done, teleop_list, cache](const sensor_msgs::JointStateConstPtr &msg)
          {
            if (done->load())
              return;

            if (!cache->resolved)
              cache->resolve(*msg, js_init_topic_);

            bool any = false;
            cache->apply(*msg, [&](size_t k, int i)
                         {
                         if (i >= 0 && i < (int)msg->position.size())
                         {
                           joint_commands_[(*teleop_list)[k]].position.set(msg->position[i]);
                           any = true;
                         } });

            if (any)
            {
              done->store(true);
              js_init_sub_.shutdown();
              js_init_timer_.stop();
              ROS_INFO("joint_state_initializer finished");
            }
          });
    }

    // ---------------------------- config parse (ROS1 rosparam tree) ----------------------------
    void parse_config()
    {
      // Required
      if (!pnh_.getParam("sim_path", sim_path_) || sim_path_.empty())
        throw std::runtime_error("Missing required parameter: ~sim_path");
      if (!pnh_.getParam("model_path", model_path_) || model_path_.empty())
        throw std::runtime_error("Missing required parameter: ~model_path");

      std::filesystem::path sim_fs = std::filesystem::absolute(sim_path_);
      std::filesystem::path model_fs = std::filesystem::absolute(model_path_);
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
      sim_path_ = sim_fs.string();
      model_path_ = model_fs.string();

      if (!std::filesystem::exists(sim_path_))
        throw std::runtime_error("sim_path does not exist: " + sim_path_);
      if (!std::filesystem::exists(model_path_))
        throw std::runtime_error("model_path does not exist: " + model_path_);

      ROS_INFO("Sim path: %s", sim_path_.c_str());
      ROS_INFO("Model path: %s", model_path_.c_str());

      pnh_.param("filter/time_constant", filter_time_constant_, filter_time_constant_);
      pnh_.param("filter/damping", filter_damping_, filter_damping_);
      if (!(filter_time_constant_ >= 0.0))
        throw std::runtime_error("filter/time_constant should be >= 0");
      if (!(filter_damping_ >= 0.0))
        throw std::runtime_error("filter/damping should be >= 0");
      ROS_INFO("Filter params: time_constant=%.6f, damping=%.6f", filter_time_constant_, filter_damping_);

      // reset
      publishers_.clear();
      subscribers_.clear();
      config_.send_objects.clear();
      config_.receive_objects.clear();
      config_.init_frames.clear();
      config_.init_joints.clear();

      marker_ref_to_source_.clear();
      init_teleop_to_source_ros_.clear();
      ros_joint_to_mujoco_joint_.clear();
      mujoco_joint_to_ros_joint_.clear();

      // ---- markers_initializer ----
      {
        pnh_.param<std::string>("initializers/markers_initializer/frame_id", marker_root_frame_, marker_root_frame_);

        XmlRpc::XmlRpcValue frames;
        if (pnh_.getParam("initializers/markers_initializer/frames", frames) &&
            frames.getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
          for (const auto &k : xmlrpc_struct_keys(frames))
          {
            std::string source;
            if (xmlrpc_get_string(frames[k], source) && !source.empty())
              marker_ref_to_source_[k] = source;
          }
        }
      }

      // ---- joint_state_initializer ----
      {
        pnh_.param<std::string>("initializers/joint_state_initializer/topic", js_init_topic_, js_init_topic_);

        XmlRpc::XmlRpcValue joints;
        if (pnh_.getParam("initializers/joint_state_initializer/joints", joints) &&
            joints.getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
          for (const auto &teleop_joint : xmlrpc_struct_keys(joints))
          {
            std::string source_ros_joint;
            if (!xmlrpc_get_string(joints[teleop_joint], source_ros_joint) || source_ros_joint.empty())
              continue;

            init_teleop_to_source_ros_[teleop_joint] = source_ros_joint;
            (void)joint_commands_[teleop_joint];

            // send it during init_phase
            config_.init_joints[teleop_joint] = SignalType::JointAngularPos;
          }
        }

        // persistent cache subscriber for later refresh in execute()
        init_js_cache_sub_ = nh_.subscribe<sensor_msgs::JointState>(
            js_init_topic_, 10,
            [this](const sensor_msgs::JointStateConstPtr &msg)
            {
              std::lock_guard<std::mutex> lk(init_js_cache_m_);
              init_js_cache_last_ = msg;
            });

        start_joint_state_initializer();
      }

      {
        XmlRpc::XmlRpcValue pubs;
        if (pnh_.getParam("publishers", pubs) && pubs.getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
          for (const auto &pub_name : xmlrpc_struct_keys(pubs))
          {
            const auto &p = pubs[pub_name];
            if (p.getType() != XmlRpc::XmlRpcValue::TypeStruct)
              continue;

            std::string topic;
            int rate = 100;
            if (xmlrpc_has_member(p, "topic"))
              (void)xmlrpc_get_string(p["topic"], topic);
            if (xmlrpc_has_member(p, "rate"))
              (void)xmlrpc_get_int(p["rate"], rate);

            if (topic.empty())
              continue;

            PublisherCfg cfg;
            cfg.topic = topic;
            cfg.publisher = nh_.advertise<std_msgs::Float64MultiArray>(topic, 10);
            cfg.rate = rate;
            cfg.period_s = (rate > 0) ? (1.0 / (double)rate) : 0.0;
            cfg.next_pub_time_s = 0.0;
            cfg.last_warn_time_s = 0.0;
            cfg.max_lag_s = 0.25;

            if (xmlrpc_has_member(p, "joints"))
            {
              const auto &j = p["joints"];
              if (j.getType() == XmlRpc::XmlRpcValue::TypeStruct)
              {
                for (const auto &ros_joint : xmlrpc_struct_keys(j))
                {
                  const auto &jj = j[ros_joint];
                  if (jj.getType() != XmlRpc::XmlRpcValue::TypeStruct)
                    continue;

                  std::string mujoco_joint;
                  std::string type_str;
                  if (xmlrpc_has_member(jj, "from"))
                    (void)xmlrpc_get_string(jj["from"], mujoco_joint);
                  if (xmlrpc_has_member(jj, "type"))
                    (void)xmlrpc_get_string(jj["type"], type_str);

                  if (mujoco_joint.empty() || type_str.empty())
                    continue;

                  cfg.actuators.push_back(mujoco_joint);
                  ros_joint_to_mujoco_joint_[ros_joint] = mujoco_joint;
                  (void)joint_commands_[mujoco_joint];

                  const SignalType st = parse_signal_type(type_str);
                  if (st == SignalType::JointAngularPos || st == SignalType::JointLinearPos)
                    config_.receive_objects[mujoco_joint] = st;
                }
              }
            }

            if (cfg.actuators.empty())
              continue;

            std::string controller_ns = topic;
            if (ends_with(controller_ns, "/command"))
              controller_ns = controller_ns.substr(0, controller_ns.size() - 8);

            std::vector<std::string> ordered_ros_joints;
            if (nh_.getParam(controller_ns + "/joints", ordered_ros_joints) && !ordered_ros_joints.empty())
            {
              std::vector<std::string> ordered_act;
              ordered_act.reserve(cfg.actuators.size());

              for (const auto &ros_joint : ordered_ros_joints)
              {
                auto it = ros_joint_to_mujoco_joint_.find(ros_joint);
                if (it == ros_joint_to_mujoco_joint_.end())
                  continue;

                const std::string &act = it->second;
                if (std::find(cfg.actuators.begin(), cfg.actuators.end(), act) != cfg.actuators.end())
                  ordered_act.push_back(act);
              }

              if (!ordered_act.empty())
                cfg.actuators = std::move(ordered_act);
            }
            else
            {
              throw std::runtime_error("Missing required parameter: " + controller_ns + "/joints (list of ROS joints in order for this publisher)");
            }

            cfg.msg.data.resize(cfg.actuators.size(), 0.0);
            publishers_.push_back(std::move(cfg));
          }
        }
      }

      {
        XmlRpc::XmlRpcValue subs;
        if (pnh_.getParam("subscribers", subs) && subs.getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
          for (const auto &sub_name : xmlrpc_struct_keys(subs))
          {
            const auto &s = subs[sub_name];
            if (s.getType() != XmlRpc::XmlRpcValue::TypeStruct)
              continue;

            std::string topic;
            if (xmlrpc_has_member(s, "topic"))
              (void)xmlrpc_get_string(s["topic"], topic);
            if (topic.empty())
              continue;

            std::vector<std::string> ros_joints;
            std::vector<std::string> mujoco_joints;

            if (xmlrpc_has_member(s, "joints"))
            {
              const auto &j = s["joints"];
              if (j.getType() == XmlRpc::XmlRpcValue::TypeStruct)
              {
                for (const auto &ros_joint : xmlrpc_struct_keys(j))
                {
                  const auto &jj = j[ros_joint];
                  if (jj.getType() != XmlRpc::XmlRpcValue::TypeStruct)
                    continue;

                  std::string mujoco_joint;
                  std::string type_str;

                  if (xmlrpc_has_member(jj, "to"))
                    (void)xmlrpc_get_string(jj["to"], mujoco_joint);
                  if (xmlrpc_has_member(jj, "type"))
                    (void)xmlrpc_get_string(jj["type"], type_str);

                  if (mujoco_joint.empty() || type_str.empty())
                    continue;

                  ros_joints.push_back(ros_joint);
                  mujoco_joints.push_back(mujoco_joint);

                  mujoco_joint_to_ros_joint_[mujoco_joint] = ros_joint;
                  (void)joint_states_[ros_joint];

                  const SignalType st = parse_signal_type(type_str);
                  if (st == SignalType::JointAngularPos || st == SignalType::JointLinearPos)
                    config_.send_objects[mujoco_joint] = st;
                }
              }
            }

            if (ros_joints.empty() || mujoco_joints.empty())
              continue;

            SubscriberCfg cfg;
            cfg.topic = topic;
            cfg.joints = mujoco_joints;

            auto ros_joints_sp = std::make_shared<std::vector<std::string>>(std::move(ros_joints));
            auto cache = std::make_shared<JointStateIndexCache>(*ros_joints_sp);

            cfg.subscriber = nh_.subscribe<sensor_msgs::JointState>(
                topic, 10,
                [this, ros_joints_sp, cache, topic](const sensor_msgs::JointStateConstPtr &msg)
                {
                  if (!cache->resolved)
                    cache->resolve(*msg, topic);

                  cache->apply(*msg, [&](size_t k, int i)
                               {
                               const auto &ros_joint = (*ros_joints_sp)[k];
                               if (i < (int)msg->position.size())
                                 joint_states_[ros_joint].position.set(msg->position[i]);
                               if (i < (int)msg->velocity.size())
                                 joint_states_[ros_joint].velocity.set(msg->velocity[i]); });
                });

            subscribers_.push_back(std::move(cfg));
          }
        }
      }

      build_filter_actuator_list();
    }

    void init_multiverse_client()
    {
      XmlRpc::XmlRpcValue md;
      if (pnh_.getParam("meta_data", md) && md.getType() == XmlRpc::XmlRpcValue::TypeStruct)
      {
        for (const auto &k : xmlrpc_struct_keys(md))
        {
          std::string value;
          if (!xmlrpc_get_string(md[k], value) || value.empty())
            continue;

          if (k == "transport")
          {
            if (value == "tcp")
              set_transport(ClientTransportType::Tcp);
            else if (value == "udp")
              set_transport(ClientTransportType::Udp);
            else if (value == "zmq")
              set_transport(ClientTransportType::Zmq);
            else
              throw std::runtime_error("Invalid transport type: " + value);
            config_.transport = value;
          }
          else if (k == "host")
            config_.host = value;
          else if (k == "server_port")
            config_.server_port = value;
          else if (k == "client_port")
            config_.client_port = value;
          else if (k == "world_name")
            config_.world_name = value;
          else if (k == "simulation_name")
            config_.simulation_name = value;
        }
      }

      host = config_.host;
      server_port = config_.server_port;
      client_port = config_.client_port;

      *world_time = 0.0;
      start_time_ = get_time_now();

      ROS_INFO("Multiverse Server: %s:%s - Multiverse Client: %s:%s",
               host.c_str(), server_port.c_str(), host.c_str(), client_port.c_str());

      connect();
    }

    void build_filter_actuator_list()
    {
      std::unordered_map<std::string, bool> seen;
      filter_actuators_.clear();

      for (const auto &pub_cfg : publishers_)
      {
        for (const auto &act : pub_cfg.actuators)
        {
          if (seen.emplace(act, true).second)
            filter_actuators_.push_back(act);
        }
      }

      y1_.assign(filter_actuators_.size(), 0.0);
      y2_.assign(filter_actuators_.size(), 0.0);
      out_.assign(filter_actuators_.size(), 0.0);

      filter_ready_ = false;

      ROS_INFO("Filter actuator list: %zu actuators", filter_actuators_.size());
    }

    void update_filtered_commands_second_order()
    {
      if (filter_actuators_.empty())
        return;

      const double T = filter_time_constant_;
      const double D = filter_damping_;
      const double T1 = T * T;
      const double T2 = 2.0 * D * T;

      const double current_time = get_time_now() - start_time_;

      std::vector<double> u(filter_actuators_.size(), 0.0);
      for (size_t k = 0; k < filter_actuators_.size(); ++k)
        u[k] = joint_commands_[filter_actuators_[k]].position.get();

      if (!filter_ready_)
      {
        out_ = u;
        y1_ = out_;
        y2_ = out_;

        t2_ = current_time;
        t1_ = current_time;

        for (size_t k = 0; k < filter_actuators_.size(); ++k)
          joint_commands_[filter_actuators_[k]].filtered_position.set(out_[k]);

        filter_ready_ = true;
        return;
      }

      const double delta_T_1 = current_time - t1_;
      const double delta_T_2 = t1_ - t2_;

      if (delta_T_1 <= 1e-9 || delta_T_2 <= 1e-9 || T <= 1e-9)
      {
        out_ = u;
      }
      else
      {
        const double a = (delta_T_1 * delta_T_2) / T1;

        const double b1 = (-T2 / delta_T_1 + T1 / (delta_T_1 * delta_T_1) + T1 / (delta_T_1 * delta_T_2));
        const double b2 = (-1.0 + T2 / delta_T_1 - T1 / (delta_T_1 * delta_T_1));

        for (size_t k = 0; k < out_.size(); ++k)
          out_[k] = a * (u[k] + b1 * y1_[k] + b2 * y2_[k]);
      }

      // Output to filtered_position
      for (size_t k = 0; k < filter_actuators_.size(); ++k)
        joint_commands_[filter_actuators_[k]].filtered_position.set(out_[k]);

      t2_ = t1_;
      t1_ = current_time;
      y2_ = y1_;
      y1_ = out_;
    }

  private:
    // ---------------------------- Action callbacks ----------------------------
    void on_goal()
    {
      if (goal_running_.load())
      {
        ROS_WARN("Rejecting goal: another goal is running");
        // ROS1 SimpleActionServer doesn't have explicit reject after goal callback;
        // we accept goal by default, but we can immediately setAborted.
        TeleopResult res;
        res.return_code = TeleopResult::FAILED;
        res.message = "Another goal is running";
        as_.setAborted(res, res.message);
        return;
      }

      const vr_teleop_msgs::TeleopGoalConstPtr goal = as_.acceptNewGoal();
      if (!goal)
        return;

      timeout_s_ = goal->timeout.toSec();
      ROS_INFO("Received goal request with duration %.3f sec", timeout_s_);

      goal_running_.store(true);

      if (goal_thread_.joinable())
        goal_thread_.join();

      goal_thread_ = std::thread([this]()
                                 { execute(); });
    }

    void on_preempt()
    {
      ROS_INFO("Received request to preempt/cancel goal");
    }

    // ---------------------------- core loop ----------------------------
    void run_phase(bool phase)
    {
      init_phase_ = phase;
      communicate(true);
      communicate();
    }

    void publish_filtered_with_rate_guard(double now_s)
    {
      for (auto &pub_cfg : publishers_)
      {
        if (pub_cfg.rate <= 0 || pub_cfg.period_s <= 0.0)
          continue;

        if (pub_cfg.next_pub_time_s <= 0.0)
          pub_cfg.next_pub_time_s = now_s;

        if (now_s < pub_cfg.next_pub_time_s)
          continue;

        const double lag_s = now_s - pub_cfg.next_pub_time_s;

        if (lag_s > pub_cfg.max_lag_s)
        {
          if ((now_s - pub_cfg.last_warn_time_s) > 1.0)
          {
            const double eff_hz = (lag_s > 1e-9) ? (1.0 / lag_s) : 0.0;
            ROS_WARN("Publisher '%s' cannot keep up: target=%d Hz (period=%.4fs), lag=%.4fs (effective ~%.1f Hz).",
                     pub_cfg.topic.c_str(), pub_cfg.rate, pub_cfg.period_s, lag_s, eff_hz);
            pub_cfg.last_warn_time_s = now_s;
          }
          pub_cfg.next_pub_time_s = now_s;
        }

        auto &data = pub_cfg.msg.data;
        for (size_t i = 0; i < pub_cfg.actuators.size(); ++i)
        {
          const std::string &act = pub_cfg.actuators[i];
          data[i] = joint_commands_[act].filtered_position.get();
        }

        pub_cfg.publisher.publish(pub_cfg.msg);

        pub_cfg.next_pub_time_s += pub_cfg.period_s;
        if (pub_cfg.next_pub_time_s < now_s - pub_cfg.period_s)
          pub_cfg.next_pub_time_s = now_s;
      }
    }

    void execute()
    {
      refresh_marker_init_frames();
      refresh_joint_states();

      run_phase(true);
      run_phase(false);

      ROS_INFO("Executing goal");

      pid_.store(-1, std::memory_order_release);
      sim_starting_.store(false);
      sim_started_.store(false);
      sim_start_failed_.store(false);

      ros::Rate loop_rate(1000);

      TeleopFeedback fb;
      fb.state = TeleopFeedback::SYNCHRONIZING;

      TeleopResult result;
      bool have_result = false;

      while (ros::ok())
      {
        if (is_process_dead())
        {
          fb.state = TeleopFeedback::ERROR;
          message_ = "Simulator process died unexpectedly";
        }

        if (as_.isPreemptRequested())
        {
          result.return_code = TeleopResult::CANCELED;
          have_result = true;
          break;
        }

        if (fb.state == TeleopFeedback::ERROR)
        {
          result.return_code = TeleopResult::FAILED;
          result.message = message_;
          have_result = true;
        }

        if (!ros::ok() || have_result)
          break;

        as_.publishFeedback(fb);

        switch (fb.state)
        {
        case TeleopFeedback::SYNCHRONIZING:
          fb.state = synchronizing();
          break;
        case TeleopFeedback::WAITING_FOR_OPERATOR:
          if (is_timeout())
          {
            result.return_code = TeleopResult::TIMEOUT;
            result.message = "Timeout reached";
            have_result = true;
            break;
          }
          fb.state = waiting_for_operator();
          break;
        case TeleopFeedback::ACTIVE:
          if (is_timeout())
          {
            result.return_code = TeleopResult::TIMEOUT;
            result.message = "Timeout reached";
            have_result = true;
            break;
          }
          fb.state = active();
          break;
        case TeleopFeedback::COMPLETING:
          result.return_code = TeleopResult::DONE;
          have_result = true;
          break;
        default:
          fb.state = TeleopFeedback::ERROR;
          message_ = "Unknown state";
          break;
        }

        communicate();
        update_filtered_commands_second_order();

        ros::spinOnce();
        loop_rate.sleep();
      }

      const pid_t p = pid_.load(std::memory_order_acquire);
      if (p > 0)
      {
        ::kill(p, SIGTERM);
        ::waitpid(p, nullptr, 0);
        pid_.store(-1, std::memory_order_release);
      }

      if (!have_result)
      {
        result.return_code = TeleopResult::FAILED;
        result.message = "No result (unexpected)";
      }

      result.duration = ros::Duration(get_time_now() - start_time_);

      if (ros::ok())
      {
        if (result.return_code == TeleopResult::DONE)
          as_.setSucceeded(result, result.message);
        else if (result.return_code == TeleopResult::CANCELED)
          as_.setPreempted(result, result.message);
        else
          as_.setAborted(result, result.message);
      }

      goal_running_.store(false);
    }

    uint8_t synchronizing()
    {
      if (!sim_starting_.exchange(true))
      {
        if (sim_thread_.joinable())
          sim_thread_.join();

        sim_thread_ = std::thread([this]()
                                  {
                                  char *argv[] = {
                                      const_cast<char *>(sim_path_.c_str()),
                                      const_cast<char *>(model_path_.c_str()),
                                      nullptr};

                                  pid_t child_pid = -1;
                                  const int rc = ::posix_spawn(&child_pid, sim_path_.c_str(), nullptr, nullptr, argv, environ);
                                  if (rc != 0)
                                  {
                                    message_ = std::string("posix_spawn failed: ") + std::strerror(rc);
                                    sim_start_failed_.store(true);
                                    sim_started_.store(false);
                                    return;
                                  }
                                  pid_.store(child_pid, std::memory_order_release);
                                  sim_started_.store(true); });
      }

      if (sim_start_failed_.load())
        return TeleopFeedback::ERROR;

      if (!sim_started_.load())
        return TeleopFeedback::SYNCHRONIZING;

      if (is_synchronized())
      {
        start_time_ = get_time_now();
        ROS_INFO("Synchronized with simulator, starting teleoperation at t=%.3f", start_time_);
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

      publish_filtered_with_rate_guard(get_time_now() - start_time_);
      return TeleopFeedback::ACTIVE;
    }

    bool is_timeout() const
    {
      return timeout_s_ < 0.0 ? false : (get_time_now() - start_time_ >= timeout_s_);
    }

    bool is_synchronized() const
    {
      constexpr double tol = 1e-3;

      for (const auto &kv : mujoco_joint_to_ros_joint_)
      {
        const std::string &ros_joint = kv.second;
        if (ros_joint_to_mujoco_joint_.find(ros_joint) == ros_joint_to_mujoco_joint_.end())
          continue;

        const std::string mujoco_joint = ros_joint_to_mujoco_joint_.at(ros_joint);

        auto it_cmd = joint_commands_.find(mujoco_joint);
        auto it_state = joint_states_.find(ros_joint);

        if (it_cmd == joint_commands_.end() || it_state == joint_states_.end())
        {
          ROS_WARN("Not synchronized: missing joint command or state for %s (ROS joint: %s)",
                   mujoco_joint.c_str(), ros_joint.c_str());
          return false;
        }

        const double cmd = it_cmd->second.position.get();
        const double state = it_state->second.position.get();
        const double err = std::abs(cmd - state);

        if (err > tol)
        {
          ROS_WARN("Not synchronized: %s vs %s, err=%.6f", mujoco_joint.c_str(), ros_joint.c_str(), err);
          return false;
        }
      }

      return true;
    }

    bool is_trigger() const { return true; }
    bool is_done() const { return false; }

    bool is_process_dead()
    {
      const pid_t p = pid_.load(std::memory_order_acquire);
      if (p <= 0)
        return false;

      int status = 0;
      const pid_t r = ::waitpid(p, &status, WNOHANG);
      if (r == 0)
        return false;
      if (r == p)
      {
        pid_.store(-1, std::memory_order_release);
        return true;
      }
      return false;
    }

  private:
    // ROS
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    actionlib::SimpleActionServer<TeleopAction> as_;

    // Threads
    std::thread goal_thread_;
    std::thread sim_thread_;

    // Filter params/state
    double filter_time_constant_ = 0.02;
    double filter_damping_ = 1.0;
    std::vector<std::string> filter_actuators_;
    std::vector<double> y1_, y2_, out_;
    double t1_ = 0.0, t2_ = 0.0;
    bool filter_ready_ = false;

    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unordered_map<std::string, std::string> marker_ref_to_source_;
    std::string marker_root_frame_{"pelvis"};

    // JointState initializer state
    ros::Timer js_init_timer_;
    ros::Subscriber js_init_sub_;
    std::string js_init_topic_{"/joint_states"};
    std::unordered_map<std::string, std::string> init_teleop_to_source_ros_;

    ros::Subscriber init_js_cache_sub_;
    std::mutex init_js_cache_m_;
    sensor_msgs::JointStateConstPtr init_js_cache_last_;

    // Core state
    MultiverseConfig config_;

    double start_time_ = 0.0;
    double timeout_s_ = -1.0;

    std::vector<PublisherCfg> publishers_;
    std::vector<SubscriberCfg> subscribers_;

    std::unordered_map<std::string, JointState> joint_states_;
    std::unordered_map<std::string, JointCommand> joint_commands_;

    std::unordered_map<std::string, std::string> ros_joint_to_mujoco_joint_;
    std::unordered_map<std::string, std::string> mujoco_joint_to_ros_joint_;

    std::unordered_map<std::string, size_t> ros_joint_pos_to_index_;
    std::unordered_map<std::string, size_t> mujoco_joint_pos_to_index_;
    std::unordered_map<std::string, size_t> mujoco_body_pos_to_index_;
    std::unordered_map<std::string, size_t> mujoco_body_quat_to_index_;

    std::string sim_path_;
    std::string model_path_;
    bool init_phase_ = true;

    std::atomic<pid_t> pid_{-1};
    std::atomic_bool goal_running_{false};
    std::atomic_bool sim_starting_{false};
    std::atomic_bool sim_started_{false};
    std::atomic_bool sim_start_failed_{false};

    std::string message_;
  };

} // namespace vr_teleop_action

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vr_teleop_action_server_node");
  ros::NodeHandle nh;
  ros::NodeHandle cfg_nh("/vr_teleop_action_server"); 

  try
  {
    vr_teleop_action::VrTeleopActionServer server(nh, cfg_nh);
    ros::spin();
  }
  catch (const std::exception &e)
  {
    ROS_ERROR("Fatal: %s", e.what());
    return 1;
  }
  return 0;
}