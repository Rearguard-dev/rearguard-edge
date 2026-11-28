#include "ros_library_recovery.hpp"
#include "recovery_environment.hpp"
#include "recovery_executor.hpp"
#include "ros_recovery.hpp"
#include "ros_recovery_transport.hpp"
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <future>
#include <mutex>
#include <set>
#include <thread>

namespace harness::observation {
namespace {
using namespace std::chrono_literals;
using String = std_msgs::msg::String;
rclcpp::ExecutorOptions executor_options(const std::shared_ptr<rclcpp::Context> &context) {
  rclcpp::ExecutorOptions result; result.context = context; return result;
}
class RosWorkcellChannel final : public RecoveryWorkcellChannel {
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread worker_;
  std::mutex mutex_;
  RecoveryChannelMessage state_, ack_;
  Json joints_, binding_;
  Ns joint_receipt_ = -1;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
  rclcpp::Subscription<String>::SharedPtr state_sub_, ack_sub_;
  rclcpp::Publisher<String>::SharedPtr requests_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr controllers_;
  static std::string topic(const Json &v) {
    const auto name = required_text(v, "recovery environment topic");
    require(name.starts_with("/rearguard/") && name.find("//") == std::string::npos,
            "recovery environment topic must be under /rearguard/");
    return name;
  }
  void receive(const String &message, bool ack) {
    // Reject malformed or oversized messages without killing the ROS executor.
    if (message.data.size() > 1024 * 1024) return;
    try {
      auto value = Json::parse(message.data);
      if (!value.is_object()) return;
      std::scoped_lock lock(mutex_);
      auto &target = ack ? ack_ : state_;
      // No retained/out-of-order state may replace a newer observation.
      if (target.payload.is_object() && value.value<Ns>("epoch", -1) == target.payload.value<Ns>("epoch", -1) &&
          value.value<Ns>("t_ns", -1) < target.payload.value<Ns>("t_ns", -1)) return;
      target = {std::move(value), monotonic_ns()};
    } catch (...) {}
  }
public:
  explicit RosWorkcellChannel(const std::shared_ptr<rclcpp::Context> &context)
      : node_(std::make_shared<rclcpp::Node>("rearguard_recovery_environment", rclcpp::NodeOptions().context(context))),
        executor_(executor_options(context)) {
    executor_.add_node(node_);
    worker_ = std::thread([this] { executor_.spin(); });
  }
  ~RosWorkcellChannel() override { executor_.cancel(); if (worker_.joinable()) worker_.join(); }
  bool configure(const Json &config, const Json &binding) override {
    try {
      {
        std::scoped_lock lock(mutex_);
        binding_ = binding; joints_ = nullptr; joint_receipt_ = -1; state_ = {}; ack_ = {};
      }
      const auto input_binding = std::make_shared<const Json>(binding);
      node_->set_parameter(rclcpp::Parameter("use_sim_time", binding.value("clock", "") == "ros_sim"));
      const auto state_topic = topic(config.at("state_topic"));
      const auto ack_topic = topic(config.at("ack_topic"));
      const auto request_topic = topic(config.at("request_topic"));
      require(state_topic != ack_topic && state_topic != request_topic && ack_topic != request_topic, "distinct recovery protocol topics required");
      const auto manager = required_text(binding.at("controller_manager").at("id"), "controller manager");
      require(manager.starts_with("rearguard_"), "namespaced controller manager required");
      // Volatile protocol messages: an old ready acknowledgement is not latched.
      state_sub_ = node_->create_subscription<String>(state_topic, rclcpp::QoS(4), [this](String::ConstSharedPtr v) { receive(*v, false); });
      ack_sub_ = node_->create_subscription<String>(ack_topic, rclcpp::QoS(4), [this](String::ConstSharedPtr v) { receive(*v, true); });
      requests_ = node_->create_publisher<String>(request_topic, rclcpp::QoS(8));
      std::string joint_topic;
      for (const auto &input : binding.at("topics")) if (input.value("role", "") == "joint_state") {
        require(joint_topic.empty(), "one recovery joint-state source required"); joint_topic = input.at("name").get<std::string>();
      }
      require(!joint_topic.empty(), "recovery joint-state source missing");
      joints_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(joint_topic, rclcpp::SensorDataQoS(),
        [this, input_binding](sensor_msgs::msg::JointState::ConstSharedPtr value) {
          std::map<std::string, std::size_t> indices;
          for (std::size_t i = 0; i < value->name.size(); ++i) if (!indices.emplace(value->name[i], i).second) return;
          if (value->position.size() != value->name.size() || value->velocity.size() != value->name.size()) return;
          Json positions = Json::array(), arm = Json::array(), velocities = Json::array();
          auto append = [&](const Json &order, Json &p, Json *v) {
            for (const auto &joint : order) {
              const auto found = indices.find(joint.get<std::string>());
              if (found == indices.end() || !std::isfinite(value->position[found->second]) || !std::isfinite(value->velocity[found->second])) return false;
              p.push_back(value->position[found->second]); if (v) v->push_back(value->velocity[found->second]);
            }
            return true;
          };
          if (!append(input_binding->at("joint_order"), positions, nullptr) ||
              !append(input_binding->at("recovery_adapters").at("arm").at("joint_order"), arm, &velocities)) return;
          const Ns stamp = static_cast<Ns>(value->header.stamp.sec) * 1000000000LL + value->header.stamp.nanosec;
          std::scoped_lock lock(mutex_);
          joints_ = {{"joint_t_ns", stamp}, {"nominal_positions", positions}, {"arm_positions", arm}, {"arm_velocities", velocities}};
          joint_receipt_ = monotonic_ns();
        });
      controllers_ = node_->create_client<controller_manager_msgs::srv::ListControllers>("/" + manager + "/list_controllers");
      return true;
    } catch (...) { return false; }
  }
  RecoveryChannelMessage state() override {
    std::scoped_lock lock(mutex_);
    if (!joints_.is_object() || !state_.payload.is_object()) return {};
    auto result = state_; result.payload.update(joints_);
    result.received_monotonic_ns = std::min(result.received_monotonic_ns, joint_receipt_);
    bool stopped = true;
    const auto tolerance = binding_.at("recovery_adapters").at("arm").at("stopped_velocity").get<double>();
    for (const auto &v : joints_.at("arm_velocities")) if (std::abs(v.get<double>()) > tolerance) stopped = false;
    result.payload["quiescent"] = stopped && result.payload.value("quiescent", false);
    return result;
  }
  RecoveryChannelMessage acknowledgement() override { std::scoped_lock lock(mutex_); return ack_; }
  bool owns(const std::string &, const Json &binding) override {
    if (!controllers_ || !controllers_->service_is_ready()) return false;
    auto pending = controllers_->async_send_request(std::make_shared<controller_manager_msgs::srv::ListControllers::Request>());
    if (pending.wait_for(100ms) != std::future_status::ready) { controllers_->remove_pending_request(pending); return false; }
    const auto response = pending.get();
    const auto expected = binding.at("controller_id").get<std::string>();
    const auto claims = binding.at("claimed_interfaces").get<std::set<std::string>>();
    bool owned = false;
    for (const auto &controller : response->controller) {
      if (controller.state != "active") continue;
      const std::set<std::string> actual(controller.claimed_interfaces.begin(), controller.claimed_interfaces.end());
      if (controller.name == expected) owned = std::includes(actual.begin(), actual.end(), claims.begin(), claims.end());
      else for (const auto &claim : claims) if (actual.contains(claim)) return false;
    }
    return owned;
  }
  bool send(const Json &value) override {
    if (!requests_ || requests_->get_subscription_count() == 0) return false;
    String message; message.data = value.dump(); requests_->publish(message); return true;
  }
};

class RosRecoveryExecutor final : public ControllerAdapter {
  std::shared_ptr<rclcpp::Context> context_;
  RosTrajectoryRecovery legacy_;
  std::unique_ptr<RosWorkcellChannel> channel_;
  std::unique_ptr<ChannelRecoveryEnvironment> environment_;
  std::unique_ptr<RosRecoveryActionTransport> transport_;
  std::unique_ptr<AdapterRecoveryLibraryPort> port_;
  std::unique_ptr<LibraryRecoveryController> library_;
  RecoveryControllerPlaceholder unavailable_;
  std::unique_ptr<RecoveryExecutorRouter> router_;
public:
  explicit RosRecoveryExecutor(const std::shared_ptr<rclcpp::Context> &context) : context_(context), legacy_(context) {}
  bool ready() const override { return legacy_.ready(); }
  bool prepare(const fs::path &artifact, const Json &binding, const Json &contract) override {
    router_.reset();
    try {
      const auto manifest = read_json(artifact);
      if (manifest.value("kind", "") == "recovery_action_library") {
        std::vector<std::pair<RecoveryBackend, std::string>> endpoints;
        const auto &adapters = binding.at("recovery_adapters");
        endpoints.emplace_back(RecoveryBackend::joint_trajectory, adapters.at("arm").at("trajectory_action").get<std::string>());
        if (adapters.contains("moveit")) endpoints.emplace_back(RecoveryBackend::moveit_plan, adapters.at("moveit").at("action").get<std::string>());
        if (adapters.contains("nav2")) endpoints.emplace_back(RecoveryBackend::nav2_pose, adapters.at("nav2").at("action").get<std::string>());
        if (adapters.contains("tool")) endpoints.emplace_back(RecoveryBackend::gripper_gap, adapters.at("tool").at("action").get<std::string>());
        if (!library_) {
          channel_ = std::make_unique<RosWorkcellChannel>(context_);
          environment_ = std::make_unique<ChannelRecoveryEnvironment>(*channel_);
          transport_ = std::make_unique<RosRecoveryActionTransport>(context_, endpoints, binding.value("clock", "") == "ros_sim");
          port_ = std::make_unique<AdapterRecoveryLibraryPort>(*transport_, *environment_, installed_recovery_implementations("/proc/self/exe"));
          library_ = std::make_unique<LibraryRecoveryController>(*port_);
        }
      }
      ControllerAdapter &library = library_ ? static_cast<ControllerAdapter &>(*library_) : unavailable_;
      router_ = std::make_unique<RecoveryExecutorRouter>(legacy_, library);
      return router_->prepare(artifact, binding, contract);
    } catch (...) { return false; }
  }
  bool begin() override { return router_ && router_->begin(); }
  bool can_begin() const override { return router_ && router_->can_begin(); }
  std::string poll() override { return router_ ? router_->poll() : "unavailable"; }
  bool resume() override { return router_ && router_->resume(); }
  bool hold(const std::string &reason) override { return router_ && router_->hold(reason); }
  void observe(const Observations &v, double now) override { if (router_) router_->observe(v, now); }
  Handback prepare_handback(const Json &v) override { return router_ ? router_->prepare_handback(v) : Handback::failed; }
  void handback_confirmed() override { if (router_) router_->handback_confirmed(); }
  Json telemetry() const override {
    auto report = router_ ? router_->telemetry() : Json(nullptr);
    if (report.is_object() && environment_) report["workcell_state"] = environment_->measured_state();
    return report;
  }
};
}
std::map<std::string, std::string> installed_recovery_implementations(const fs::path &executable) {
  const auto hash = file_hash(executable);
  std::map<std::string, std::string> result;
  for (const auto *capability : {"pause_condition_v1", "operator_halt_v1", "moveit_joint_checkpoint_v1", "nav2_pose_checkpoint_v1", "gripper_gap_v1", "qualified_contact_exit_v1"})
    result[capability] = digest(Json{{"schema_version", 1}, {"capability", capability}, {"executable_sha256", hash}});
  return result;
}
std::unique_ptr<ControllerAdapter> make_ros_recovery_executor(const std::shared_ptr<rclcpp::Context> &context) {
  return std::make_unique<RosRecoveryExecutor>(context);
}
}
