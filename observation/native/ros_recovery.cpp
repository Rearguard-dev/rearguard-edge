#include "ros_recovery.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <future>
#include <map>
#include <mutex>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <thread>

namespace harness::observation {
namespace {
using namespace std::chrono_literals;
using Follow = control_msgs::action::FollowJointTrajectory;

struct PrivateExecutor {
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::thread thread;
  bool stopped = false;

  PrivateExecutor(const std::shared_ptr<rclcpp::Context> &context,
                  const std::string &name)
      : node(std::make_shared<rclcpp::Node>(
            name, rclcpp::NodeOptions().context(context))) {
    rclcpp::ExecutorOptions options;
    options.context = context;
    executor =
        std::make_unique<rclcpp::executors::SingleThreadedExecutor>(options);
    executor->add_node(node);
    thread = std::thread([this] { executor->spin(); });
  }
  void stop() {
    if (stopped) return;
    stopped = true;
    executor->cancel();
    if (thread.joinable()) thread.join();
  }
  ~PrivateExecutor() { stop(); }
};

std::string required_controller(const Json &binding, const char *key) {
  require(binding.contains(key) && binding.at(key).is_object(),
          std::string("binding requires ") + key);
  return required_text(binding.at(key).at("id"),
                       std::string(key) + ".id");
}

void require_rearguard_name(const std::string &name, const std::string &field) {
  require(name.starts_with("rearguard_"),
          field + " must start with rearguard_");
}

std::chrono::milliseconds bounded_timeout(const Json &binding) {
  const auto value = binding.value("controller_switch_timeout_ms", 1000.0);
  require(std::isfinite(value) && value >= 10 && value <= 10000,
          "controller_switch_timeout_ms must be within 10..10000");
  return std::chrono::milliseconds(static_cast<std::int64_t>(value));
}
} // namespace

struct RosControllerAuthority::Impl {
  PrivateExecutor ros;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr change;
  std::chrono::milliseconds timeout{1000};
  std::string nominal, recovery, fallback, command_interface;
  std::vector<std::string> claims;
  bool configured = false;

  explicit Impl(const std::shared_ptr<rclcpp::Context> &context)
      : ros(context, "rearguard_controller_authority") {}
  ~Impl() { ros.stop(); }

  bool services_ready() const {
    return configured && list && change && list->service_is_ready() &&
           change->service_is_ready();
  }

  controller_manager_msgs::srv::ListControllers::Response::SharedPtr controllers() {
    if (!configured || !list->wait_for_service(timeout)) return nullptr;
    auto future = list->async_send_request(
        std::make_shared<controller_manager_msgs::srv::ListControllers::Request>());
    if (future.wait_for(timeout) != std::future_status::ready) return nullptr;
    return future.get();
  }

  bool owns(const std::string &name) {
    const auto response = controllers();
    if (!response) return false;
    for (const auto &controller : response->controller) {
      if (controller.name != name || controller.state != "active") continue;
      const auto &actual = controller.claimed_interfaces;
      return std::all_of(claims.begin(), claims.end(), [&](const auto &claim) {
        return std::find(actual.begin(), actual.end(), claim) != actual.end();
      });
    }
    return false;
  }

  bool switch_to(const std::string &activate,
                 const std::vector<std::string> &deactivate) {
    if (!configured || !change->wait_for_service(timeout)) return false;
    const auto response = controllers();
    if (!response) return false;
    std::vector<std::string> active_deactivate;
    bool target_active = false;
    for (const auto &controller : response->controller) {
      if (controller.state != "active") continue;
      if (controller.name == activate) target_active = true;
      if (std::find(deactivate.begin(), deactivate.end(), controller.name) !=
          deactivate.end())
        active_deactivate.push_back(controller.name);
    }
    if (target_active && active_deactivate.empty()) return owns(activate);
    auto request =
        std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    if (!target_active) request->activate_controllers = {activate};
    request->deactivate_controllers = active_deactivate;
    request->strictness =
        controller_manager_msgs::srv::SwitchController::Request::STRICT;
    request->activate_asap = true;
    request->timeout = rclcpp::Duration(timeout);
    auto future = change->async_send_request(request);
    return future.wait_for(timeout + 100ms) == std::future_status::ready &&
           future.get()->ok && owns(activate);
  }
};

RosControllerAuthority::RosControllerAuthority(
    const std::shared_ptr<rclcpp::Context> &context)
    : impl_(std::make_unique<Impl>(context)) {}
RosControllerAuthority::~RosControllerAuthority() = default;
bool RosControllerAuthority::ready() const { return impl_->services_ready(); }
bool RosControllerAuthority::configure(const Json &binding,
                                       const Json &contract) {
  try {
    impl_->configured = false;
    impl_->nominal = required_controller(binding, "nominal_controller");
    impl_->recovery = required_controller(binding, "recovery_controller");
    impl_->fallback = required_controller(binding, "fallback_controller");
    const auto manager = required_controller(binding, "controller_manager");
    require_rearguard_name(manager, "controller_manager.id");
    require_rearguard_name(impl_->recovery, "recovery_controller.id");
    require_rearguard_name(impl_->fallback, "fallback_controller.id");
    require(impl_->nominal != impl_->recovery &&
                impl_->fallback != impl_->nominal &&
                impl_->fallback != impl_->recovery,
            "nominal, recovery and fallback controller identities must be distinct");
    require(contract.at("controller_id") == impl_->recovery,
            "approved recovery controller differs from robot binding");
    impl_->command_interface =
        required_text(binding.at("command_interface"), "command_interface");
    impl_->claims.clear();
    for (const auto &joint :
         binding.at("joint_order").get<std::vector<std::string>>())
      impl_->claims.push_back(joint + "/" + impl_->command_interface);
    require(!impl_->claims.empty(), "controller binding has no command interfaces");
    impl_->timeout = bounded_timeout(binding);
    const auto manager_root = "/" + manager;
    impl_->list =
        impl_->ros.node
            ->create_client<controller_manager_msgs::srv::ListControllers>(
                manager_root + "/list_controllers");
    impl_->change =
        impl_->ros.node
            ->create_client<controller_manager_msgs::srv::SwitchController>(
                manager_root + "/switch_controller");
    impl_->configured = true;
    return impl_->change->wait_for_service(impl_->timeout) &&
           impl_->owns(impl_->nominal);
  } catch (...) {
    impl_->configured = false;
    return false;
  }
}
bool RosControllerAuthority::acquire() {
  return impl_->switch_to(impl_->recovery, {impl_->nominal, impl_->fallback});
}
bool RosControllerAuthority::recovery_owns_commands() {
  return impl_->owns(impl_->recovery);
}
bool RosControllerAuthority::release() {
  return impl_->switch_to(impl_->nominal, {impl_->recovery, impl_->fallback});
}
bool RosControllerAuthority::nominal_owns_commands() {
  return impl_->owns(impl_->nominal);
}
bool RosControllerAuthority::hold(const std::string &) {
  return impl_->switch_to(impl_->fallback, {impl_->nominal, impl_->recovery});
}

struct RosTrajectoryRecovery::Impl {
  using GoalHandle = rclcpp_action::ClientGoalHandle<Follow>;
  PrivateExecutor ros;
  rclcpp_action::Client<Follow>::SharedPtr action;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr nominal_sub;
  mutable std::mutex mutex;
  Json stages;
  std::vector<std::string> joint_order;
  std::vector<double> last_positions, final_positions;
  std::string action_name;
  std::shared_ptr<GoalHandle> goal;
  std::chrono::milliseconds action_wait{1000};
  Ns last_joint_ns = -1, completed_ns = -1, nominal_ns = -1;
  std::size_t stage = 0;
  double tolerance = 0.02;
  bool configured = false, active = false, stage_done = false, failed = false;

  explicit Impl(const std::shared_ptr<rclcpp::Context> &context)
      : ros(context, "rearguard_recovery_executor") {}
  ~Impl() { ros.stop(); }

  bool send_stage(std::size_t index) {
    Follow::Goal message;
    message.trajectory.joint_names = joint_order;
    for (const auto &point : stages.at(index).at("trajectory")) {
      trajectory_msgs::msg::JointTrajectoryPoint target;
      target.positions = point.at("positions").get<std::vector<double>>();
      const auto ms = point.at("time_from_start_ms").get<std::int64_t>();
      require(ms > 0, "recovery waypoint time must be positive");
      target.time_from_start =
          rclcpp::Duration(std::chrono::milliseconds(ms));
      message.trajectory.points.push_back(std::move(target));
    }
    require(!message.trajectory.points.empty(), "recovery stage has no waypoints");
    auto options = rclcpp_action::Client<Follow>::SendGoalOptions();
    options.result_callback = [this](const GoalHandle::WrappedResult &result) {
      std::scoped_lock lock(mutex);
      stage_done = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
                   result.result && result.result->error_code ==
                                        Follow::Result::SUCCESSFUL;
      failed = !stage_done;
      if (stage_done && stage + 1 == stages.size())
        completed_ns = monotonic_ns();
    };
    auto future = action->async_send_goal(message, options);
    if (future.wait_for(action_wait) != std::future_status::ready) return false;
    goal = future.get();
    return goal != nullptr;
  }

  bool final_state_ready() const {
    if (completed_ns < 0 || last_joint_ns < completed_ns ||
        nominal_ns < completed_ns || last_positions.size() != final_positions.size())
      return false;
    for (std::size_t i = 0; i < final_positions.size(); ++i)
      if (std::abs(last_positions[i] - final_positions[i]) > tolerance) return false;
    return true;
  }

  void cancel() {
    std::shared_ptr<GoalHandle> current;
    {
      std::scoped_lock lock(mutex);
      current = goal;
      active = false;
    }
    if (current) action->async_cancel_goal(current);
  }
};

RosTrajectoryRecovery::RosTrajectoryRecovery(
    const std::shared_ptr<rclcpp::Context> &context)
    : impl_(std::make_unique<Impl>(context)) {}
RosTrajectoryRecovery::~RosTrajectoryRecovery() { impl_->cancel(); }
bool RosTrajectoryRecovery::ready() const { return rclcpp::ok(impl_->ros.node->get_node_base_interface()->get_context()); }

bool RosTrajectoryRecovery::prepare(const fs::path &artifact,
                                    const Json &binding,
                                    const Json &contract) {
  try {
    impl_->cancel();
    const auto policy = read_json(artifact).at("ros2_policy");
    require(policy.at("action_type") ==
                "control_msgs/action/FollowJointTrajectory",
            "unsupported recovery action type");
    require(policy.at("controller_id") == contract.at("controller_id") &&
                policy.at("controller_id") ==
                    binding.at("recovery_controller").at("id"),
            "recovery artifact controller differs from approved binding");
    impl_->joint_order =
        policy.at("joint_order").get<std::vector<std::string>>();
    require(impl_->joint_order ==
                binding.at("joint_order").get<std::vector<std::string>>(),
            "recovery artifact joint order differs from binding");
    impl_->stages = policy.at("stages");
    require(impl_->stages.is_array() && !impl_->stages.empty(),
            "recovery artifact has no stages");
    for (const auto &stage : impl_->stages)
      for (const auto &point : stage.at("trajectory")) {
        const auto positions = point.at("positions").get<std::vector<double>>();
        require(positions.size() == impl_->joint_order.size() &&
                    std::all_of(positions.begin(), positions.end(),
                                [](double value) { return std::isfinite(value); }),
                "recovery waypoint differs from finite bound joint vector");
      }
    impl_->final_positions = impl_->stages.back().at("trajectory").back().at(
        "positions").get<std::vector<double>>();
    impl_->action_name = required_text(policy.at("action_name"), "action_name");
    const auto recovery_action_root = "/" +
        binding.at("recovery_controller").at("id").get<std::string>() + "/";
    require(impl_->action_name.starts_with(recovery_action_root),
            "recovery action must be namespaced by recovery_controller.id");
    impl_->tolerance = contract.value("completion_tolerance", 0.02);
    require(std::isfinite(impl_->tolerance) && impl_->tolerance > 0 &&
                impl_->tolerance <= 0.25,
            "completion_tolerance must be within (0, 0.25]");
    impl_->action_wait = bounded_timeout(binding);
    impl_->action = rclcpp_action::create_client<Follow>(impl_->ros.node,
                                                         impl_->action_name);
    const auto joint_topic = [&]() {
      for (const auto &topic : binding.at("topics"))
        if (topic.value("role", "") == "joint_state")
          return topic.at("name").get<std::string>();
      throw std::runtime_error("binding has no joint_state topic");
    }();
    const auto nominal_topic = required_text(
        binding.at("nominal_controller").at("command_topic"),
        "nominal_controller.command_topic");
    impl_->joints_sub = impl_->ros.node->create_subscription<sensor_msgs::msg::JointState>(
        joint_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
          std::map<std::string, double> positions;
          for (std::size_t i = 0;
               i < std::min(message->name.size(), message->position.size()); ++i)
            positions[message->name[i]] = message->position[i];
          std::vector<double> ordered;
          for (const auto &name : impl_->joint_order) {
            if (!positions.contains(name) || !std::isfinite(positions[name])) return;
            ordered.push_back(positions[name]);
          }
          std::scoped_lock lock(impl_->mutex);
          impl_->last_positions = std::move(ordered);
          impl_->last_joint_ns = monotonic_ns();
        });
    impl_->nominal_sub =
        impl_->ros.node->create_subscription<std_msgs::msg::Float64MultiArray>(
            nominal_topic, rclcpp::SensorDataQoS(),
            [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr message) {
              if (message->data.size() != impl_->joint_order.size() ||
                  !std::all_of(message->data.begin(), message->data.end(),
                               [](double value) { return std::isfinite(value); }))
                return;
              std::scoped_lock lock(impl_->mutex);
              impl_->nominal_ns = monotonic_ns();
            });
    impl_->configured = impl_->action->wait_for_action_server(impl_->action_wait);
    return impl_->configured;
  } catch (...) {
    impl_->configured = false;
    return false;
  }
}

bool RosTrajectoryRecovery::begin() {
  try {
    std::scoped_lock lock(impl_->mutex);
    if (!impl_->configured) return false;
    impl_->stage = 0;
    impl_->stage_done = false;
    impl_->failed = false;
    impl_->completed_ns = -1;
    impl_->active = impl_->send_stage(0);
    return impl_->active;
  } catch (...) {
    return false;
  }
}

std::string RosTrajectoryRecovery::poll() {
  std::unique_lock lock(impl_->mutex);
  if (!impl_->active || impl_->failed) return "failed";
  if (!impl_->stage_done) return "running";
  if (impl_->stage + 1 == impl_->stages.size())
    return impl_->final_state_ready() ? "succeeded" : "running";
  ++impl_->stage;
  impl_->stage_done = false;
  const auto next = impl_->stage;
  lock.unlock();
  try {
    if (!impl_->send_stage(next)) {
      std::scoped_lock failed_lock(impl_->mutex);
      impl_->failed = true;
      return "failed";
    }
  } catch (...) {
    std::scoped_lock failed_lock(impl_->mutex);
    impl_->failed = true;
    return "failed";
  }
  return "running";
}

bool RosTrajectoryRecovery::resume() {
  std::scoped_lock lock(impl_->mutex);
  if (!impl_->active || !impl_->stage_done || !impl_->final_state_ready()) return false;
  impl_->active = false;
  impl_->goal.reset();
  return true;
}
bool RosTrajectoryRecovery::hold(const std::string &) {
  impl_->cancel();
  return true;
}

} // namespace harness::observation
