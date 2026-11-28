#include "ros_library_recovery.hpp"
#include "../../include/harness/recovery_client.hpp"
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <mutex>
#include <thread>
#include <iostream>
using namespace harness::observation;
using namespace std::chrono_literals;
namespace {
void run(const std::shared_ptr<rclcpp::Context> &context) {
  const std::vector<std::string> names{"j1", "j2", "j3", "j4", "j5"};
  Json claims = Json::array(); for (const auto &name : names) claims.push_back(name + "/position");
  Json arm = {{"joint_order", names}, {"resource", "arm"}, {"units", "rad"},
    {"lower", std::vector<double>(5, -1)}, {"upper", std::vector<double>(5, 1)},
    {"velocity", std::vector<double>(5, 1)}, {"acceleration", std::vector<double>(5, 1)},
    {"start_tolerance", 0.02}, {"goal_tolerance", 0.02}, {"stopped_velocity", 0.01},
    {"checkpoints", {{"safe", std::vector<double>(5, 0)}}}, {"trajectory_action", "/rearguard_test_recovery/follow_joint_trajectory"}};
  Json binding = {{"clock", "ros_system"}, {"joint_order", names}, {"command_interface", "position"},
    {"controller_manager", {{"id", "rearguard_test_manager"}}}, {"recovery_controller", {{"id", "rearguard_test_recovery"}}},
    {"topics", {{{"name", "/rearguard/test/joints"}, {"role", "joint_state"}}}},
    {"recovery_adapters", {{"schema_version", 1}, {"robot_family", "so101"}, {"robot_model", "so101"}, {"arm", arm}}},
    {"recovery_environment", {{"schema_version", 1}, {"provider_id", "mock-cell"}, {"predicate_ids", {"entry", "complete", "ready", "rearm"}},
      {"state_topic", "/rearguard/test/state"}, {"request_topic", "/rearguard/test/request"}, {"ack_topic", "/rearguard/test/ack"},
      {"resources", {{"arm", {{"controller_id", "rearguard_test_recovery"}, {"claimed_interfaces", claims}}}}},
      {"command_lower", std::vector<double>(5, -1)}, {"command_upper", std::vector<double>(5, 1)}, {"max_start_delta", 0.02}}}};
  auto node = std::make_shared<rclcpp::Node>("rearguard_library_mock_cell", rclcpp::NodeOptions().context(context));
  auto states = node->create_publisher<std_msgs::msg::String>("/rearguard/test/state", rclcpp::QoS(4));
  auto joints = node->create_publisher<sensor_msgs::msg::JointState>("/rearguard/test/joints", rclcpp::SensorDataQoS());
  auto acks = node->create_publisher<std_msgs::msg::String>("/rearguard/test/ack", rclcpp::QoS(4));
  harness::RecoveryClientGate client(digest(binding), "mock-cell", "initial", std::vector<double>(5, -1), std::vector<double>(5, 1));
  std::mutex mutex;
  bool transferred = false;
  auto requests = node->create_subscription<std_msgs::msg::String>("/rearguard/test/request", rclcpp::QoS(8),
    [&](std_msgs::msg::String::ConstSharedPtr message) {
      const auto request = Json::parse(message->data);
      std::scoped_lock lock(mutex);
      if (client.handle(request))
        require(client.stage(client.command_epoch(), std::vector<double>(5, 0), node->now().nanoseconds()), "client fresh candidate refused");
      if (request.at("kind") == "authority_transferred") transferred = true;
    });
  auto list = node->create_service<controller_manager_msgs::srv::ListControllers>("/rearguard_test_manager/list_controllers",
    [&](controller_manager_msgs::srv::ListControllers::Request::SharedPtr,
        controller_manager_msgs::srv::ListControllers::Response::SharedPtr response) {
      controller_manager_msgs::msg::ControllerState controller;
      controller.name = "rearguard_test_recovery"; controller.state = "active";
      controller.claimed_interfaces = claims.get<std::vector<std::string>>(); response->controller.push_back(controller);
    });
  auto timer = node->create_wall_timer(10ms, [&] {
    const auto stamp = node->now();
    Json state = {{"provider_id", "mock-cell"}, {"binding_hash", digest(binding)}, {"epoch", 1}, {"t_ns", stamp.nanoseconds()},
      {"state_revision", "stable-state"}, {"scene_revision", "scene-1"}, {"quiescent", true}, {"payload_modelled", true}, {"disengaged", true},
      {"predicates", Json::object()}};
    for (const auto *predicate : {"entry", "complete", "ready", "rearm"}) state["predicates"][predicate] = {{"observable", true}, {"satisfied", true}};
    std_msgs::msg::String message; message.data = state.dump(); states->publish(message);
    sensor_msgs::msg::JointState measurement; measurement.header.stamp = stamp;
    measurement.name = names; measurement.position.assign(5, 0); measurement.velocity.assign(5, 0); joints->publish(measurement);
    std::scoped_lock lock(mutex);
    const auto ack = client.acknowledgement(stamp.nanoseconds(), 1);
    if (!ack.is_null()) { message.data = ack.dump(); acks->publish(message); }
  });
  rclcpp::ExecutorOptions options; options.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(options); executor.add_node(node);
  struct Worker { rclcpp::Executor &executor; std::thread thread;
    explicit Worker(rclcpp::Executor &v) : executor(v), thread([&v] { v.spin(); }) {}
    ~Worker() { executor.cancel(); thread.join(); }
  } worker(executor);
  auto recovery = make_ros_recovery_executor(context);
  const auto implementations = installed_recovery_implementations("/proc/self/exe");
  Json recipe = {{"schema_version", 1}, {"kind", "recovery_action_library"}, {"id", "pause"}, {"version", "1"},
    {"binding_hash", digest(binding)}, {"controller_id", "rearguard_test_recovery"}, {"rearm_predicate", "rearm"},
    {"handback", {{"readiness_predicate", "ready"}, {"max_age_ms", 500}, {"stability_ms", 0}, {"timeout_ms", 2000}}},
    {"stages", {{{"id", "pause"}, {"action", "pause_reobserve"}, {"capability", "pause_condition_v1"},
      {"implementation_hash", implementations.at("pause_condition_v1")}, {"parameters", Json::object()},
      {"entry_predicate", "entry"}, {"completion_predicate", "complete"}, {"timeout_ms", 2000}}}}};
  const auto path = fs::temp_directory_path() / ("ros-library-" + unique_id() + ".json");
  atomic_json(path, recipe);
  const Json contract = {{"controller_id", "rearguard_test_recovery"}, {"retry_budget", 0}, {"timeout_ms", 5000}};
  require(recovery->prepare(path, binding, contract), "live factory refused library recipe");
  const auto deadline = std::chrono::steady_clock::now() + 8s;
  auto tick = [&] {
    require(std::chrono::steady_clock::now() < deadline, "live library protocol timed out");
    Observations observations; observations.epoch = 1; observations.receipt_ros_ns = node->now().nanoseconds();
    recovery->observe(observations, static_cast<double>(monotonic_ns())/1e9); std::this_thread::sleep_for(15ms);
  };
  // Discovery and fresh measured facts must precede the first stage entry.
  for (int i = 0; i < 20; ++i) tick();
  if (!recovery->begin()) throw std::runtime_error("live library stage failed to begin: " + recovery->telemetry().dump());
  while (recovery->poll() != "succeeded") tick();
  const Json recovery_context = {{"episode_id", "episode"}, {"deployment_id", "deployment"}, {"deployment_content_hash", "content"}};
  while (recovery->prepare_handback(recovery_context) == ControllerAdapter::Handback::pending) tick();
  require(recovery->resume(), "staged client candidate refused");
  { std::scoped_lock lock(mutex); require(client.blocked(), "client opened before authority confirmation"); }
  recovery->handback_confirmed();
  for (;;) {
    { std::scoped_lock lock(mutex); if (transferred) { require(client.take().has_value(), "client lost staged command"); break; } }
    tick();
  }
  fs::remove(path);
}
}
int main() {
  auto context = std::make_shared<rclcpp::Context>(); context->init(0, nullptr);
  try { run(context); context->shutdown("test complete"); std::cout << "live library and client handback protocol passed\n"; }
  catch (...) { context->shutdown("test failed"); throw; }
}
