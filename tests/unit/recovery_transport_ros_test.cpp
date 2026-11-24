// Real DDS/action protocol against mock servers; no robot or planning engine.
#include "ros_recovery_transport.hpp"
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
using namespace harness::observation;
using namespace std::chrono_literals;
namespace {
using Move = moveit_msgs::action::MoveGroup;
using Follow = control_msgs::action::FollowJointTrajectory;
using Navigate = nav2_msgs::action::NavigateToPose;
using Grip = control_msgs::action::GripperCommand;
template <class Fn> void wait_for(Fn fn, const char *message) {
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (!fn()) { require(std::chrono::steady_clock::now() < deadline, message); std::this_thread::sleep_for(5ms); }
}
void run(const std::shared_ptr<rclcpp::Context> &context) {
  auto node = std::make_shared<rclcpp::Node>("rearguard_recovery_mock_servers", rclcpp::NodeOptions().context(context));
  rclcpp::ExecutorOptions options; options.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(options);
  executor.add_node(node);
  std::atomic<bool> valid{true}, reject{false}, delay{false};
  std::atomic<int> follow_error{0};
  std::mutex mutex;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<Follow>> delayed;
  auto move = rclcpp_action::create_server<Move>(node, "/test_recovery/move_action",
    [&](const rclcpp_action::GoalUUID &, std::shared_ptr<const Move::Goal> goal) {
      if (!goal->planning_options.plan_only || goal->planning_options.replan || goal->planning_options.look_around ||
          goal->request.num_planning_attempts != 1 || goal->request.group_name != "piper_arm" ||
          goal->request.start_state.joint_state.name.size() != 6) valid = false;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }, [](auto) { return rclcpp_action::CancelResponse::ACCEPT; },
    [](auto handle) {
      auto result = std::make_shared<Move::Result>();
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      result->planned_trajectory.joint_trajectory.joint_names = handle->get_goal()->request.start_state.joint_state.name;
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = handle->get_goal()->request.start_state.joint_state.position;
      point.velocities.assign(point.positions.size(), 0); point.accelerations.assign(point.positions.size(), 0);
      result->planned_trajectory.joint_trajectory.points.push_back(point);
      for (auto &position : point.positions) position = 0.1;
      point.time_from_start.sec = 2;
      result->planned_trajectory.joint_trajectory.points.push_back(point);
      handle->succeed(result);
    });
  auto follow = rclcpp_action::create_server<Follow>(node, "/test_recovery/follow_joint_trajectory",
    [&](const rclcpp_action::GoalUUID &, std::shared_ptr<const Follow::Goal>) {
      if (delay) std::this_thread::sleep_for(150ms);
      return reject ? rclcpp_action::GoalResponse::REJECT : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }, [](auto) { return rclcpp_action::CancelResponse::ACCEPT; },
    [&](auto handle) {
      if (handle->get_goal()->trajectory.joint_names.size() != 6) valid = false;
      if (delay) { std::scoped_lock lock(mutex); delayed = handle; return; }
      auto result = std::make_shared<Follow::Result>(); result->error_code = follow_error.load(); handle->succeed(result);
    });
  auto nav = rclcpp_action::create_server<Navigate>(node, "/test_recovery/navigate_to_pose",
    [&](const rclcpp_action::GoalUUID &, std::shared_ptr<const Navigate::Goal> goal) {
      if (goal->pose.header.frame_id != "map" || goal->pose.pose.orientation.w != 1 ||
          goal->behavior_tree != "/qualified/no_retry.xml") valid = false;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }, [](auto) { return rclcpp_action::CancelResponse::ACCEPT; },
    [](auto handle) { auto result = std::make_shared<Navigate::Result>(); result->error_code = Navigate::Result::NONE; handle->succeed(result); });
  auto grip = rclcpp_action::create_server<Grip>(node, "/test_recovery/gripper_cmd",
    [&](const rclcpp_action::GoalUUID &, std::shared_ptr<const Grip::Goal> goal) {
      if (goal->command.position != 0.02 || goal->command.max_effort != 2) valid = false;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }, [](auto) { return rclcpp_action::CancelResponse::ACCEPT; },
    [](auto handle) {
      auto result = std::make_shared<Grip::Result>(); result->position = 0.02;
      result->effort = 1; result->stalled = false; result->reached_goal = true; handle->succeed(result);
    });
  // RAII makes failures stop the test executor before stack-owned callbacks die.
  struct Worker {
    rclcpp::Executor &executor;
    std::thread thread;
    explicit Worker(rclcpp::Executor &e) : executor(e), thread([&e] { e.spin(); }) {}
    ~Worker() { executor.cancel(); thread.join(); }
  } worker(executor);
  const std::vector<std::pair<RecoveryBackend, std::string>> endpoints = {
    {RecoveryBackend::moveit_plan, "/test_recovery/move_action"},
    {RecoveryBackend::joint_trajectory, "/test_recovery/follow_joint_trajectory"},
    {RecoveryBackend::nav2_pose, "/test_recovery/navigate_to_pose"},
    {RecoveryBackend::gripper_gap, "/test_recovery/gripper_cmd"}};
  RosRecoveryActionTransport transport(context, endpoints);
  for (const auto &[backend, name] : endpoints)
    wait_for([&] { return transport.available(backend, name); }, "action discovery timed out");
  require(!transport.available(RecoveryBackend::gripper_gap, "/unbound"), "unregistered endpoint available");
  Json request = {{"group", "piper_arm"}, {"pipeline", "ompl"}, {"planner", "RRTConnectkConfigDefault"},
    {"joint_order", {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"}},
    {"start", std::vector<double>(6, 0)}, {"goal", std::vector<double>(6, 0.1)}, {"tolerance", 0.01},
    {"planning_time_s", 1}, {"velocity_scale", 0.25}, {"acceleration_scale", 0.25}};
  auto ticket = transport.send(RecoveryBackend::moveit_plan, endpoints[0].second, request);
  require(!ticket.empty(), "MoveIt send failed");
  wait_for([&] { return transport.poll(ticket).state != ActionState::pending; }, "MoveIt result timeout");
  const auto plan = transport.poll(ticket);
  require(plan.state == ActionState::succeeded && plan.result.at("points").size() == 2, "planning result conversion");
  ticket = transport.send(RecoveryBackend::joint_trajectory, endpoints[1].second, plan.result);
  wait_for([&] { return transport.poll(ticket).state != ActionState::pending; }, "trajectory timeout");
  require(transport.poll(ticket).state == ActionState::succeeded, "trajectory result");
  follow_error = Follow::Result::INVALID_GOAL;
  ticket = transport.send(RecoveryBackend::joint_trajectory, endpoints[1].second, plan.result);
  wait_for([&] { return transport.poll(ticket).state != ActionState::pending; }, "trajectory error timeout");
  require(transport.poll(ticket).state == ActionState::failed, "domain failure treated as success");
  follow_error = 0;
  reject = true;
  ticket = transport.send(RecoveryBackend::joint_trajectory, endpoints[1].second, plan.result);
  wait_for([&] { return transport.poll(ticket).state != ActionState::pending; }, "rejection timeout");
  require(transport.poll(ticket).state == ActionState::failed, "goal rejection treated as success");
  reject = false; delay = true;
  ticket = transport.send(RecoveryBackend::joint_trajectory, endpoints[1].second, plan.result);
  transport.cancel(ticket);
  require(transport.poll(ticket).state == ActionState::pending, "cancel request falsely reported terminal");
  require(transport.send(RecoveryBackend::gripper_gap, endpoints[3].second, {{"gap_m", 0.02}, {"max_effort_n", 2}}).empty(),
          "overlapping request accepted before cancelled goal became terminal");
  wait_for([&] {
    std::scoped_lock lock(mutex);
    if (!delayed || !delayed->is_canceling()) return false;
    delayed->canceled(std::make_shared<Follow::Result>()); return true;
  }, "late accepted goal did not receive cancellation");
  wait_for([&] { return transport.poll(ticket).state != ActionState::pending; }, "cancel result timeout");
  require(transport.poll(ticket).state == ActionState::cancelled, "cancel terminal result");
  delay = false;
  ticket = transport.send(RecoveryBackend::nav2_pose, endpoints[2].second,
    {{"frame", "map"}, {"pose", {0, 0, 0, 0, 0, 0, 1}}, {"behavior_tree", "/qualified/no_retry.xml"}});
  wait_for([&] { return transport.poll(ticket).state != ActionState::pending; }, "Nav2 result timeout");
  require(transport.poll(ticket).state == ActionState::succeeded, "Nav2 result");
  ticket = transport.send(RecoveryBackend::gripper_gap, endpoints[3].second, {{"gap_m", 0.02}, {"max_effort_n", 2}});
  wait_for([&] { return transport.poll(ticket).state != ActionState::pending; }, "gripper result timeout");
  require(transport.poll(ticket).state == ActionState::succeeded && transport.poll(ticket).result.at("reached_goal") == true,
          "gripper result conversion");
  require(valid, "ROS goal payload differed from bound request");
  (void)move; (void)follow; (void)nav; (void)grip;
}
}
int main(int argc, char **argv) {
  auto context = std::make_shared<rclcpp::Context>(); context->init(argc, argv);
  try { run(context); context->shutdown("test complete"); std::cout << "ROS recovery action mock tests passed\n"; return 0; }
  catch (const std::exception &e) { context->shutdown("test failed"); std::cerr << e.what() << '\n'; return 1; }
}
