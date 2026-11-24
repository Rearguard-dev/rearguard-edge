#include "ros_recovery_transport.hpp"
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <functional>
#include <mutex>
#include <thread>

namespace harness::observation {
namespace {
using Move = moveit_msgs::action::MoveGroup;
using Follow = control_msgs::action::FollowJointTrajectory;
using Navigate = nav2_msgs::action::NavigateToPose;
using Grip = control_msgs::action::GripperCommand;
struct Pending {
  std::mutex mutex;
  ActionReply reply;
  bool cancelled = false;
  std::function<void()> request_cancel;
};
template <typename Action, typename Convert>
void send_action(const typename rclcpp_action::Client<Action>::SharedPtr &client,
                 const typename Action::Goal &goal, const std::shared_ptr<Pending> &pending,
                 Convert convert) {
  using Handle = rclcpp_action::ClientGoalHandle<Action>;
  typename rclcpp_action::Client<Action>::SendGoalOptions options;
  options.goal_response_callback = [pending, client](typename Handle::SharedPtr handle) {
    bool cancel = false;
    {
      std::scoped_lock lock(pending->mutex);
      if (!handle) { pending->reply.state = ActionState::failed; return; }
      pending->request_cancel = [client, handle] { client->async_cancel_goal(handle); };
      cancel = pending->cancelled;
    }
    // A goal accepted after timeout/cancellation is immediately cancelled.
    if (cancel) {
      try { client->async_cancel_goal(handle); }
      catch (const std::exception &error) {
        std::scoped_lock lock(pending->mutex);
        pending->reply.result = {{"cancel_error", error.what()}};
      }
    }
  };
  options.result_callback = [pending, convert](const typename Handle::WrappedResult &result) {
    std::scoped_lock lock(pending->mutex);
    pending->request_cancel = {};
    if (result.code == rclcpp_action::ResultCode::CANCELED) {
      pending->reply.state = ActionState::cancelled; return;
    }
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result) {
      pending->reply.state = ActionState::failed; return;
    }
    try {
      pending->reply = convert(*result.result);
      // Never turn a terminal result after a cancel request into stage success.
      if (pending->cancelled) pending->reply.state = ActionState::cancelled;
    } catch (const std::exception &error) {
      pending->reply = {ActionState::failed, {{"error", error.what()}}};
    }
  };
  client->async_send_goal(goal, options);
}
Json trajectory(const trajectory_msgs::msg::JointTrajectory &message) {
  Json points = Json::array();
  for (const auto &point : message.points)
    points.push_back({{"positions", point.positions}, {"velocities", point.velocities},
                      {"accelerations", point.accelerations},
                      {"time_s", static_cast<double>(point.time_from_start.sec) +
                                     static_cast<double>(point.time_from_start.nanosec) / 1e9}});
  return {{"joint_order", message.joint_names}, {"points", points}};
}
}
struct RosRecoveryActionTransport::Impl {
  rclcpp::Node::SharedPtr node;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread worker;
  std::map<std::string, rclcpp_action::Client<Move>::SharedPtr> move;
  std::map<std::string, rclcpp_action::Client<Follow>::SharedPtr> follow;
  std::map<std::string, rclcpp_action::Client<Navigate>::SharedPtr> navigate;
  std::map<std::string, rclcpp_action::Client<Grip>::SharedPtr> grip;
  std::map<std::string, std::shared_ptr<Pending>> pending;
  mutable std::mutex mutex;
  explicit Impl(const std::shared_ptr<rclcpp::Context> &context, bool sim)
      : node(std::make_shared<rclcpp::Node>("rearguard_recovery_actions",
             rclcpp::NodeOptions().context(context).parameter_overrides({rclcpp::Parameter("use_sim_time", sim)}))),
        executor([&] { rclcpp::ExecutorOptions o; o.context = context; return o; }()) {}
  ~Impl() {
    // Callers must cancel/confirm quiescence before destruction. Process death
    // additionally requires the deployment's independent actuator watchdog.
    executor.cancel();
    if (worker.joinable()) worker.join();
  }
};
RosRecoveryActionTransport::RosRecoveryActionTransport(const std::shared_ptr<rclcpp::Context> &context,
    const std::vector<std::pair<RecoveryBackend, std::string>> &endpoints, bool sim)
    : impl_(std::make_unique<Impl>(context, sim)) {
  for (const auto &[backend, endpoint] : endpoints) {
    require(!endpoint.empty() && endpoint.front() == '/', "absolute action endpoint required");
    switch (backend) {
    case RecoveryBackend::moveit_plan: impl_->move[endpoint] = rclcpp_action::create_client<Move>(impl_->node, endpoint); break;
    case RecoveryBackend::joint_trajectory: impl_->follow[endpoint] = rclcpp_action::create_client<Follow>(impl_->node, endpoint); break;
    case RecoveryBackend::nav2_pose: impl_->navigate[endpoint] = rclcpp_action::create_client<Navigate>(impl_->node, endpoint); break;
    case RecoveryBackend::gripper_gap: impl_->grip[endpoint] = rclcpp_action::create_client<Grip>(impl_->node, endpoint); break;
    }
  }
  impl_->executor.add_node(impl_->node);
  impl_->worker = std::thread([this] { impl_->executor.spin(); });
}
RosRecoveryActionTransport::~RosRecoveryActionTransport() = default;
bool RosRecoveryActionTransport::available(RecoveryBackend backend, const std::string &name) const {
  switch (backend) {
  case RecoveryBackend::moveit_plan: return impl_->move.contains(name) && impl_->move.at(name)->action_server_is_ready();
  case RecoveryBackend::joint_trajectory: return impl_->follow.contains(name) && impl_->follow.at(name)->action_server_is_ready();
  case RecoveryBackend::nav2_pose: return impl_->navigate.contains(name) && impl_->navigate.at(name)->action_server_is_ready();
  case RecoveryBackend::gripper_gap: return impl_->grip.contains(name) && impl_->grip.at(name)->action_server_is_ready();
  }
  return false;
}
std::string RosRecoveryActionTransport::send(RecoveryBackend backend, const std::string &name, const Json &request) {
  if (!available(backend, name)) return {};
  // One in-flight operation per transport; old accepted goals cannot race a new
  // stage/episode. Completed records are reclaimed, keeping memory bounded.
  std::scoped_lock lock(impl_->mutex);
  for (const auto &[id, record] : impl_->pending) {
    (void)id;
    std::scoped_lock record_lock(record->mutex);
    if (record->reply.state == ActionState::pending) return {};
  }
  impl_->pending.clear();
  auto pending = std::make_shared<Pending>();
  const auto id = unique_id();
  impl_->pending[id] = pending;
  try {
    switch (backend) {
    case RecoveryBackend::moveit_plan: {
      Move::Goal goal;
      auto &m = goal.request;
      m.group_name = request.at("group"); m.pipeline_id = request.at("pipeline"); m.planner_id = request.at("planner");
      m.num_planning_attempts = 1; m.allowed_planning_time = request.at("planning_time_s");
      m.max_velocity_scaling_factor = request.at("velocity_scale");
      m.max_acceleration_scaling_factor = request.at("acceleration_scale");
      m.start_state.is_diff = true;
      m.start_state.joint_state.name = request.at("joint_order").get<std::vector<std::string>>();
      m.start_state.joint_state.position = request.at("start").get<std::vector<double>>();
      moveit_msgs::msg::Constraints constraints;
      const auto targets = request.at("goal").get<std::vector<double>>();
      require(targets.size() == m.start_state.joint_state.name.size(), "planning joint dimension mismatch");
      for (std::size_t i = 0; i < targets.size(); ++i) {
        moveit_msgs::msg::JointConstraint c;
        c.joint_name = m.start_state.joint_state.name[i]; c.position = targets[i];
        c.tolerance_above = c.tolerance_below = request.at("tolerance").get<double>(); c.weight = 1;
        constraints.joint_constraints.push_back(c);
      }
      m.goal_constraints.push_back(constraints);
      goal.planning_options.plan_only = true;
      goal.planning_options.look_around = false; goal.planning_options.replan = false;
      goal.planning_options.planning_scene_diff.is_diff = true;
      goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
      send_action<Move>(impl_->move.at(name), goal, pending, [](const Move::Result &r) -> ActionReply {
        if (r.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS ||
            !r.planned_trajectory.multi_dof_joint_trajectory.points.empty()) return {ActionState::failed, nullptr};
        return {ActionState::succeeded, trajectory(r.planned_trajectory.joint_trajectory)};
      }); break;
    }
    case RecoveryBackend::joint_trajectory: {
      Follow::Goal goal; goal.trajectory.joint_names = request.at("joint_order").get<std::vector<std::string>>();
      for (const auto &point : request.at("points")) {
        trajectory_msgs::msg::JointTrajectoryPoint p;
        p.positions = point.at("positions").get<std::vector<double>>();
        p.velocities = point.at("velocities").get<std::vector<double>>();
        p.accelerations = point.at("accelerations").get<std::vector<double>>();
        p.time_from_start = rclcpp::Duration::from_seconds(point.at("time_s").get<double>());
        goal.trajectory.points.push_back(p);
      }
      send_action<Follow>(impl_->follow.at(name), goal, pending, [](const Follow::Result &r) -> ActionReply {
        return {r.error_code == Follow::Result::SUCCESSFUL ? ActionState::succeeded : ActionState::failed,
                {{"error_code", r.error_code}}};
      }); break;
    }
    case RecoveryBackend::nav2_pose: {
      Navigate::Goal goal;
      const auto pose = request.at("pose").get<std::vector<double>>();
      require(pose.size() == 7, "navigation pose dimension mismatch");
      goal.pose.header.frame_id = request.at("frame"); goal.pose.header.stamp = impl_->node->now();
      goal.pose.pose.position.x = pose[0]; goal.pose.pose.position.y = pose[1]; goal.pose.pose.position.z = pose[2];
      goal.pose.pose.orientation.x = pose[3]; goal.pose.pose.orientation.y = pose[4];
      goal.pose.pose.orientation.z = pose[5]; goal.pose.pose.orientation.w = pose[6];
      goal.behavior_tree = request.at("behavior_tree");
      send_action<Navigate>(impl_->navigate.at(name), goal, pending, [](const Navigate::Result &r) -> ActionReply {
        return {r.error_code == Navigate::Result::NONE ? ActionState::succeeded : ActionState::failed,
                {{"error_code", r.error_code}}};
      }); break;
    }
    case RecoveryBackend::gripper_gap: {
      Grip::Goal goal; goal.command.position = request.at("gap_m"); goal.command.max_effort = request.at("max_effort_n");
      send_action<Grip>(impl_->grip.at(name), goal, pending, [](const Grip::Result &r) -> ActionReply {
        return {ActionState::succeeded, {{"position", r.position}, {"effort", r.effort},
                                        {"reached_goal", r.reached_goal}, {"stalled", r.stalled}}};
      }); break;
    }
    }
  } catch (const std::exception &error) {
    // Sending may have reached the server. Retain unknown ownership/quiescence
    // rather than falsely declaring terminal without a ROS terminal callback.
    std::scoped_lock record_lock(pending->mutex);
    pending->cancelled = true;
    pending->reply.result = {{"transport_error", error.what()}};
  }
  return id;
}
ActionReply RosRecoveryActionTransport::poll(const std::string &id) {
  std::scoped_lock lock(impl_->mutex);
  if (!impl_->pending.contains(id)) return {ActionState::pending, {{"unknown_ticket", true}}};
  const auto &record = impl_->pending.at(id);
  std::scoped_lock record_lock(record->mutex);
  return record->reply;
}
void RosRecoveryActionTransport::cancel(const std::string &id) {
  std::function<void()> cancel;
  {
    std::scoped_lock lock(impl_->mutex);
    if (!impl_->pending.contains(id)) return;
    const auto &record = impl_->pending.at(id);
    std::scoped_lock record_lock(record->mutex);
    record->cancelled = true; cancel = record->request_cancel;
  }
  if (cancel) cancel();
}
} // namespace harness::observation
