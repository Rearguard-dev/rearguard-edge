#include "recovery_adapters.hpp"
#include <algorithm>
#include <cmath>
#include <set>

namespace harness::observation {
namespace {
bool number(const Json &v, double lo, double hi) {
  return v.is_number() && std::isfinite(v.get<double>()) && v.get<double>() >= lo && v.get<double>() <= hi;
}
bool vector(const Json &v, std::size_t size) {
  return v.is_array() && v.size() == size && std::all_of(v.begin(), v.end(), [](const Json &x) {
    return number(x, -1e9, 1e9);
  });
}
bool exact_keys(const Json &v, const std::set<std::string> &allowed) {
  if (!v.is_object()) return false;
  for (auto it = v.begin(); it != v.end(); ++it) if (!allowed.contains(it.key())) return false;
  return true;
}
std::string endpoint(const Json &value) {
  auto text = required_text(value, "adapter endpoint");
  require(text.front() == '/' && text.find("//") == std::string::npos, "absolute adapter endpoint required");
  return text;
}
const std::map<std::string, std::size_t> arm_sizes = {
    {"so101/so101", 5}, {"agilex_piper/piper", 6}};
}

AdapterRecoveryLibraryPort::AdapterRecoveryLibraryPort(RecoveryActionTransport &transport,
    RecoveryEnvironment &environment, std::map<std::string, std::string> implementations)
    : transport_(transport), environment_(environment), implementations_(std::move(implementations)) {}

bool AdapterRecoveryLibraryPort::configure(const RecoveryRecipe &recipe, const Json &binding,
                                          const Json &contract) {
  if ((!ticket_.empty() || contact_active_) && !terminal()) return false;
  configured_ = false;
  try {
    config_ = binding.at("recovery_adapters");
    require(exact_keys(config_, {"schema_version", "robot_family", "robot_model", "arm", "moveit", "tool", "nav2", "contact"}),
            "unknown recovery adapter binding field");
    require(config_.at("schema_version") == 1, "unsupported adapter binding schema");
    const auto model = required_text(config_.at("robot_model"), "robot_model");
    const auto family = config_.at("robot_family").get<std::string>();
    const auto model_key = family + "/" + model;
    require(arm_sizes.contains(model_key), "unsupported first-target robot model");
    const auto &arm = config_.at("arm");
    require(exact_keys(arm, {"joint_order", "resource", "units", "lower", "upper", "velocity", "acceleration",
                            "start_tolerance", "goal_tolerance", "stopped_velocity", "checkpoints", "trajectory_action"}),
            "unknown arm binding field");
    const auto joints = arm.at("joint_order").get<std::vector<std::string>>();
    require(joints.size() == arm_sizes.at(model_key) && arm.at("units") == "rad", "explicit arm axes in radians required");
    const auto observed = binding.at("joint_order").get<std::vector<std::string>>();
    std::set<std::string> unique;
    for (const auto &joint : joints)
      require(!joint.empty() && unique.insert(joint).second &&
              std::find(observed.begin(), observed.end(), joint) != observed.end(), "arm joint not uniquely bound");
    required_text(arm.at("resource"), "arm.resource");
    for (const auto *key : {"lower", "upper", "velocity", "acceleration"})
      require(vector(arm.at(key), joints.size()), "arm limit dimensions invalid");
    for (std::size_t i = 0; i < joints.size(); ++i)
      require(arm.at("lower")[i].get<double>() < arm.at("upper")[i].get<double>() &&
              arm.at("velocity")[i].get<double>() > 0 && arm.at("acceleration")[i].get<double>() > 0,
              "invalid arm limits");
    for (const auto *key : {"start_tolerance", "goal_tolerance", "stopped_velocity"})
      require(number(arm.at(key), 1e-6, 0.25), "invalid arm tolerance");
    const auto root = "/" + binding.at("recovery_controller").at("id").get<std::string>() + "/";
    require(endpoint(arm.at("trajectory_action")).starts_with(root), "trajectory must use recovery controller");
    require(arm.at("checkpoints").is_object() && !arm.at("checkpoints").empty(), "approved checkpoints required");
    for (const auto &checkpoint : arm.at("checkpoints")) {
      require(vector(checkpoint, joints.size()), "checkpoint dimension invalid");
      for (std::size_t i = 0; i < joints.size(); ++i)
        require(number(checkpoint[i], arm.at("lower")[i], arm.at("upper")[i]), "checkpoint outside limits");
    }
    if (config_.contains("moveit")) {
      const auto &m = config_.at("moveit");
      require(exact_keys(m, {"action", "group", "pipeline", "planner", "planning_time_s", "velocity_scale", "acceleration_scale"}), "unknown MoveIt field");
      endpoint(m.at("action")); required_text(m.at("group"), "moveit.group");
      required_text(m.at("pipeline"), "moveit.pipeline"); required_text(m.at("planner"), "moveit.planner");
      require(number(m.at("planning_time_s"), 0.01, 30) && number(m.at("velocity_scale"), 0.001, 1) &&
              number(m.at("acceleration_scale"), 0.001, 1), "invalid MoveIt planning limits");
    }
    if (config_.contains("tool")) {
      const auto &tool = config_.at("tool");
      require(exact_keys(tool, {"action", "resource", "units", "min_gap", "max_gap", "max_effort", "tolerance"}), "unknown tool field");
      endpoint(tool.at("action"));
      require(required_text(tool.at("resource"), "tool.resource") != arm.at("resource").get<std::string>() &&
              tool.at("units") == "m" && number(tool.at("min_gap"), 0, 1) &&
              number(tool.at("max_gap"), 0, 1) && tool.at("min_gap").get<double>() < tool.at("max_gap").get<double>() &&
              number(tool.at("max_effort"), 0.001, 10000) && number(tool.at("tolerance"), 1e-6, 0.01), "invalid gripper binding");
    }
    if (config_.contains("nav2")) {
      const auto &nav = config_.at("nav2");
      require(exact_keys(nav, {"action", "resource", "frame", "behavior_tree", "checkpoints"}), "unknown Nav2 field");
      endpoint(nav.at("action")); required_text(nav.at("resource"), "nav2.resource");
      required_text(nav.at("frame"), "nav2.frame"); required_text(nav.at("behavior_tree"), "nav2.behavior_tree");
      require(nav.at("resource") != arm.at("resource") && (!config_.contains("tool") ||
              nav.at("resource") != config_.at("tool").at("resource")), "base resource must be distinct");
      require(nav.at("checkpoints").is_object() && !nav.at("checkpoints").empty(), "base checkpoints required");
      for (const auto &pose : nav.at("checkpoints")) {
        require(vector(pose, 7), "Nav2 pose requires xyz + xyzw");
        double norm = 0;
        for (std::size_t i = 3; i < 7; ++i) norm += pose[i].get<double>() * pose[i].get<double>();
        require(std::abs(norm - 1) < 1e-6, "invalid checkpoint quaternion");
      }
    }
    if (config_.contains("contact")) {
      const auto &contact = config_.at("contact");
      require(exact_keys(contact, {"resource", "procedures"}) && contact.at("resource") == arm.at("resource") &&
              contact.at("procedures").is_object() && !contact.at("procedures").empty(), "qualified arm contact procedures required");
      for (const auto &procedure : contact.at("procedures")) {
        const auto hash = required_text(procedure.at("implementation_hash"), "contact procedure hash");
        require(exact_keys(procedure, {"implementation_hash", "max_force_n", "max_distance_m", "timeout_s"}) &&
                hash.size() == 64 && hash.find_first_not_of("0123456789abcdef") == std::string::npos &&
                number(procedure.at("max_force_n"), 0.001, 10000) && number(procedure.at("max_distance_m"), 0.000001, 1) &&
                number(procedure.at("timeout_s"), 0.001, 300), "invalid contact procedure envelope");
      }
    }
    recipe_ = recipe;
    resources_.clear();
    for (const auto &stage : recipe.stages) {
      require(validate_stage(stage), "unsupported recovery stage binding");
      const auto type = stage.capability == "nav2_pose_checkpoint_v1" ? "nav2" :
                        stage.capability == "gripper_gap_v1" ? "tool" : "arm";
      resources_.insert(config_.at(type).at("resource").get<std::string>());
    }
    require(environment_.configure(recipe, binding, contract), "workcell predicates/client binding refused");
    ticket_.clear(); phase_ = Phase::idle; cancelling_ = false;
    configured_ = true;
    return true;
  } catch (const std::exception &) { return false; }
}

bool AdapterRecoveryLibraryPort::validate_stage(const RecoveryStage &stage) const {
  if (!implementations_.contains(stage.capability) || implementations_.at(stage.capability) != stage.implementation_hash) return false;
  const auto &p = stage.parameters;
  if (stage.action == RecoveryAction::halt_until_operator)
    return stage.capability == "operator_halt_v1" && p.is_object() && p.empty();
  if (stage.action == RecoveryAction::pause_reobserve)
    return stage.capability == "pause_condition_v1" && p.is_object() && p.empty();
  if (stage.action == RecoveryAction::retreat_checkpoint) {
    if (stage.capability == "moveit_joint_checkpoint_v1")
      return exact_keys(p, {"checkpoint"}) && config_.contains("moveit") && p.contains("checkpoint") &&
          p.at("checkpoint").is_string() && config_.at("arm").at("checkpoints").contains(p.at("checkpoint").get<std::string>()) &&
          config_.at("moveit").at("planning_time_s").get<double>() < stage.timeout_s &&
          transport_.available(RecoveryBackend::moveit_plan, config_.at("moveit").at("action")) &&
          transport_.available(RecoveryBackend::joint_trajectory, config_.at("arm").at("trajectory_action"));
    if (stage.capability == "nav2_pose_checkpoint_v1")
      return exact_keys(p, {"checkpoint"}) && config_.contains("nav2") && p.contains("checkpoint") &&
          p.at("checkpoint").is_string() && config_.at("nav2").at("checkpoints").contains(p.at("checkpoint").get<std::string>()) &&
          transport_.available(RecoveryBackend::nav2_pose, config_.at("nav2").at("action"));
    return false;
  }
  if (stage.action == RecoveryAction::reset_end_effector && stage.capability == "gripper_gap_v1" && config_.contains("tool")) {
    const auto &tool = config_.at("tool");
    return exact_keys(p, {"operation", "gap_m", "max_effort_n"}) && p.at("operation") == "set_gap" &&
        number(p.at("gap_m"), tool.at("min_gap"), tool.at("max_gap")) &&
        number(p.at("max_effort_n"), 0.001, tool.at("max_effort")) &&
        transport_.available(RecoveryBackend::gripper_gap, tool.at("action"));
  }
  if (stage.action == RecoveryAction::unload_disengage && stage.capability == "qualified_contact_exit_v1" && config_.contains("contact")) {
    if (!exact_keys(p, {"procedure", "max_force_n", "max_distance_m"}) || !p.contains("procedure") || !p.at("procedure").is_string() ||
        !config_.at("contact").at("procedures").contains(p.at("procedure").get<std::string>())) return false;
    const auto &limits = config_.at("contact").at("procedures").at(p.at("procedure").get<std::string>());
    return number(p.at("max_force_n"), 0.001, limits.at("max_force_n")) &&
        number(p.at("max_distance_m"), 0.000001, limits.at("max_distance_m")) && stage.timeout_s <= limits.at("timeout_s").get<double>();
  }
  // No generic force-controlled disengagement or vendor-tool reset fallback.
  return false;
}
void AdapterRecoveryLibraryPort::observe(const Observations &obs) { observations_ = obs; environment_.observe(obs); }
RecoveryEvidence AdapterRecoveryLibraryPort::test(const std::string &predicate) { return environment_.test(predicate); }
bool AdapterRecoveryLibraryPort::measured_arm_ready(const Json &state) const {
  const auto &arm = config_.at("arm");
  const auto n = arm.at("joint_order").size();
  if (!state.is_object() || state.value<Ns>("epoch", -1) != observations_.epoch ||
      !state.contains("t_ns") || !state.at("t_ns").is_number_integer() ||
      state.at("t_ns").get<Ns>() < 0 || state.at("t_ns").get<Ns>() > observations_.receipt_ros_ns ||
      observations_.receipt_ros_ns - state.at("t_ns").get<Ns>() > static_cast<Ns>(recipe_.max_age_s * 1e9) ||
      !state.value("disengaged", false) || !state.value("payload_modelled", false) ||
      state.value("scene_revision", "").empty() ||
      !vector(state.at("arm_positions"), n) || !vector(state.at("arm_velocities"), n)) return false;
  for (std::size_t i = 0; i < n; ++i)
    if (!number(state.at("arm_positions")[i], arm.at("lower")[i], arm.at("upper")[i]) ||
        std::abs(state.at("arm_velocities")[i].get<double>()) > arm.at("stopped_velocity").get<double>()) return false;
  return true;
}
bool AdapterRecoveryLibraryPort::start(const RecoveryStage &stage) {
  if (!configured_ || cancelling_ || !validate_stage(stage) || !terminal() ||
      !resources_owned() || !environment_.quiescent()) return false;
  const auto approved = std::find_if(recipe_.stages.begin(), recipe_.stages.end(), [&](const RecoveryStage &s) {
    return s.id == stage.id && s.action == stage.action && s.capability == stage.capability &&
           s.implementation_hash == stage.implementation_hash && s.parameters == stage.parameters &&
           s.entry == stage.entry && s.completion == stage.completion && s.timeout_s == stage.timeout_s;
  });
  if (approved == recipe_.stages.end()) return false;
  stage_started_monotonic_ns_ = monotonic_ns();
  stage_timeout_s_ = stage.timeout_s;
  current_stage_ = {{"id", stage.id}, {"capability", stage.capability}, {"parameters", stage.parameters}};
  ticket_.clear();
  if (stage.capability == "qualified_contact_exit_v1") {
    resource_ = config_.at("arm").at("resource");
    if (!environment_.start_contact(stage)) return false;
    contact_active_ = true; phase_ = Phase::contacting; return true;
  }
  if (stage.capability == "pause_condition_v1" || stage.capability == "operator_halt_v1") {
    resource_ = config_.at("arm").at("resource");
    if (!environment_.owns(resource_) || !environment_.quiescent()) return false;
    phase_ = Phase::paused; return true;
  }
  Json request;
  RecoveryBackend backend;
  std::string action;
  if (stage.capability == "moveit_joint_checkpoint_v1") {
    const auto &arm = config_.at("arm"); const auto &m = config_.at("moveit");
    resource_ = arm.at("resource"); planning_state_ = environment_.measured_state();
    if (!environment_.owns(resource_) || !measured_arm_ready(planning_state_)) return false;
    request = {{"group", m.at("group")}, {"pipeline", m.at("pipeline")}, {"planner", m.at("planner")},
               {"joint_order", arm.at("joint_order")}, {"start", planning_state_.at("arm_positions")},
               {"goal", arm.at("checkpoints").at(stage.parameters.at("checkpoint").get<std::string>())},
               {"tolerance", arm.at("goal_tolerance")}, {"planning_time_s", m.at("planning_time_s")},
               {"velocity_scale", m.at("velocity_scale")}, {"acceleration_scale", m.at("acceleration_scale")}};
    backend = RecoveryBackend::moveit_plan; action = m.at("action"); phase_ = Phase::planning;
  } else if (stage.capability == "nav2_pose_checkpoint_v1") {
    const auto &nav = config_.at("nav2"); resource_ = nav.at("resource");
    if (!environment_.owns(resource_)) return false;
    request = {{"pose", nav.at("checkpoints").at(stage.parameters.at("checkpoint").get<std::string>())},
               {"frame", nav.at("frame")}, {"behavior_tree", nav.at("behavior_tree")}};
    backend = RecoveryBackend::nav2_pose; action = nav.at("action"); phase_ = Phase::executing;
  } else {
    const auto &tool = config_.at("tool"); resource_ = tool.at("resource");
    if (!environment_.owns(resource_)) return false;
    request = stage.parameters; backend = RecoveryBackend::gripper_gap;
    action = tool.at("action"); phase_ = Phase::executing;
  }
  ticket_ = transport_.send(backend, action, request);
  if (ticket_.empty()) { phase_ = Phase::failed; return false; }
  return true;
}
bool AdapterRecoveryLibraryPort::validate_plan(const Json &result) const {
  const auto &arm = config_.at("arm");
  const auto &m = config_.at("moveit");
  const auto n = arm.at("joint_order").size();
  const auto state = environment_.measured_state();
  if (!measured_arm_ready(state) || state.at("scene_revision") != planning_state_.at("scene_revision") ||
      result.at("joint_order") != arm.at("joint_order") || !result.at("points").is_array() ||
      result.at("points").size() < 2 || result.at("points").size() > 10000) return false;
  const auto &points = result.at("points");
  const double elapsed = static_cast<double>(monotonic_ns() - stage_started_monotonic_ns_) / 1e9;
  if (points.front().at("time_s") != 0 || !number(points.back().at("time_s"), 0.001, stage_timeout_s_ - elapsed)) return false;
  double previous = -1;
  for (std::size_t index = 0; index < points.size(); ++index) {
    const auto &point = points[index];
    if (!exact_keys(point, {"positions", "velocities", "accelerations", "time_s"}) ||
        !vector(point.at("positions"), n) || !vector(point.at("velocities"), n) ||
        !vector(point.at("accelerations"), n) || !number(point.at("time_s"), 0, 300)) return false;
    const double time = point.at("time_s");
    if (time <= previous) return false;
    for (std::size_t i = 0; i < n; ++i) {
      if (!number(point.at("positions")[i], arm.at("lower")[i], arm.at("upper")[i]) ||
          std::abs(point.at("velocities")[i].get<double>()) > arm.at("velocity")[i].get<double>() * m.at("velocity_scale").get<double>() ||
          std::abs(point.at("accelerations")[i].get<double>()) > arm.at("acceleration")[i].get<double>() * m.at("acceleration_scale").get<double>()) return false;
      if (previous >= 0) {
        const double delta = std::abs(point.at("positions")[i].get<double>() - points[index - 1].at("positions")[i].get<double>());
        if (delta / (time - previous) > arm.at("velocity")[i].get<double>() * m.at("velocity_scale").get<double>()) return false;
      }
    }
    previous = time;
  }
  const auto goal = arm.at("checkpoints").at(current_stage_.at("parameters").at("checkpoint").get<std::string>());
  for (std::size_t i = 0; i < n; ++i)
    if (std::abs(points.front().at("positions")[i].get<double>() - state.at("arm_positions")[i].get<double>()) > arm.at("start_tolerance").get<double>() ||
        std::abs(points.back().at("positions")[i].get<double>() - goal[i].get<double>()) > arm.at("goal_tolerance").get<double>() ||
        std::abs(points.front().at("velocities")[i].get<double>()) > arm.at("stopped_velocity").get<double>() ||
        std::abs(points.back().at("velocities")[i].get<double>()) > arm.at("stopped_velocity").get<double>()) return false;
  return true;
}
bool AdapterRecoveryLibraryPort::terminal() {
  if (contact_active_) {
    if (!environment_.contact_quiescent()) return false;
    contact_active_ = false;
  }
  if (ticket_.empty()) return true;
  return transport_.poll(ticket_).state != ActionState::pending;
}
bool AdapterRecoveryLibraryPort::resources_owned() const {
  return std::all_of(resources_.begin(), resources_.end(), [&](const std::string &resource) {
    return environment_.owns(resource);
  });
}
std::string AdapterRecoveryLibraryPort::poll() {
  if (!configured_ || cancelling_) return "failed";
  if (phase_ == Phase::paused) return resources_owned() && environment_.quiescent() ? "succeeded" : "failed";
  if (phase_ == Phase::idle || phase_ == Phase::failed) return "failed";
  if (!resources_owned()) { if (!ticket_.empty()) transport_.cancel(ticket_); phase_ = Phase::failed; return "failed"; }
  if (phase_ == Phase::succeeded) return "succeeded";
  if (phase_ == Phase::contacting) {
    const auto outcome = environment_.poll_contact();
    if (outcome == "succeeded" && terminal()) { phase_ = Phase::succeeded; return outcome; }
    if (outcome == "running") return outcome;
    phase_ = Phase::failed; return "failed";
  }
  try {
    const auto reply = transport_.poll(ticket_);
    if (reply.state == ActionState::pending) return "running";
    if (reply.state != ActionState::succeeded) { phase_ = Phase::failed; return "failed"; }
    if (phase_ == Phase::planning) {
      if (!validate_plan(reply.result)) { phase_ = Phase::failed; return "failed"; }
      ticket_ = transport_.send(RecoveryBackend::joint_trajectory, config_.at("arm").at("trajectory_action"), reply.result);
      if (ticket_.empty()) { phase_ = Phase::failed; return "failed"; }
      phase_ = Phase::executing; return "running";
    }
    if (current_stage_.at("capability") == "gripper_gap_v1") {
      const auto &result = reply.result;
      const auto gap = current_stage_.at("parameters").at("gap_m").get<double>();
      if (!result.value("reached_goal", false) || result.value("stalled", true) ||
          !number(result.at("effort"), -current_stage_.at("parameters").at("max_effort_n").get<double>(),
                                       current_stage_.at("parameters").at("max_effort_n").get<double>()) ||
          !number(result.at("position"), gap - config_.at("tool").at("tolerance").get<double>(),
                                            gap + config_.at("tool").at("tolerance").get<double>())) {
        phase_ = Phase::failed; return "failed";
      }
    }
    phase_ = Phase::succeeded; return "succeeded";
  } catch (const std::exception &) { phase_ = Phase::failed; return "failed"; }
}
bool AdapterRecoveryLibraryPort::quiescent() { return terminal() && resources_owned() && environment_.quiescent(); }
Json AdapterRecoveryLibraryPort::state_report() const { return environment_.measured_state(); }
bool AdapterRecoveryLibraryPort::prepare_client(const Json &t, const Json &r) { return quiescent() && environment_.prepare_client(t, r); }
Json AdapterRecoveryLibraryPort::client_acknowledgement() { return environment_.client_acknowledgement(); }
bool AdapterRecoveryLibraryPort::validate_candidate(const Json &t, const Json &c) { return quiescent() && environment_.validate_candidate(t, c); }
bool AdapterRecoveryLibraryPort::commit_candidate(const Json &t) { return quiescent() && environment_.commit_candidate(t); }
void AdapterRecoveryLibraryPort::handback_confirmed(const Json &t) { environment_.handback_confirmed(t); }
bool AdapterRecoveryLibraryPort::cancel(const std::string &reason) {
  cancelling_ = true;
  if (!ticket_.empty() && !terminal()) transport_.cancel(ticket_);
  const bool client_cancelled = environment_.cancel(reason);
  return terminal() && environment_.quiescent() && client_cancelled;
}
} // namespace harness::observation
