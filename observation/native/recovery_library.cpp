#include "recovery_library.hpp"
#include <algorithm>
#include <cmath>
#include <set>

namespace harness::observation {
namespace {
void keys(const Json &value, const std::set<std::string> &allowed) {
  require(value.is_object(), "recipe object required");
  for (auto it = value.begin(); it != value.end(); ++it)
    require(allowed.contains(it.key()), "unknown recipe field: " + it.key());
}
double seconds(const Json &value, double minimum, double maximum) {
  require(value.is_number(), "recipe duration must be numeric");
  const auto ms = value.get<double>();
  require(std::isfinite(ms) && ms >= minimum && ms <= maximum,
          "recipe duration out of bounds");
  return ms / 1000;
}
bool finite_command(const Json &value) {
  return value.is_array() && !value.empty() &&
      std::all_of(value.begin(), value.end(), [](const Json &v) {
        return v.is_number() && std::isfinite(v.get<double>());
      });
}
} // namespace

RecoveryRecipe compile_recovery_recipe(const Json &manifest, const Json &binding,
                                       const Json &contract) {
  keys(manifest, {"schema_version", "kind", "id", "version", "binding_hash",
                  "controller_id", "stages", "handback", "rearm_predicate"});
  require(manifest.at("schema_version") == 1 &&
              manifest.at("kind") == "recovery_action_library",
          "unsupported recovery recipe schema");
  require(manifest.at("binding_hash") == digest(binding), "recipe binding mismatch");
  require(manifest.at("controller_id") == contract.at("controller_id") &&
              manifest.at("controller_id") == binding.at("recovery_controller").at("id"),
          "recipe controller mismatch");
  require(contract.at("retry_budget") == 0,
          "library recipes require zero implicit retries");
  RecoveryRecipe result;
  result.manifest = manifest;
  result.id = required_text(manifest.at("id"), "recipe.id");
  result.version = required_text(manifest.at("version"), "recipe.version");
  result.rearm = required_text(manifest.at("rearm_predicate"), "rearm_predicate");
  const auto &handback = manifest.at("handback");
  keys(handback, {"readiness_predicate", "max_age_ms", "stability_ms", "timeout_ms"});
  result.readiness = required_text(handback.at("readiness_predicate"), "readiness_predicate");
  result.max_age_s = seconds(handback.at("max_age_ms"), 1, 10000);
  result.stability_s = seconds(handback.at("stability_ms"), 0, 60000);
  result.handback_timeout_s = seconds(handback.at("timeout_ms"), 1, 300000);
  require(result.stability_s < result.handback_timeout_s,
          "readiness stability must fit handback timeout");
  const auto &stages = manifest.at("stages");
  require(stages.is_array() && !stages.empty() && stages.size() <= 32,
          "recipe requires 1..32 stages");
  std::set<std::string> ids;
  double total_s = result.handback_timeout_s;
  for (const auto &stage : stages) {
    keys(stage, {"id", "action", "capability", "implementation_hash", "parameters",
                 "entry_predicate", "completion_predicate", "timeout_ms"});
    RecoveryStage s;
    s.id = required_text(stage.at("id"), "stage.id");
    require(ids.insert(s.id).second, "duplicate recovery stage id");
    const auto action = stage.at("action").get<std::string>();
    if (action == "pause_reobserve") s.action = RecoveryAction::pause_reobserve;
    else if (action == "retreat_checkpoint") s.action = RecoveryAction::retreat_checkpoint;
    else if (action == "reset_end_effector") s.action = RecoveryAction::reset_end_effector;
    else if (action == "unload_disengage") s.action = RecoveryAction::unload_disengage;
    else if (action == "halt_until_operator") {
      require(&stage == &stages.back(), "operator halt must be the final stage");
      s.action = RecoveryAction::halt_until_operator;
    }
    else throw std::runtime_error("unknown recovery action: " + action);
    s.capability = required_text(stage.at("capability"), "capability");
    s.implementation_hash = required_text(stage.at("implementation_hash"), "implementation_hash");
    require(s.implementation_hash.size() == 64 &&
                s.implementation_hash.find_first_not_of("0123456789abcdef") == std::string::npos,
            "implementation_hash must be a SHA-256 digest");
    s.entry = required_text(stage.at("entry_predicate"), "entry_predicate");
    s.completion = required_text(stage.at("completion_predicate"), "completion_predicate");
    s.parameters = stage.at("parameters");
    require(s.parameters.is_object(), "stage parameters must be an object");
    s.timeout_s = seconds(stage.at("timeout_ms"), 1, 300000);
    total_s += s.timeout_s;
    result.stages.push_back(std::move(s));
  }
  const auto timeout_s = seconds(contract.at("timeout_ms"), 1, 300000);
  require(total_s <= timeout_s, "stage and handback budgets exceed approved timeout");
  return result;
}

bool LibraryRecoveryController::prepare(const fs::path &artifact, const Json &binding,
                                       const Json &contract) {
  if (state_ != State::unprepared && state_ != State::idle && state_ != State::returned)
    return false;
  state_ = State::unprepared;
  try {
    auto recipe = compile_recovery_recipe(read_json(artifact), binding, contract);
    if (!port_.ready() || !port_.configure(recipe, binding, contract)) return false;
    recipe_ = std::move(recipe);
    observed_ = false;
    now_ = -1;
    transaction_ = nullptr;
    report_ = {{"recipe_id", recipe_.id}, {"recipe_version", recipe_.version},
               {"recipe_hash", digest(recipe_.manifest)}, {"stages", Json::array()},
               {"action_completed", false}, {"physical_ready", false},
               {"client_ready", false}, {"handback_prepared", false},
               {"authority_handback_confirmed", false},
               {"task_outcome", "unknown"}};
    state_ = State::idle;
    return true;
  } catch (const std::exception &error) {
    report_ = {{"preparation_error", error.what()}};
    return false;
  }
}
void LibraryRecoveryController::fail(const std::string &reason) {
  state_ = State::failed;
  report_["failure"] = reason;
  report_["handback_prepared"] = false;
}
void LibraryRecoveryController::observe(const Observations &observations, double now) {
  const bool invalid_clock = !std::isfinite(now) || now < 0 || observations.receipt_ros_ns < 0;
  const bool reset = observed_ && (now < now_ || observations.epoch != observations_.epoch ||
                                  observations.receipt_ros_ns < observations_.receipt_ros_ns);
  if (invalid_clock || reset) {
    observed_ = false;
    stable_since_ = rearm_since_ = -1;
    rearmed_ = false;
    if (state_ != State::idle && state_ != State::unprepared && state_ != State::returned) {
      fail("observation clock changed");
      return;
    }
    if (invalid_clock) return;
  }
  observations_ = observations;
  now_ = now;
  observed_ = true;
  port_.observe(observations);
  if (state_ == State::returned && !rearmed_) {
    const auto cleared = port_.test(recipe_.rearm);
    if (!evidence(cleared)) rearm_since_ = -1;
    else {
      if (rearm_since_ < 0) rearm_since_ = observations_.receipt_ros_ns;
      rearmed_ = observations_.receipt_ros_ns - rearm_since_ >=
          static_cast<Ns>(recipe_.stability_s * 1e9);
    }
  }
}
bool LibraryRecoveryController::evidence(const RecoveryEvidence &value) const {
  return observed_ && value.observable && value.satisfied &&
      value.epoch == observations_.epoch && value.t_ns >= 0 &&
      value.t_ns <= observations_.receipt_ros_ns &&
      observations_.receipt_ros_ns - value.t_ns <= static_cast<Ns>(recipe_.max_age_s * 1e9);
}
bool LibraryRecoveryController::start_stage() {
  const auto &s = recipe_.stages.at(stage_);
  if (!evidence(port_.test(s.entry)) || !port_.start(s)) {
    fail("stage entry refused: " + s.id);
    return false;
  }
  action_completed_ = false;
  stage_started_ns_ = observations_.receipt_ros_ns;
  stage_deadline_ = now_ + s.timeout_s;
  report_["current_stage"] = s.id;
  return true;
}
bool LibraryRecoveryController::can_begin() const {
  return observed_ && (state_ == State::idle || (state_ == State::returned && rearmed_));
}
bool LibraryRecoveryController::begin() {
  if (!can_begin()) return false;
  state_ = State::running;
  stage_ = 0;
  stable_since_ = rearm_since_ = -1;
  rearmed_ = false;
  transaction_ = nullptr;
  report_["stages"] = Json::array();
  for (const auto *key : {"action_completed", "physical_ready", "client_ready", "handback_prepared", "authority_handback_confirmed"})
    report_[key] = false;
  report_.erase("failure");
  return start_stage();
}
std::string LibraryRecoveryController::poll() {
  if (state_ == State::operator_halt) return "operator_halt";
  if (state_ == State::completed || state_ == State::preparing || state_ == State::prepared)
    return "succeeded";
  if (state_ != State::running) return "failed";
  if (now_ >= stage_deadline_) { fail("stage timeout"); return "failed"; }
  if (!action_completed_) {
    const auto outcome = port_.poll();
    if (outcome == "failed" || (outcome != "running" && outcome != "succeeded")) {
      fail("stage execution failed"); return "failed";
    }
    action_completed_ = outcome == "succeeded";
  }
  const auto &s = recipe_.stages.at(stage_);
  const auto completion = port_.test(s.completion);
  if (!action_completed_ || !evidence(completion) || completion.t_ns < stage_started_ns_)
    return "running";
  report_["stages"].push_back({{"id", s.id}, {"outcome", "completed"},
                               {"epoch", observations_.epoch},
                               {"t_ns", observations_.receipt_ros_ns}});
  if (s.action == RecoveryAction::halt_until_operator) {
    state_ = State::operator_halt;
    report_["operator_intervention_required"] = true;
    // The coordinator establishes and confirms the robot-specific latched hold.
    // Never report success: that would permit automatic handback.
    return "operator_halt";
  }
  if (++stage_ < recipe_.stages.size()) return start_stage() ? "running" : "failed";
  state_ = State::completed;
  completed_ns_ = observations_.receipt_ros_ns;
  report_["action_completed"] = true;
  handback_deadline_ = now_ + recipe_.handback_timeout_s;
  return "succeeded";
}
bool LibraryRecoveryController::physical_ready() {
  const auto physical = port_.test(recipe_.readiness);
  if (!evidence(physical) || physical.t_ns < completed_ns_ ||
      physical.state_revision.empty() || !port_.quiescent()) {
    stable_since_ = -1;
    stable_revision_.clear();
    report_["physical_ready"] = false;
    return false;
  }
  if (stable_since_ < 0 || stable_revision_ != physical.state_revision) {
    stable_since_ = observations_.receipt_ros_ns;
    stable_revision_ = physical.state_revision;
  }
  const bool ready = observations_.receipt_ros_ns - stable_since_ >=
      static_cast<Ns>(recipe_.stability_s * 1e9);
  report_["physical_ready"] = ready;
  return ready;
}
ControllerAdapter::Handback LibraryRecoveryController::prepare_handback(const Json &context) {
  if (state_ != State::completed && state_ != State::preparing && state_ != State::prepared)
    return Handback::failed;
  if (transaction_.is_object()) {
    for (auto it = context.begin(); it != context.end(); ++it)
      if (!transaction_.contains(it.key()) || transaction_.at(it.key()) != it.value()) {
        fail("handback deployment or episode changed"); return Handback::failed;
      }
  }
  if (now_ >= handback_deadline_) { fail("handback timeout"); return Handback::failed; }
  if (!physical_ready()) {
    state_ = State::completed;
    transaction_ = nullptr;
    report_["client_ready"] = report_["handback_prepared"] = false;
    return Handback::pending;
  }
  const auto physical = port_.test(recipe_.readiness);
  if (!evidence(physical) || physical.t_ns < completed_ns_ ||
      physical.state_revision != stable_revision_)
    return Handback::pending;
  if (transaction_.is_null() || transaction_.at("state_revision") != physical.state_revision) {
    for (const auto *key : {"episode_id", "deployment_id", "deployment_content_hash"})
      required_text(context.at(key), key);
    transaction_ = context;
    transaction_.update({{"transaction_id", unique_id()}, {"command_epoch", unique_id()},
                         {"state_revision", physical.state_revision},
                         {"epoch", observations_.epoch},
                         {"prepared_after_ns", observations_.receipt_ros_ns}});
    report_["handback_transaction"] = transaction_;
    report_["post_recovery_state"] = port_.state_report();
    report_["client_ready"] = report_["handback_prepared"] = false;
    if (!port_.prepare_client(transaction_, report_)) {
      fail("client preparation refused"); return Handback::failed;
    }
    state_ = State::preparing;
  }
  // An acknowledgement is valid only for this exact prepared state/generation.
  const auto ack = port_.client_acknowledgement();
  if (!ack.is_object() || ack.value("transaction", Json()) != transaction_ ||
      !ack.value("ready", false) || !ack.contains("command") ||
      !finite_command(ack.at("command")) ||
      !port_.validate_candidate(transaction_, ack.at("command"))) return Handback::pending;
  state_ = State::prepared;
  report_["client_ready"] = report_["handback_prepared"] = true;
  return Handback::ready;
}
bool LibraryRecoveryController::resume() {
  if (state_ != State::prepared || now_ >= handback_deadline_ || !physical_ready()) return false;
  const auto physical = port_.test(recipe_.readiness);
  if (!evidence(physical) || physical.t_ns < completed_ns_ ||
      physical.state_revision != transaction_.at("state_revision").get<std::string>()) return false;
  if (!port_.commit_candidate(transaction_)) return false;
  state_ = State::committed;
  return true;
}
void LibraryRecoveryController::handback_confirmed() {
  if (state_ != State::committed) return;
  port_.handback_confirmed(transaction_);
  state_ = State::returned;
  report_["authority_handback_confirmed"] = true;
}
bool LibraryRecoveryController::hold(const std::string &reason) {
  state_ = State::held;
  report_["failure"] = reason;
  report_["handback_prepared"] = false;
  return port_.cancel(reason);
}
} // namespace harness::observation
