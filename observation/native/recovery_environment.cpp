#include "recovery_environment.hpp"
#include <algorithm>
#include <cmath>
#include <set>

namespace harness::observation {
namespace {
bool numbers(const Json &v, std::size_t n) {
  return v.is_array() && v.size() == n && std::all_of(v.begin(), v.end(), [](const Json &x) {
    return x.is_number() && std::isfinite(x.get<double>());
  });
}
}
bool ChannelRecoveryEnvironment::configure(const RecoveryRecipe &recipe, const Json &binding, const Json &) {
  configured_ = observed_ = false;
  context_sent_monotonic_ns_ = -1;
  measured_ = {}; acknowledgement_ = {};
  transaction_ = staged_ = committed_ = contact_ = nullptr;
  try {
    config_ = binding.at("recovery_environment"); binding_ = binding;
    require(config_.at("schema_version") == 1, "unsupported recovery environment");
    required_text(config_.at("provider_id"), "recovery environment provider");
    const auto predicates = config_.at("predicate_ids").get<std::set<std::string>>();
    require(predicates.contains(recipe.readiness) && predicates.contains(recipe.rearm), "missing readiness/rearm predicates");
    for (const auto &stage : recipe.stages)
      require(predicates.contains(stage.entry) && predicates.contains(stage.completion), "unbound stage predicate");
    require(config_.at("resources").is_object() && !config_.at("resources").empty(), "measured resource bindings required");
    for (const auto &resource : config_.at("resources")) {
      require(required_text(resource.at("controller_id"), "resource controller").starts_with("rearguard_"), "resource controller must be namespaced");
      const auto claims = resource.at("claimed_interfaces").get<std::set<std::string>>();
      require(!claims.empty() && !claims.contains(""), "explicit resource claims required");
    }
    for (const auto &stage : recipe.stages) {
      const auto kind = stage.capability == "nav2_pose_checkpoint_v1" ? "nav2" : stage.capability == "gripper_gap_v1" ? "tool" : "arm";
      const auto id = binding.at("recovery_adapters").at(kind).at("resource").get<std::string>();
      require(config_.at("resources").contains(id), "recovery resource has no ownership query");
    }
    const auto n = binding.at("joint_order").size();
    require(n > 0 && numbers(config_.at("command_lower"), n) && numbers(config_.at("command_upper"), n), "nominal command limits required");
    require(binding.at("command_interface") == "position", "recovery client requires position commands");
    const auto tolerance = config_.at("max_start_delta").get<double>();
    require(std::isfinite(tolerance) && tolerance > 0 && tolerance <= 0.25, "invalid handback start tolerance");
    for (std::size_t i = 0; i < n; ++i)
      require(config_.at("command_lower")[i].get<double>() < config_.at("command_upper")[i].get<double>(), "invalid nominal limits");
    max_age_s_ = recipe.max_age_s;
    configured_ = channel_.configure(config_, binding);
    return configured_;
  } catch (...) { return false; }
}
void ChannelRecoveryEnvironment::observe(const Observations &value) {
  if (observed_ && (value.epoch != observations_.epoch || value.receipt_ros_ns < observations_.receipt_ros_ns)) {
    (void)cancel("recovery observation epoch changed");
    observed_ = false;
    context_sent_monotonic_ns_ = -1;
    measured_ = {}; acknowledgement_ = {};
  }
  observations_ = value;
  observed_ = configured_ && value.receipt_ros_ns >= 0;
  const auto monotonic = monotonic_ns();
  if (observed_ && (context_sent_monotonic_ns_ < 0 || monotonic-context_sent_monotonic_ns_ >= 1000000000LL)) {
    if (channel_.send({{"schema_version", 1}, {"kind", "observation_context"}, {"binding_hash", digest(binding_)},
                      {"provider_id", config_.at("provider_id")}, {"epoch", value.epoch}, {"t_ns", value.receipt_ros_ns}}))
      context_sent_monotonic_ns_ = monotonic;
  }
  // Freeze a causal snapshot for this entire tick. ROS callbacks may deliver a
  // newer sample during polling/handback; it belongs to the next tick.
  const auto capture = [&](RecoveryChannelMessage next, RecoveryChannelMessage &cached) {
    try {
      const auto &p = next.payload;
      if (p.at("epoch") == value.epoch && (p.at("t_ns").get<Ns>() > value.receipt_ros_ns ||
          (p.contains("joint_t_ns") && p.at("joint_t_ns").get<Ns>() > value.receipt_ros_ns))) return;
    } catch (...) {}
    cached = std::move(next);
  };
  capture(channel_.state(), measured_);
  capture(channel_.acknowledgement(), acknowledgement_);
}
Json ChannelRecoveryEnvironment::fresh(const RecoveryChannelMessage &message) const {
  try {
    const auto &v = message.payload;
    const auto now = monotonic_ns();
    if (!configured_ || !observed_ || !v.is_object() || message.received_monotonic_ns < 0 ||
        message.received_monotonic_ns > now || now - message.received_monotonic_ns > static_cast<Ns>(max_age_s_ * 1e9) ||
        v.at("binding_hash") != digest(binding_) || v.at("provider_id") != config_.at("provider_id") ||
        v.at("epoch") != observations_.epoch || !v.at("t_ns").is_number_integer()) return nullptr;
    const Ns stamp = v.at("t_ns");
    if (stamp < 0 || stamp > observations_.receipt_ros_ns || observations_.receipt_ros_ns - stamp > static_cast<Ns>(max_age_s_ * 1e9)) return nullptr;
    if (v.contains("joint_t_ns")) {
      const Ns joints = v.at("joint_t_ns");
      if (joints < 0 || joints > observations_.receipt_ros_ns || observations_.receipt_ros_ns - joints > static_cast<Ns>(max_age_s_ * 1e9)) return nullptr;
    }
    return v;
  } catch (...) { return nullptr; }
}
Json ChannelRecoveryEnvironment::measured_state() const { return fresh(measured_); }
RecoveryEvidence ChannelRecoveryEnvironment::test(const std::string &predicate) {
  try {
    const auto state = measured_state();
    const auto &fact = state.at("predicates").at(predicate);
    return {fact.at("observable").get<bool>(), fact.at("satisfied").get<bool>(),
            state.at("epoch"), state.at("t_ns"), required_text(state.at("state_revision"), "measured state revision")};
  } catch (...) { return {}; }
}
bool ChannelRecoveryEnvironment::owns(const std::string &resource) const {
  return configured_ && config_.at("resources").contains(resource) && channel_.owns(resource, config_.at("resources").at(resource));
}
bool ChannelRecoveryEnvironment::resources_owned() {
  for (auto it = config_.at("resources").begin(); it != config_.at("resources").end(); ++it)
    if (!owns(it.key())) return false;
  return true;
}
bool ChannelRecoveryEnvironment::quiescent() const {
  const auto state = measured_state();
  return state.is_object() && state.value("quiescent", false);
}
bool ChannelRecoveryEnvironment::prepare_client(const Json &transaction, const Json &report) {
  if (!quiescent() || !resources_owned()) return false;
  transaction_ = transaction; staged_ = committed_ = nullptr;
  return channel_.send({{"schema_version", 1}, {"kind", "prepare"}, {"binding_hash", digest(binding_)},
                        {"transaction", transaction}, {"report", report}});
}
Json ChannelRecoveryEnvironment::client_acknowledgement() {
  const auto ack = fresh(acknowledgement_);
  if (!ack.is_object() || transaction_.is_null() || ack.value("transaction", Json()) != transaction_ ||
      ack.value("phase", "") != "staged" || !ack.value("queue_invalidated", false) ||
      ack.at("t_ns").get<Ns>() < transaction_.at("prepared_after_ns").get<Ns>()) return nullptr;
  staged_ = ack;
  return {{"transaction", transaction_}, {"ready", true}, {"command", ack.value("command", Json())}};
}
bool ChannelRecoveryEnvironment::validate_candidate(const Json &transaction, const Json &command) {
  try {
    const auto ack = fresh(acknowledgement_);
    const auto state = measured_state();
    const auto n = binding_.at("joint_order").size();
    if (transaction != transaction_ || !staged_.is_object() || ack != staged_ || !quiescent() || !resources_owned() ||
        state.at("state_revision") != transaction.at("state_revision") || !numbers(command, n) ||
        !numbers(state.at("nominal_positions"), n) || staged_.at("command") != command) return false;
    for (std::size_t i = 0; i < n; ++i) {
      const auto value = command[i].get<double>();
      if (value < config_.at("command_lower")[i].get<double>() || value > config_.at("command_upper")[i].get<double>() ||
          std::abs(value - state.at("nominal_positions")[i].get<double>()) > config_.at("max_start_delta").get<double>()) return false;
    }
    return true;
  } catch (...) { return false; }
}
bool ChannelRecoveryEnvironment::commit_candidate(const Json &transaction) {
  if (!staged_.is_object() || !validate_candidate(transaction, staged_.at("command"))) return false;
  committed_ = transaction;
  // The candidate remains gated. Only confirmed authority transfer opens it.
  return channel_.send({{"schema_version", 1}, {"kind", "commit"}, {"binding_hash", digest(binding_)}, {"transaction", transaction}});
}
void ChannelRecoveryEnvironment::handback_confirmed(const Json &transaction) {
  if (transaction == committed_ && !transaction.is_null())
    (void)channel_.send({{"schema_version", 1}, {"kind", "authority_transferred"}, {"binding_hash", digest(binding_)}, {"transaction", transaction}});
}
bool ChannelRecoveryEnvironment::cancel(const std::string &reason) {
  const bool sent = channel_.send({{"schema_version", 1}, {"kind", "cancel"}, {"binding_hash", digest(binding_)}, {"transaction", transaction_},
    {"contact_request_id", contact_.is_object() ? contact_.at("request_id") : Json(nullptr)}, {"reason", reason}});
  transaction_ = staged_ = committed_ = nullptr;
  return sent;
}
bool ChannelRecoveryEnvironment::start_contact(const RecoveryStage &stage) {
  try {
    if (!contact_.is_null() && !contact_quiescent()) return false;
    const auto state = measured_state();
    const auto procedure = stage.parameters.at("procedure").get<std::string>();
    const auto &qualified = binding_.at("recovery_adapters").at("contact").at("procedures").at(procedure);
    const double max_force = stage.parameters.at("max_force_n"), max_distance = stage.parameters.at("max_distance_m");
    if (stage.action != RecoveryAction::unload_disengage || stage.capability != "qualified_contact_exit_v1" ||
        !std::isfinite(max_force) || !std::isfinite(max_distance) || max_force <= 0 || max_distance <= 0 ||
        max_force > qualified.at("max_force_n").get<double>() || max_distance > qualified.at("max_distance_m").get<double>() ||
        !std::isfinite(stage.timeout_s) || stage.timeout_s <= 0 || stage.timeout_s > qualified.at("timeout_s").get<double>()) return false;
    const auto &feedback = state.at("contact_feedback");
    const Ns stamp = feedback.at("t_ns");
    const double force = feedback.at("force_n");
    if (!quiescent() || !resources_owned() || state.at("contact_capabilities").at(procedure) != qualified.at("implementation_hash") ||
        stamp < 0 || stamp > observations_.receipt_ros_ns || observations_.receipt_ros_ns-stamp > static_cast<Ns>(max_age_s_*1e9) ||
        !std::isfinite(force) || force < 0 || force > stage.parameters.at("max_force_n").get<double>()) return false;
    contact_ = {{"request_id", unique_id()}, {"started_ns", observations_.receipt_ros_ns},
      {"parameters", stage.parameters}, {"procedure_implementation_hash", qualified.at("implementation_hash")}};
    const bool sent = channel_.send({{"schema_version", 1}, {"kind", "execute_contact_exit"}, {"binding_hash", digest(binding_)},
      {"operation", contact_}, {"epoch", observations_.epoch}, {"timeout_s", stage.timeout_s}});
    if (!sent) contact_ = nullptr;
    return sent;
  } catch (...) { return false; }
}
std::string ChannelRecoveryEnvironment::poll_contact() {
  try {
    if (!contact_.is_object() || !resources_owned()) return "failed";
    const auto state = measured_state();
    if (!state.is_object()) return "failed";
    const auto &feedback = state.at("contact_feedback");
    const Ns force_stamp = feedback.at("t_ns");
    const double current_force = feedback.at("force_n");
    if (force_stamp < 0 || force_stamp > observations_.receipt_ros_ns ||
        observations_.receipt_ros_ns-force_stamp > static_cast<Ns>(max_age_s_*1e9) || !std::isfinite(current_force) ||
        current_force < 0 || current_force > contact_.at("parameters").at("max_force_n").get<double>()) {
      (void)cancel("contact force feedback missing or outside envelope"); return "failed";
    }
    if (!state.contains("contact_exit") || state.at("contact_exit").value("request_id", "") != contact_.at("request_id").get<std::string>()) return "running";
    const auto &result = state.at("contact_exit");
    const Ns stamp = result.at("t_ns");
    const double force = result.at("force_n"), distance = result.at("distance_m");
    if (stamp < contact_.at("started_ns").get<Ns>() || stamp > observations_.receipt_ros_ns ||
        observations_.receipt_ros_ns-stamp > static_cast<Ns>(max_age_s_*1e9) || !std::isfinite(force) || !std::isfinite(distance) ||
        force < 0 || force > contact_.at("parameters").at("max_force_n").get<double>() ||
        distance < 0 || distance > contact_.at("parameters").at("max_distance_m").get<double>()) {
      (void)cancel("contact feedback outside approved envelope"); return "failed";
    }
    const auto phase = result.at("phase").get<std::string>();
    if (phase == "running" || phase == "accepted") return "running";
    if (phase == "succeeded" && result.value("disengaged", false) && contact_quiescent()) return "succeeded";
    return "failed";
  } catch (...) { return "failed"; }
}
bool ChannelRecoveryEnvironment::contact_quiescent() {
  try {
    if (contact_.is_null()) return true;
    const auto state = measured_state();
    const auto &result = state.at("contact_exit");
    const auto phase = result.at("phase").get<std::string>();
    const Ns stamp = result.at("t_ns");
    return result.at("request_id") == contact_.at("request_id") && stamp >= contact_.at("started_ns").get<Ns>() &&
      stamp <= observations_.receipt_ros_ns && observations_.receipt_ros_ns-stamp <= static_cast<Ns>(max_age_s_*1e9) &&
      (phase == "succeeded" || phase == "cancelled" || phase == "rejected" || phase == "failed") &&
      result.value("settled", false) && result.value("no_late_work", false) && quiescent();
  } catch (...) { return false; }
}
} // namespace harness::observation
