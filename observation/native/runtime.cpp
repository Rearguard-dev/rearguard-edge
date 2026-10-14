#include "runtime.hpp"
#include <cmath>
namespace harness::observation {
void Runtime::validate_authorization(const Json &s) const {
  require(!deployment_id_.is_null() && s.at("installed_deployment") == deployment_id_,
          "installed deployment changed; explicit reactivation required");
  const auto &d = s.at("deployments").at(deployment_id_.get<std::string>());
  const auto &b = s.at("bundles").at(d.at("bundle_id").get<std::string>());
  const auto &f = s.at("failures").at(d.at("failure_id").get<std::string>());
  require(!proposition_id_.empty() && f.value("proposition_id", "") == proposition_id_,
          "accepted failure proposition required");
  const auto &p = s.at("propositions").at(proposition_id_);
  const auto &link = p.at("recovery");
  require(link.is_object() && link.at("bundle_id") == b.at("id") &&
              link.at("content_hash") == b.at("content_hash"),
          "proposition requires an attached approved recovery");
  require(b.at("status") == "approved" && b.at("failure_id") == d.at("failure_id") &&
              b.at("guidance_revision") == f.at("guidance_revision") &&
              b.at("content_hash") == deployment_content_hash_ &&
              d.at("content_hash") == deployment_content_hash_ &&
              digest(b.at("content")) == deployment_content_hash_ &&
              b.at("content").at("binding_hash") == s.at("binding_hash"),
          "recovery approval is stale or does not match deployment");
}
Json Runtime::activate() {
  require(status == "disabled" || status == "blocked",
          "deactivate runtime before activation");
  if (!critic_.ready() || !controller_.ready()) {
    status = "blocked";
    throw std::runtime_error("enforcement unavailable: critic inference and "
                             "recovery controller adapters are required");
  }
  const auto s = store_.snapshot();
  require(!s["installed_deployment"].is_null(),
          "no approved deployment installed");
  const auto &d =
      s["deployments"].at(s["installed_deployment"].get<std::string>());
  const auto &b = s["bundles"].at(d["bundle_id"].get<std::string>());
  require(b["status"] == "approved", "deployment not approved");
  deployment_id_ = d.at("id");
  failure_id_ = d.at("failure_id");
  deployment_content_hash_ = d.at("content_hash").get<std::string>();
  proposition_id_ = s.at("failures").at(failure_id_.get<std::string>())
                        .value("proposition_id", "");
  validate_authorization(s);
  const auto &content = b["content"];
  require(
      critic_.load(store_.verify_artifact(content["critic"], "critic_model"),
                   s["binding"], content["contract"]),
      "critic adapter refused deployment");
  require(controller_.prepare(
              store_.verify_artifact(content["recovery"], "recovery_model"),
              s["binding"], content["contract"]),
          "controller adapter refused deployment");
  contract_ = content["contract"];
  deployment_id_ = d["id"];
  failure_id_ = d["failure_id"];
  deployment_content_hash_ = d.value("content_hash", "");
  enforcement_event_id_.clear();
  enforcement_started_wall_ns_ = 0;
  last_match_ = false;
  authority_requested_ = false;
  status = "nominal";
  return snapshot();
}
Json Runtime::snapshot() const {
  return {{"status", status},
          {"deployment_id", deployment_id_},
          {"proposition_id", proposition_id_},
          {"critic_ready", critic_.ready()},
          {"controller_ready", controller_.ready()},
          {"enforcement_event_id", enforcement_event_id_.empty()
                                       ? Json(nullptr) : Json(enforcement_event_id_)},
          {"recovery_attempt",
           status == "recovering" ? Json(retries_ + 1) : Json(nullptr)}};
}
void Runtime::record_enforcement(const std::string &outcome,
                                 const Json &detail, bool terminal) {
  if (enforcement_event_id_.empty()) {
    enforcement_event_id_ = "enforcement-" + unique_id();
    enforcement_started_wall_ns_ = wall_ns();
  }
  Json record = {{"schema_version", 1},
                 {"event_id", enforcement_event_id_},
                 {"profile_id", store_.snapshot()["profile_id"]},
                 {"deployment_id", deployment_id_},
                 {"failure_id", failure_id_},
                 {"mode", "active"},
                 {"outcome", outcome},
                 {"started_wall_ns", enforcement_started_wall_ns_},
                 {"ended_wall_ns", terminal ? Json(wall_ns()) : Json(nullptr)},
                 {"integrity", {{"deployment_content_hash", deployment_content_hash_}}},
                 {"record", detail}};
  store_.record_enforcement(record);
  if (terminal) {
    enforcement_event_id_.clear();
    enforcement_started_wall_ns_ = 0;
  }
}
Json Runtime::fallback(const std::string &reason) {
  // Monitoring failures do not authorize actuator intervention. Only a matched,
  // approved proposition may initiate authority acquisition (including hold).
  if (!authority_requested_) {
    status = "blocked";
    auto result = snapshot();
    result.update({{"kind", "blocked"}, {"reason", reason}});
    return result;
  }
  bool held = false;
  try {
    held = controller_.hold(reason);
  } catch (...) {
  }
  status = held ? "held" : "fault_unconfirmed_hold";
  auto result = snapshot();
  result.update(
      {{"kind", "fallback"}, {"reason", reason}, {"hold_acknowledged", held}});
  if (!enforcement_event_id_.empty())
    record_enforcement(reason.find("timeout") != std::string::npos
                           ? "timed_out"
                           : reason == "runtime deactivated" ? "aborted" : "fallback",
                       result, true);
  return result;
}
Json Runtime::deactivate() {
  if (status != "disabled" && status != "blocked") {
    auto result = fallback("runtime deactivated");
    if (status == "fault_unconfirmed_hold")
      return result;
  }
  status = "disabled";
  deployment_id_ = nullptr;
  failure_id_ = nullptr;
  deployment_content_hash_.clear();
  return snapshot();
}
Json Runtime::step(const Observations &observations, double now, bool fresh) {
  if (status != "nominal" && status != "recovering")
    return nullptr;
  auto event = [&](const char *kind) {
    auto result = snapshot();
    result["kind"] = kind;
    return result;
  };
  try {
    validate_authorization(store_.snapshot());
    if (!fresh || !std::isfinite(now))
      return fallback("observations or clock stale");
    if (status == "recovering") {
      if (now >= deadline_)
        return fallback("recovery timeout");
      const auto outcome = controller_.poll();
      if (outcome == "succeeded") {
        if (!controller_.resume())
          return fallback("nominal reentry not acknowledged");
        authority_requested_ = false;
        status = "nominal";
        auto result = event("recovery_completed");
        record_enforcement("completed", result, true);
        return result;
      }
      if (outcome == "failed") {
        if (retries_ < contract_["retry_budget"].get<int>()) {
          ++retries_;
          if (controller_.begin()) {
            deadline_ = now + contract_["timeout_ms"].get<double>() / 1000;
            return event("recovery_retry");
          }
        }
        return fallback("recovery failed");
      }
      if (outcome != "running")
        return fallback("invalid controller response");
      return nullptr;
    }
    const auto detection = critic_.evaluate(observations);
    if (detection && (!std::isfinite(detection->confidence) ||
                      detection->confidence < 0 || detection->confidence > 1))
      return fallback("invalid critic confidence");
    const bool match =
        detection &&
        detection->source == "trained_critic" &&
        detection->deployment_validated &&
        detection->deployment_id == deployment_id_.get<std::string>() &&
        detection->proposition_id == proposition_id_ &&
        detection->evidence_class ==
            contract_["evidence_class"].get<std::string>() &&
        detection->confidence >= contract_["confidence_min"].get<double>();
    const bool rising = match && !last_match_;
    last_match_ = match;
    if (rising) {
      authority_requested_ = true;
      if (!controller_.begin())
        return fallback("recovery authority not acknowledged");
      status = "recovering";
      retries_ = 0;
      deadline_ = now + contract_["timeout_ms"].get<double>() / 1000;
      auto result = event("recovery_started");
      record_enforcement("unknown", result, false);
      return result;
    }
  } catch (const std::exception &error) {
    return fallback(std::string("adapter error: ") + error.what());
  }
  return nullptr;
}
} // namespace harness::observation
