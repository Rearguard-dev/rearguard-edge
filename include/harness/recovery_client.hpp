#pragma once
#include <nlohmann/json.hpp>
#include <cmath>
#include <cstdint>
#include <optional>
#include <set>
#include <string>
#include <stdexcept>
#include <vector>

namespace harness {
// Embed this gate at the policy's sole nominal command output. In-flight policy
// work carries command_epoch(); old generations cannot enter its command queue.
// The host resets/reobserves its policy on prepare before calling stage().
class RecoveryClientGate {
  using Json = nlohmann::json;
  std::string binding_hash_, provider_, epoch_;
  std::vector<double> lower_, upper_;
  Json transaction_, candidate_;
  std::optional<Json> pending_;
  std::set<std::string> retired_;
  bool nominal_ = true, committed_ = false;
  bool command(const Json &value) const {
    if (!value.is_array() || value.size() != lower_.size()) return false;
    for (std::size_t i = 0; i < lower_.size(); ++i)
      if (!value[i].is_number() || !std::isfinite(value[i].get<double>()) ||
          value[i].get<double>() < lower_[i] || value[i].get<double>() > upper_[i]) return false;
    return true;
  }
public:
  RecoveryClientGate(std::string binding_hash, std::string provider, std::string initial_epoch,
                     std::vector<double> lower, std::vector<double> upper)
      : binding_hash_(std::move(binding_hash)), provider_(std::move(provider)), epoch_(std::move(initial_epoch)),
        lower_(std::move(lower)), upper_(std::move(upper)) {
    if (lower_.empty() || lower_.size() != upper_.size() || epoch_.empty()) throw std::runtime_error("invalid recovery client gate");
    for (std::size_t i = 0; i < lower_.size(); ++i)
      if (!std::isfinite(lower_[i]) || !std::isfinite(upper_[i]) || lower_[i] >= upper_[i]) throw std::runtime_error("invalid recovery client limits");
  }
  const std::string &command_epoch() const { return epoch_; }
  bool blocked() const { return !nominal_; }
  // Returns true only when a newly accepted prepare requires policy reset.
  bool handle(const Json &request) {
    try {
      if (request.at("schema_version") != 1 || request.at("binding_hash") != binding_hash_) return false;
      const auto kind = request.at("kind").get<std::string>();
      const auto &transaction = request.at("transaction");
      if (kind == "cancel") {
        // A transaction-free cancellation is allowed before handback preparation.
        if (!transaction.is_null() && transaction != transaction_) return false;
        if (transaction_.is_object()) retired_.insert(transaction_.at("transaction_id").get<std::string>());
        nominal_ = committed_ = false; candidate_ = transaction_ = nullptr; pending_.reset(); return false;
      }
      if (kind == "prepare") {
        const auto id = transaction.at("transaction_id").get<std::string>();
        const auto epoch = transaction.at("command_epoch").get<std::string>();
        if (id.empty() || epoch.empty() || retired_.contains(id) || retired_.size() >= 4096 || transaction == transaction_) return false;
        if (transaction_.is_object()) retired_.insert(transaction_.at("transaction_id").get<std::string>());
        transaction_ = transaction; epoch_ = epoch;
        nominal_ = committed_ = false; candidate_ = nullptr; pending_.reset(); return true;
      }
      if (transaction_.is_null() || transaction != transaction_ || candidate_.is_null()) return false;
      if (kind == "commit" && !nominal_) committed_ = true;
      if (kind == "authority_transferred" && committed_ && !nominal_) {
        nominal_ = true; pending_ = candidate_; retired_.insert(transaction_.at("transaction_id").get<std::string>());
      }
    } catch (...) {}
    return false;
  }
  bool stage(const std::string &epoch, const Json &candidate, std::int64_t observation_ns) {
    if (nominal_ || transaction_.is_null() || epoch != epoch_ || !command(candidate) ||
        observation_ns < transaction_.at("prepared_after_ns").get<std::int64_t>()) return false;
    if (committed_) return candidate == candidate_;
    candidate_ = candidate; return true;
  }
  Json acknowledgement(std::int64_t now_ns, std::int64_t physical_epoch) const {
    if (transaction_.is_null() || candidate_.is_null() || nominal_ || transaction_.at("epoch") != physical_epoch) return nullptr;
    return {{"binding_hash", binding_hash_}, {"provider_id", provider_}, {"epoch", physical_epoch}, {"t_ns", now_ns},
      {"transaction", transaction_}, {"phase", "staged"}, {"queue_invalidated", true}, {"command", candidate_}};
  }
  bool submit(const std::string &epoch, const Json &value) {
    if (!nominal_ || epoch != epoch_ || !command(value)) return false;
    pending_ = value; return true;
  }
  std::optional<Json> take() {
    if (!nominal_) return std::nullopt;
    auto value = pending_; pending_.reset(); return value;
  }
};
}
