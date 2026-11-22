#pragma once
#include "common.hpp"
#include <optional>
#include <random>

namespace harness::observation {
// Selection only. MCAP export and networking belong to the supervised uploader.
class ExecutionSampler {
  fs::path root_;
  Json policy_, ledger_;
  std::mt19937 random_;
  std::vector<double> anchor_, target_;
  Ns anchor_at_ = -1, last_joint_ = -1, activity_at_ = -1, motion_at_ = -1;
  Ns last_tick_ = -1, last_end_ = -1;
  double eligible_s_ = 0, next_s_ = 0;
  int lock_ = -1;
  std::string state_ = "waiting_for_activity";
  void next();
public:
  explicit ExecutionSampler(fs::path root, unsigned seed = std::random_device{}());
  ~ExecutionSampler();
  ExecutionSampler(const ExecutionSampler &) = delete;
  ExecutionSampler &operator=(const ExecutionSampler &) = delete;
  void reset();
  void joints(const std::vector<double> &positions, Ns monotonic);
  void command(const std::vector<double> &target, Ns monotonic);
  std::optional<Json> select(Ns ros, Ns monotonic, Ns wall, const Json &snapshot,
                             const Json &observation);
  Json status() const;
};
// Strict receipt-clock coverage. Header stamps remain in the MCAP unchanged.
bool sample_coverage(const Json &binding, const Json &coverage, Ns start, Ns end);
} // namespace harness::observation
