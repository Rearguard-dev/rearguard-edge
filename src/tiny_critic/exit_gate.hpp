#pragma once
#include <cmath>
#include <cstdint>
namespace harness::tiny {
// Unknown/stale evidence resets persistence. No allocation or model execution.
class ExitGate {
  std::int64_t since_ = -1, previous_ = -1, epoch_ = -1;
public:
  void reset() noexcept { since_ = previous_ = epoch_ = -1; }
  bool update(bool eligible, double score, double threshold,
              std::int64_t evidence_ns, std::int64_t now_ns,
              std::int64_t epoch, std::int64_t max_age_ns,
              std::int64_t persistence_ns) noexcept {
    if (epoch != epoch_ || now_ns < previous_) since_ = -1;
    epoch_ = epoch; previous_ = now_ns;
    const bool met = eligible && std::isfinite(score) && std::isfinite(threshold) &&
      now_ns >= 0 && evidence_ns >= 0 && evidence_ns <= now_ns && max_age_ns > 0 &&
      persistence_ns >= 0 && now_ns - evidence_ns <= max_age_ns && score >= threshold;
    if (!met) { since_ = -1; return false; }
    if (since_ < 0) since_ = now_ns;
    return now_ns - since_ >= persistence_ns;
  }
};
}
