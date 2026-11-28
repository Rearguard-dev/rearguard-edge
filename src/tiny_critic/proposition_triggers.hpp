#pragma once
#include <cstdint>
#include <map>
#include <optional>
#include <string>

namespace harness::tiny {
// Track each proposition independently. Unknown evidence cannot clear a failure
// or allow another proposition to suppress a simultaneous incident.
class PropositionTriggers {
  struct State { bool positive = false, pending = false; std::int64_t last = -1, seen = -1; };
  std::map<std::string, State> states_;
public:
  void reset() { states_.clear(); }
  void update(const std::string &id, bool known, bool positive, std::int64_t stamp) {
    if (!known) { states_[id].pending = false; return; }
    auto &state = states_[id];
    if (stamp <= state.seen) return;
    state.seen = stamp;
    if (positive && !state.positive) state.pending = true;
    if (!positive) state.pending = false;
    state.positive = positive;
  }
  std::optional<std::string> next(std::int64_t now) {
    for (auto &[id, state] : states_) {
      if (now < state.seen || now-state.seen > 250000000LL) { state.pending = false; continue; }
      if (state.pending && (state.last < 0 || now-state.last >= 10000000000LL)) {
        state.pending = false; state.last = now; return id;
      }
    }
    return std::nullopt;
  }
};
} // namespace harness::tiny
