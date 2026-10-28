#pragma once
#include "runtime.hpp"

namespace harness::observation {
// Implemented by the robot integration, outside the real-time update loop.
// Switch calls must have bounded timeouts. Ownership queries must inspect the
// actual command interfaces, not just service success or controller names.
struct ControllerAuthority {
  virtual ~ControllerAuthority() = default;
  virtual bool ready() const = 0;
  // Bind authority to the exact approved deployment. Implementations must
  // reject ambiguous controller names or command-interface sets.
  virtual bool configure(const Json &binding, const Json &contract) = 0;
  virtual bool acquire() = 0;
  virtual bool recovery_owns_commands() = 0;
  virtual bool release() = 0;
  virtual bool nominal_owns_commands() = 0;
  // Retain exclusive fallback authority, even after a partially failed switch.
  // This is robot-specific: zero effort is not a general safe hold.
  virtual bool hold(const std::string &reason) = 0;
};

// The executor computes recovery commands; authority selects who can emit them.
// Runtime is the approval boundary and must be the only caller of begin().
class SwitchingRecoveryController final : public ControllerAdapter {
  ControllerAuthority &authority_;
  ControllerAdapter &executor_;
  enum class State { unprepared, nominal, acquiring, recovering, releasing, held };
  State state_ = State::unprepared;

public:
  SwitchingRecoveryController(ControllerAuthority &authority,
                              ControllerAdapter &executor)
      : authority_(authority), executor_(executor) {}
  // Before prepare(), the deployment-specific controller-manager endpoint is
  // intentionally unknown. Authority readiness is checked after configure().
  bool ready() const override { return executor_.ready(); }
  bool prepare(const fs::path &artifact, const Json &binding,
               const Json &contract) override {
    if (state_ != State::unprepared && state_ != State::nominal) return false;
    state_ = State::unprepared;
    if (!authority_.configure(binding, contract) || !authority_.ready() ||
        !executor_.ready() ||
        !executor_.prepare(artifact, binding, contract)) return false;
    state_ = State::nominal;
    return true;
  }
  bool begin() override {
    if (state_ == State::recovering) {
      // Bounded retries retain authority; never switch through nominal.
      return authority_.recovery_owns_commands() && executor_.begin();
    }
    if (state_ != State::nominal) return false;
    state_ = State::acquiring;
    if (!authority_.acquire() || !authority_.recovery_owns_commands()) return false;
    if (!executor_.begin()) return false;
    state_ = State::recovering;
    return true;
  }
  std::string poll() override {
    if (state_ != State::recovering || !authority_.recovery_owns_commands())
      return "authority_lost";
    return executor_.poll();
  }
  bool resume() override {
    if (state_ != State::recovering || !authority_.recovery_owns_commands()) return false;
    // Executor resume acknowledges completion predicates, discards stale work,
    // and prepares nominal reentry. It must not itself change command ownership.
    if (!executor_.resume()) return false;
    state_ = State::releasing;
    if (!authority_.release() || !authority_.nominal_owns_commands()) return false;
    state_ = State::nominal;
    return true;
  }
  bool hold(const std::string &reason) override {
    if (state_ == State::unprepared || state_ == State::nominal) return false;
    state_ = State::held;
    return authority_.hold(reason);
  }
};
} // namespace harness::observation
