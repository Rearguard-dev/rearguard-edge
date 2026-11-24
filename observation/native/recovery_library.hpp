#pragma once
#include "runtime.hpp"

namespace harness::observation {

enum class RecoveryAction { pause_reobserve, retreat_checkpoint,
                            reset_end_effector, unload_disengage };
struct RecoveryStage {
  std::string id, capability, implementation_hash, entry, completion;
  RecoveryAction action;
  double timeout_s = 0;
  Json parameters;
};
struct RecoveryRecipe {
  std::string id, version, readiness, rearm;
  double max_age_s = 0, stability_s = 0, handback_timeout_s = 0;
  std::vector<RecoveryStage> stages;
  Json manifest;
};
// Pure deterministic validation; capability-specific parameters/predicates are
// additionally validated by the bound integration before activation.
RecoveryRecipe compile_recovery_recipe(const Json &, const Json &binding,
                                       const Json &contract);

struct RecoveryEvidence {
  bool observable = false, satisfied = false;
  Ns epoch = -1, t_ns = -1;
  std::string state_revision;
};

// Trusted, configuration-specific adapter. Methods run on the observer worker,
// never a realtime controller update. start/poll/preparation must not wait on
// services: enqueue work and report pending until acknowledged. Implementations
// own their worker and must keep all command paths inside ControllerAuthority.
struct RecoveryLibraryPort {
  virtual ~RecoveryLibraryPort() = default;
  virtual bool ready() const = 0;
  // Validate exact capabilities, hashes, resources, parameters, predicates and
  // client protocol. Unknown/unsupported values MUST return false.
  virtual bool configure(const RecoveryRecipe &, const Json &binding,
                         const Json &contract) = 0;
  virtual void observe(const Observations &) = 0;
  virtual RecoveryEvidence test(const std::string &predicate) = 0;
  virtual bool start(const RecoveryStage &) = 0;
  virtual std::string poll() = 0; // running, succeeded, failed
  // Physical readiness also establishes terminal/cancelled goals, stopped or
  // retained tool state, payload/contact/clearance and no late recovery work.
  virtual bool quiescent() = 0;
  // Optional measured checkpoint/payload/contact/tool facts for client context.
  // Unknown values must stay unknown; this is not a task-success judgement.
  virtual Json state_report() const { return nullptr; }
  // Invalidate queued nominal commands before exposing the transaction to the
  // client. Client acknowledgement must come through the gated preparation path.
  virtual bool prepare_client(const Json &transaction, const Json &report) = 0;
  virtual Json client_acknowledgement() = 0;
  // Check dimensions, units, limits, current state compatibility and command
  // epoch; retain exactly this candidate behind the command gate for transfer.
  virtual bool validate_candidate(const Json &transaction, const Json &command) = 0;
  // Stage the accepted candidate for the authority coordinator's transfer. This
  // MUST NOT release command exclusion or publish around the authority gate.
  virtual bool commit_candidate(const Json &transaction) = 0;
  virtual bool cancel(const std::string &reason) = 0;
};

// Wrap this executor with SwitchingRecoveryController. This class never releases
// authority, chooses a task retry, or substitutes a guessed capability.
class LibraryRecoveryController final : public ControllerAdapter {
  RecoveryLibraryPort &port_;
  RecoveryRecipe recipe_;
  enum class State { unprepared, idle, running, completed, preparing, prepared, committed,
                     returned, failed, held };
  State state_ = State::unprepared;
  Observations observations_;
  double now_ = -1, stage_deadline_ = 0, handback_deadline_ = 0;
  Ns stable_since_ = -1, rearm_since_ = -1;
  Ns stage_started_ns_ = -1, completed_ns_ = -1;
  std::size_t stage_ = 0;
  bool observed_ = false, action_completed_ = false, rearmed_ = false;
  Json transaction_, report_;
  std::string stable_revision_;
  bool evidence(const RecoveryEvidence &) const;
  bool physical_ready();
  bool start_stage();
  void fail(const std::string &);

public:
  explicit LibraryRecoveryController(RecoveryLibraryPort &port) : port_(port) {}
  bool ready() const override { return port_.ready(); }
  bool prepare(const fs::path &, const Json &, const Json &) override;
  void observe(const Observations &, double) override;
  bool begin() override;
  bool can_begin() const override;
  std::string poll() override;
  Handback prepare_handback(const Json &) override;
  bool resume() override;
  void handback_confirmed() override;
  bool hold(const std::string &) override;
  Json telemetry() const override { return report_; }
};
} // namespace harness::observation
