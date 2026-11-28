#pragma once
#include "recovery_library.hpp"
#include <map>
#include <set>

namespace harness::observation {
enum class RecoveryBackend { moveit_plan, joint_trajectory, nav2_pose, gripper_gap };
enum class ActionState { pending, succeeded, failed, cancelled };
struct ActionReply { ActionState state = ActionState::pending; Json result; };
// Transport must track late acceptance/results after cancellation. A request is
// terminal only after rejection or a terminal result, never a cancel request.
struct RecoveryActionTransport {
  virtual ~RecoveryActionTransport() = default;
  virtual bool available(RecoveryBackend, const std::string &endpoint) const = 0;
  virtual std::string send(RecoveryBackend, const std::string &, const Json &) = 0;
  virtual ActionReply poll(const std::string &ticket) = 0;
  virtual void cancel(const std::string &ticket) = 0;
};
// The workcell/client integration supplies measured predicates and the existing
// gated VLA protocol. Adapters never derive physical readiness from action success.
struct RecoveryEnvironment {
  virtual ~RecoveryEnvironment() = default;
  virtual bool configure(const RecoveryRecipe &, const Json &, const Json &) = 0;
  virtual void observe(const Observations &) = 0;
  virtual RecoveryEvidence test(const std::string &) = 0;
  virtual Json measured_state() const = 0;
  // measured_state: epoch, t_ns, scene_revision, arm_positions, arm_velocities,
  // disengaged, payload_modelled; optional tool facts remain explicit.
  virtual bool owns(const std::string &resource) const = 0;
  virtual bool quiescent() const = 0;
  virtual bool prepare_client(const Json &, const Json &) = 0;
  virtual Json client_acknowledgement() = 0;
  virtual bool validate_candidate(const Json &, const Json &) = 0;
  virtual bool commit_candidate(const Json &) = 0;
  virtual void handback_confirmed(const Json &) {}
  virtual bool cancel(const std::string &) { return true; }
  virtual bool start_contact(const RecoveryStage &) { return false; }
  virtual std::string poll_contact() { return "failed"; }
  virtual bool contact_quiescent() { return false; }
};

// Concrete recipe dispatch for MoveIt joint checkpoints, Nav2 pose checkpoints,
// measured-gap grippers and passive pause. Exact limits/endpoints come from the
// approved binding; robot family names never choose actuator topics or limits.
class AdapterRecoveryLibraryPort final : public RecoveryLibraryPort {
  RecoveryActionTransport &transport_;
  RecoveryEnvironment &environment_;
  // Trusted hashes supplied by the host's installed adapter registry, rather
  // than accepting an implementation identity declared only by a recipe.
  std::map<std::string, std::string> implementations_;
  std::set<std::string> resources_;
  Json config_, current_stage_, planning_state_;
  Observations observations_;
  RecoveryRecipe recipe_;
  std::string ticket_, resource_;
  enum class Phase { idle, paused, contacting, planning, executing, succeeded, failed };
  Phase phase_ = Phase::idle;
  bool configured_ = false, cancelling_ = false;
  bool contact_active_ = false;
  Ns stage_started_monotonic_ns_ = 0;
  double stage_timeout_s_ = 0;
  bool measured_arm_ready(const Json &) const;
  bool validate_plan(const Json &) const;
  bool validate_stage(const RecoveryStage &) const;
  bool terminal();
  bool resources_owned() const;
public:
  AdapterRecoveryLibraryPort(RecoveryActionTransport &, RecoveryEnvironment &,
                            std::map<std::string, std::string> implementations);
  bool ready() const override { return true; }
  bool configure(const RecoveryRecipe &, const Json &, const Json &) override;
  void observe(const Observations &) override;
  RecoveryEvidence test(const std::string &) override;
  bool start(const RecoveryStage &) override;
  std::string poll() override;
  bool quiescent() override;
  Json state_report() const override;
  bool prepare_client(const Json &, const Json &) override;
  Json client_acknowledgement() override;
  bool validate_candidate(const Json &, const Json &) override;
  bool commit_candidate(const Json &) override;
  void handback_confirmed(const Json &) override;
  bool cancel(const std::string &) override;
};
} // namespace harness::observation
