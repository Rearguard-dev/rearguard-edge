#pragma once
#include "runtime.hpp"

namespace harness::observation {
// Artifact format selects one executor. A refused library recipe is never
// retried as a legacy trajectory policy.
class RecoveryExecutorRouter final : public ControllerAdapter {
  ControllerAdapter &legacy_, &library_;
  ControllerAdapter *selected_ = nullptr;
public:
  RecoveryExecutorRouter(ControllerAdapter &legacy, ControllerAdapter &library)
      : legacy_(legacy), library_(library) {}
  bool ready() const override { return legacy_.ready() || library_.ready(); }
  bool prepare(const fs::path &artifact, const Json &binding, const Json &contract) override {
    try {
      const auto manifest = read_json(artifact);
      selected_ = nullptr;
      if (manifest.value("kind", "") == "recovery_action_library") selected_ = &library_;
      else if (!manifest.contains("kind") && manifest.contains("ros2_policy")) selected_ = &legacy_;
      return selected_ && selected_->ready() && selected_->prepare(artifact, binding, contract);
    } catch (...) { selected_ = nullptr; return false; }
  }
  bool begin() override { return selected_ && selected_->begin(); }
  bool can_begin() const override { return selected_ && selected_->can_begin(); }
  std::string poll() override { return selected_ ? selected_->poll() : "unavailable"; }
  bool resume() override { return selected_ && selected_->resume(); }
  bool hold(const std::string &reason) override { return selected_ && selected_->hold(reason); }
  void observe(const Observations &value, double now) override { if (selected_) selected_->observe(value, now); }
  Handback prepare_handback(const Json &context) override { return selected_ ? selected_->prepare_handback(context) : Handback::failed; }
  void handback_confirmed() override { if (selected_) selected_->handback_confirmed(); }
  Json telemetry() const override { return selected_ ? selected_->telemetry() : Json(nullptr); }
};
} // namespace harness::observation
