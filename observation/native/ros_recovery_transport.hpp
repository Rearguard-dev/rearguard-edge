#pragma once
#include "recovery_adapters.hpp"
#include <memory>
#include <rclcpp/context.hpp>

namespace harness::observation {
// Explicitly registered endpoints only. No controller switching, task retry or
// discovery by robot name. Uses a private ROS executor and asynchronous actions.
class RosRecoveryActionTransport final : public RecoveryActionTransport {
  struct Impl;
  std::unique_ptr<Impl> impl_;
public:
  RosRecoveryActionTransport(const std::shared_ptr<rclcpp::Context> &context,
      const std::vector<std::pair<RecoveryBackend, std::string>> &endpoints,
      bool use_sim_time = false);
  ~RosRecoveryActionTransport() override;
  bool available(RecoveryBackend, const std::string &) const override;
  std::string send(RecoveryBackend, const std::string &, const Json &) override;
  ActionReply poll(const std::string &) override;
  void cancel(const std::string &) override;
};
} // namespace harness::observation
