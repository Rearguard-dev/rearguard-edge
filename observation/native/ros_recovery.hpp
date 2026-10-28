#pragma once

#include "controller_handover.hpp"
#include <memory>
#include <rclcpp/context.hpp>

namespace harness::observation {

// Non-realtime ROS 2 adapters. Both own a private executor thread so bounded
// service/action waits never try to recursively spin the observer executor.
class RosControllerAuthority final : public ControllerAuthority {
  struct Impl;
  std::unique_ptr<Impl> impl_;

public:
  explicit RosControllerAuthority(const std::shared_ptr<rclcpp::Context> &context);
  ~RosControllerAuthority() override;
  bool ready() const override;
  bool configure(const Json &binding, const Json &contract) override;
  bool acquire() override;
  bool recovery_owns_commands() override;
  bool release() override;
  bool nominal_owns_commands() override;
  bool hold(const std::string &reason) override;
};

class RosTrajectoryRecovery final : public ControllerAdapter {
  struct Impl;
  std::unique_ptr<Impl> impl_;

public:
  explicit RosTrajectoryRecovery(const std::shared_ptr<rclcpp::Context> &context);
  ~RosTrajectoryRecovery() override;
  bool ready() const override;
  bool prepare(const fs::path &artifact, const Json &binding,
               const Json &contract) override;
  bool begin() override;
  std::string poll() override;
  bool resume() override;
  bool hold(const std::string &reason) override;
};

} // namespace harness::observation
