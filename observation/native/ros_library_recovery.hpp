#pragma once
#include "runtime.hpp"
#include <rclcpp/context.hpp>
namespace harness::observation {
std::unique_ptr<ControllerAdapter> make_ros_recovery_executor(const std::shared_ptr<rclcpp::Context> &);
// Identity comes from the installed executable, not from recipe declarations.
std::map<std::string, std::string> installed_recovery_implementations(const fs::path &executable);
}
