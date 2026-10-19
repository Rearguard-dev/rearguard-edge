#pragma once

#include "common.hpp"

#include <optional>

namespace harness::observation {

// Select and freeze an observation binding from the ROS graph captured during
// setup. Runtime assignment is deliberately structural: cloud embodiment names
// are metadata, never selectors for bundled robot presets.
std::optional<Ns> select_observation_domain(const Json &inventory);
Json compile_observation_binding(const Json &inventory, Ns domain_id,
                                 const std::string &robot_label);

} // namespace harness::observation
