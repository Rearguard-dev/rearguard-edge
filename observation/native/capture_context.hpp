#pragma once
#include "common.hpp"
namespace harness::observation {
// Capture metadata is independent of the active inference configuration.
Json freeze_capture_context(const Json &profile, const Json &configured = nullptr);
}
