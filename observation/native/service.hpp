#pragma once

#include "common.hpp"

namespace harness::observation {

void set_service_executable(const fs::path &path);
const fs::path &service_executable();
Json service_command(const Options &options);
Json start_service_after_connect();
Json stop_service_if_running();
// Background model delivery must not restart an armed observer or interrupt
// retained recovery/fallback authority. Unknown runtime state waits for receipt.
inline bool critic_reconfiguration_allowed(const Json &profile, const Json &state) {
  try {
    if (!profile.is_object() || !state.is_object() || !profile.value("installed_deployment", Json(nullptr)).is_null()) return false;
    const auto runtime = state.value("runtime", Json(nullptr));
    if (!runtime.is_object() || runtime.value("authority_retained", true) ||
        runtime.at("profile_revision") != profile.at("revision")) return false;
    const auto status = runtime.value("status", "");
    return status == "disabled" || status == "blocked";
  } catch (...) { return false; }
}

} // namespace harness::observation
