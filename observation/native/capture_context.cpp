#include "capture_context.hpp"
namespace harness::observation {
Json freeze_capture_context(const Json &profile, const Json &configured) {
  Json captured;
  const auto &binding = profile.at("binding");
  if (configured.is_object()) {
    require(configured.value("schema_version", 0) == 1, "unsupported capture context schema");
    require(configured.at("profile_id") == profile.at("profile_id"), "capture context profile mismatch");
    require(configured.at("binding_hash") == profile.at("binding_hash"), "capture context binding mismatch");
    require(configured.at("profile_revision") == profile.at("revision"), "capture context profile revision changed");
    captured = configured;
  } else {
    const auto context = profile.value("critic_context", Json(nullptr));
    captured = {{"schema_version", 1}, {"source", "edge_profile"},
        {"profile_id", profile.at("profile_id")}, {"profile_revision", profile.at("revision")},
        {"binding_hash", profile.at("binding_hash")}, {"critic_context", context},
        {"status", context.is_object() ? "complete" : "partial"},
        {"missing_fields", context.is_object() ? Json::array() : Json::array({"task_context"})}};
  }
  const auto context = captured.value("critic_context", Json(nullptr));
  if (context.is_object()) {
    require(context.value("schema_version", 0) == 1, "unsupported captured critic context schema");
    required_text(context.at("task"), "captured task");
    require(context.at("embodiment").at("joint_names") == binding.at("joint_order"),
            "captured context joint names differ from binding");
    const auto interface = binding.value("command_interface", "");
    require(context.value("command_interface", "") ==
                (interface == "position" ? "POSITION" : interface == "velocity" ? "VELOCITY" : interface == "effort" ? "EFFORT" : ""),
            "captured command interface differs from binding");
    require(context.contains("cameras") && context.at("cameras").is_array(), "captured context needs camera list");
    for (const auto &camera : context.at("cameras")) {
      bool bound = false;
      for (const auto &topic : binding.at("topics"))
        if (topic.value("role", "") == "camera" && topic.at("name") == camera.at("name")) bound = true;
      require(bound, "captured context camera is not bound");
    }
    captured["critic_context_hash"] = digest(context);
  }
  captured["captured_edge_wall_ns"] = std::to_string(wall_ns());
  captured.erase("capture_hash");
  captured["capture_hash"] = digest(captured);
  return captured;
}
}
