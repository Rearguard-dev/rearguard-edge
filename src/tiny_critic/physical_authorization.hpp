#pragma once
#include "common.hpp"
#include <algorithm>
#include <cmath>
namespace harness::tiny {
// Called only from CriticAdapter::load after Runtime verifies the installed,
// approved bundle. A shadow model selection itself never enables enforcement.
inline observation::Json authorize_physical_critic(const observation::Json &reference,
    const observation::Json &selection, const observation::Json &manifest,
    const observation::Json &binding, const observation::Json &contract, const std::string &manifest_hash) {
  using observation::require;
  const auto &activation = contract.at("runtime_activation");
  require(reference.at("schema_version") == 1 && reference.at("kind") == "physical_critic_reference", "physical critic reference required");
  require(reference.at("model_id") == selection.at("model_id") && reference.at("profile_id") == selection.at("profile_id") &&
          reference.at("manifest_sha256") == manifest_hash && reference.at("binding_hash") == observation::digest(binding), "approved physical model identity differs");
  require(activation.at("profile_id") == selection.at("profile_id") && activation.at("binding_hash") == reference.at("binding_hash"), "physical approval scope differs");
  const auto id = reference.at("proposition_id").get<std::string>();
  require(activation.at("proposition_id") == id && contract.at("evidence_class") == id, "approved physical proposition differs");
  const auto &active = selection.at("active_proposition_ids");
  require(std::find(active.begin(), active.end(), observation::Json(id)) != active.end(), "physical proposition not selected");
  bool supported = false;
  for (const auto &p : manifest.at("propositions"))
    if (p.at("id") == id && observation::digest(p) == reference.at("proposition_hash").get<std::string>()) supported = true;
  require(supported, "physical proposition definition changed");
  const auto &gate = contract.at("exit_gate");
  require(gate.at("kind") == "proposition_clear" && gate.at("proposition_id") == id && reference.at("exit_gate") == gate,
          "physical critic requires an explicitly approved proposition-clear exit gate");
  require(std::isfinite(contract.at("confidence_min").get<double>()) && contract.at("confidence_min").get<double>() > 0 && contract.at("confidence_min").get<double>() <= 1,
          "approved entry threshold required");
  observation::required_text(activation.at("deployment_id"), "physical deployment id");
  observation::required_text(activation.at("content_hash"), "physical approval hash");
  return activation;
}
}
