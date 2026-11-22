#pragma once
#include "common.hpp"
#include <functional>

namespace harness::observation {
// Runs in the cloud-sync process, never in a ROS callback or command gate.
class ModelCache {
  fs::path root_;
  Json active_ = nullptr;
public:
  using Download = std::function<void(const Json &, const fs::path &)>;
  using Admit = std::function<bool(const fs::path &, const Json &)>;
  explicit ModelCache(fs::path);
  Json refresh(const Json &resolution, const Download &, const Admit &);
  const Json &active() const { return active_; }
};
}
