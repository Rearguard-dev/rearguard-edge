#include "recovery_library.hpp"
#include "controller_handover.hpp"
#include <iostream>
#include <limits>
#include <set>

using namespace harness::observation;
namespace {
int checks = 0;
void check(bool value, const char *message) {
  ++checks;
  require(value, message);
}
template <typename F> void rejects(F fn) {
  try { fn(); } catch (const std::exception &) { ++checks; return; }
  throw std::runtime_error("invalid recipe accepted");
}
struct Authority final : ControllerAuthority {
  bool locked = false, lost = false, release_ok = true, fallback_ok = true;
  int acquisitions = 0, releases = 0, holds = 0;
  bool ready() const override { return true; }
  bool configure(const Json &, const Json &) override { return true; }
  bool acquire() override { ++acquisitions; locked = true; return true; }
  bool recovery_owns_commands() override { return locked && !lost; }
  bool release() override { ++releases; locked = false; return release_ok; }
  bool nominal_owns_commands() override { return !locked && release_ok; }
  bool hold(const std::string &) override { ++holds; locked = fallback_ok; return fallback_ok; }
};
struct Port final : RecoveryLibraryPort {
  Authority &authority;
  Observations observations;
  bool supported = true, observable = true, physical = true, quiet = true,
       completion = true, entry = true, cleared = false, stale = false,
       wrong_epoch = false, candidate_ok = true, cancel_ok = true;
  int starts = 0, preparations = 0, commits = 0, cancels = 0;
  std::string outcome = "succeeded", revision = "state-1";
  Json request, ack;
  explicit Port(Authority &a) : authority(a) {}
  bool ready() const override { return true; }
  bool configure(const RecoveryRecipe &recipe, const Json &, const Json &) override {
    for (const auto &stage : recipe.stages)
      if (stage.capability != "qualified_mock_v1") return false;
    return supported;
  }
  void observe(const Observations &value) override { observations = value; }
  RecoveryEvidence test(const std::string &predicate) override {
    bool met = predicate == "physical" ? physical : predicate == "complete" ? completion :
               predicate == "entry" ? entry : predicate == "rearm" ? cleared : false;
    return {observable, met, observations.epoch + (wrong_epoch ? 1 : 0),
            stale ? observations.receipt_ros_ns - 200000000 : observations.receipt_ros_ns,
            revision};
  }
  bool start(const RecoveryStage &) override {
    check(authority.locked, "stage must run with VLA locked");
    ++starts; return true;
  }
  std::string poll() override { return outcome; }
  bool quiescent() override { return quiet; }
  bool prepare_client(const Json &transaction, const Json &report) override {
    check(authority.locked, "client preparation must retain VLA lock");
    check(report.at("action_completed") == true && report.at("physical_ready") == true,
          "client preparation needs measured physical readiness");
    ++preparations; request = transaction; return true;
  }
  Json client_acknowledgement() override { return ack; }
  bool validate_candidate(const Json &transaction, const Json &) override {
    return candidate_ok && transaction == request;
  }
  bool commit_candidate(const Json &transaction) override {
    check(authority.locked, "candidate commit must retain VLA lock");
    ++commits; return candidate_ok && transaction == request;
  }
  bool cancel(const std::string &) override { ++cancels; return cancel_ok; }
  void acknowledge() {
    ack = {{"transaction", request}, {"ready", true}, {"command", {0.1, 0.2}}};
  }
};
struct Fixture {
  fs::path path = fs::temp_directory_path() / ("recovery-library-" + unique_id() + ".json");
  Authority authority;
  Port port{authority};
  LibraryRecoveryController executor{port};
  SwitchingRecoveryController controller{authority, executor};
  Json binding = {{"recovery_controller", {{"id", "rearguard_recovery"}}}};
  Json contract = {{"controller_id", "rearguard_recovery"}, {"timeout_ms", 6000}, {"retry_budget", 0}};
  Json context = {{"episode_id", "episode-1"}, {"deployment_id", "deployment-1"},
                  {"deployment_content_hash", "approved-content"}};
  Json manifest;
  Fixture() {
    manifest = {{"schema_version", 1}, {"kind", "recovery_action_library"},
                {"id", "assembly_exit"}, {"version", "1"},
                {"binding_hash", digest(binding)}, {"controller_id", "rearguard_recovery"},
                {"rearm_predicate", "rearm"}, {"stages", Json::array()},
                {"handback", {{"readiness_predicate", "physical"}, {"max_age_ms", 100},
                              {"stability_ms", 100}, {"timeout_ms", 2000}}}};
    int id = 0;
    for (const auto *action : {"pause_reobserve", "unload_disengage", "reset_end_effector", "retreat_checkpoint"})
      manifest["stages"].push_back({{"id", std::to_string(++id)}, {"action", action},
          {"capability", "qualified_mock_v1"}, {"implementation_hash", std::string(64, 'a')},
          {"parameters", Json::object()}, {"entry_predicate", "entry"},
          {"completion_predicate", "complete"}, {"timeout_ms", 500}});
  }
  ~Fixture() { fs::remove(path); }
  bool prepare() {
    atomic_json(path, manifest);
    return controller.prepare(path, binding, contract);
  }
  void tick(double now) {
    Observations obs;
    obs.epoch = 1; obs.receipt_ros_ns = static_cast<Ns>(now * 1e9);
    controller.observe(obs, now);
  }
  void complete() {
    check(prepare(), "recipe should prepare");
    tick(1);
    check(controller.begin(), "recipe should acquire authority");
    for (int i = 0; i < 4; ++i) {
      tick(1.01 + static_cast<double>(i) * 0.01);
      check(controller.poll() == (i == 3 ? "succeeded" : "running"), "stage sequence");
    }
    check(authority.acquisitions == 1 && authority.releases == 0 && port.starts == 4,
          "composition must acquire once and never unlock between stages");
  }
  void start_preparation() {
    check(controller.prepare_handback(context) == ControllerAdapter::Handback::pending,
          "readiness stability required");
    tick(1.2);
    check(controller.prepare_handback(context) == ControllerAdapter::Handback::pending,
          "client acknowledgement required");
    check(port.preparations == 1, "client transaction should start");
  }
};
void schema_tests() {
  Fixture f;
  const auto recipe = compile_recovery_recipe(f.manifest, f.binding, f.contract);
  check(recipe.stages.size() == 4, "all four action types compile");
  const auto invalid = [&](const auto &edit) {
    auto manifest = f.manifest; edit(manifest);
    rejects([&] { compile_recovery_recipe(manifest, f.binding, f.contract); });
  };
  invalid([](Json &m) { m["kind"] = "unknown"; });
  invalid([](Json &m) { m["binding_hash"] = "other"; });
  invalid([](Json &m) { m["controller_id"] = "other"; });
  invalid([](Json &m) { m["stages"][0]["action"] = "retry_task"; });
  invalid([](Json &m) { m["stages"][0]["implementation_hash"] = "unversioned"; });
  invalid([](Json &m) { m["stages"][0]["timeout_ms"] = -1; });
  invalid([](Json &m) { m["stages"][0]["timeout_ms"] = 300000; });
  invalid([](Json &m) { m["stages"][0]["timeout_ms"] = std::numeric_limits<double>::infinity(); });
  invalid([](Json &m) { m["stages"][0]["id"] = "2"; });
  invalid([](Json &m) { m["stages"][0]["skip_on_failure"] = true; });
  invalid([](Json &m) { m["handback"]["max_age_ms"] = 0; });
  invalid([](Json &m) { m["handback"]["stability_ms"] = 2000; });
  invalid([](Json &m) { m["stages"] = Json::array(); });
  f.contract["retry_budget"] = 1;
  rejects([&] { compile_recovery_recipe(f.manifest, f.binding, f.contract); });
  f.contract["retry_budget"] = 0;
  f.manifest["stages"][0]["capability"] = "unsupported_tool";
  check(!f.prepare() && !f.controller.begin() && f.authority.acquisitions == 0,
        "missing capabilities refuse activation without taking control");
}
void handback_tests() {
  Fixture f;
  f.complete();
  check(!f.controller.resume() && f.authority.releases == 0,
        "action completion cannot bypass handback preparation");
  f.start_preparation();
  f.port.acknowledge();
  const auto valid = f.port.ack;
  for (const auto *key : {"episode_id", "deployment_id", "deployment_content_hash", "transaction_id",
                         "command_epoch", "state_revision", "epoch", "prepared_after_ns"}) {
    f.port.ack = valid;
    f.port.ack["transaction"][key] = "old";
    check(f.controller.prepare_handback(f.context) == ControllerAdapter::Handback::pending,
          "stale or wrong-generation acknowledgement must not unlock VLA");
  }
  f.port.ack = valid;
  f.port.ack["command"] = {std::numeric_limits<double>::quiet_NaN()};
  check(f.controller.prepare_handback(f.context) == ControllerAdapter::Handback::pending,
        "nonfinite command rejected");
  f.port.ack = valid;
  f.port.candidate_ok = false;
  check(f.controller.prepare_handback(f.context) == ControllerAdapter::Handback::pending,
        "out-of-envelope command rejected");
  f.port.candidate_ok = true;
  check(f.controller.prepare_handback(f.context) == ControllerAdapter::Handback::ready,
        "matching prepared command permits handback");
  check(f.authority.locked && f.authority.releases == 0, "readiness does not release authority");
  check(f.controller.resume() && f.authority.releases == 1 && !f.authority.locked,
        "coordinator returns control after readiness");
  check(f.controller.telemetry()["authority_handback_confirmed"] == true &&
        f.controller.telemetry()["task_outcome"] == "unknown", "task success is separate from handback");
  check(!f.controller.begin(), "same incident cannot consume another recipe execution");
  f.port.cleared = true;
  f.tick(1.3); f.tick(1.5);
  check(f.controller.begin(), "confirmed cleared incident permits a new episode");
}
void failure_tests() {
  for (int fault = 0; fault < 5; ++fault) {
    Fixture f; f.complete();
    if (fault == 0) f.port.physical = false;
    if (fault == 1) f.port.observable = false;
    if (fault == 2) f.port.stale = true;
    if (fault == 3) f.port.quiet = false;
    if (fault == 4) f.port.wrong_epoch = true;
    check(f.controller.prepare_handback(f.context) == ControllerAdapter::Handback::pending &&
          f.port.preparations == 0 && !f.controller.resume() && f.authority.releases == 0,
          "failed physical evidence must retain lock");
    f.tick(3.1);
    check(f.controller.prepare_handback(f.context) == ControllerAdapter::Handback::failed,
          "handback wait is bounded");
    check(f.controller.hold("timeout") && f.port.cancels == 1 && f.authority.holds == 1,
          "fallback cancels executor and retains authority");
  }
  {
    Fixture f; f.complete(); f.start_preparation(); f.port.acknowledge();
    check(f.controller.prepare_handback(f.context) == ControllerAdapter::Handback::ready, "prepared");
    f.port.revision = "changed-scene";
    check(!f.controller.resume() && f.authority.releases == 0, "scene change before release blocks transfer");
    check(f.controller.prepare_handback(f.context) == ControllerAdapter::Handback::pending, "scene requires stability again");
    f.tick(1.4);
    check(f.controller.prepare_handback(f.context) == ControllerAdapter::Handback::pending &&
          f.port.preparations == 2, "changed state invalidates old client acknowledgement");
  }
  {
    Fixture f; f.complete(); f.start_preparation(); f.port.acknowledge();
    check(f.controller.prepare_handback(f.context) == ControllerAdapter::Handback::ready, "prepared");
    f.authority.release_ok = false;
    check(!f.controller.resume() && f.controller.telemetry()["authority_handback_confirmed"] == false,
          "partial switch must not claim confirmed handback");
    f.port.cancel_ok = false;
    check(!f.controller.hold("partial switch") && f.authority.holds == 1,
          "cancellation failure must still attempt authority fallback");
  }
  {
    Fixture f; f.complete(); f.authority.lost = true;
    check(f.controller.poll() == "authority_lost" &&
          f.controller.prepare_handback(f.context) == ControllerAdapter::Handback::failed &&
          !f.controller.resume(), "ownership loss blocks client preparation and release");
  }
  for (int fault = 0; fault < 4; ++fault) {
    Fixture f; check(f.prepare(), "prepare"); f.tick(1);
    if (fault == 0) f.port.entry = false;
    if (fault == 0) check(!f.controller.begin() && f.port.starts == 0, "unknown entry refuses execution");
    else {
      check(f.controller.begin(), "begin");
      if (fault == 1) { f.port.outcome = "failed"; }
      if (fault == 2) { f.tick(1.6); }
      if (fault == 3) {
        Observations reset; reset.epoch = 2; reset.receipt_ros_ns = 1;
        f.controller.observe(reset, 1.1);
      }
      check(f.controller.poll() == "failed" && f.port.starts == 1,
            "failure, timeout or clock reset cannot skip to the next stage");
    }
  }
}
} // namespace
int main() {
  try {
    schema_tests(); handback_tests(); failure_tests();
    std::cout << checks << " recovery library checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}
