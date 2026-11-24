// Limits/poses here are synthetic test values, never hardware defaults.
#include "recovery_adapters.hpp"
#include "controller_handover.hpp"
#include <iostream>
#include <limits>
#include <set>
using namespace harness::observation;
namespace {
unsigned checks = 0;
void check(bool value, const char *message) { ++checks; require(value, message); }
struct Transport final : RecoveryActionTransport {
  struct Sent { RecoveryBackend backend; std::string endpoint; Json request; std::string id; };
  std::vector<Sent> sent;
  std::map<std::string, ActionReply> replies;
  std::set<RecoveryBackend> unavailable;
  unsigned cancels = 0;
  bool available(RecoveryBackend backend, const std::string &) const override { return !unavailable.contains(backend); }
  std::string send(RecoveryBackend backend, const std::string &endpoint, const Json &request) override {
    const auto id = std::to_string(sent.size());
    sent.push_back({backend, endpoint, request, id}); replies[id] = {}; return id;
  }
  ActionReply poll(const std::string &id) override { return replies.at(id); }
  void cancel(const std::string &) override { ++cancels; }
  void finish(ActionState state, Json result = Json::object()) { replies.at(sent.back().id) = {state, std::move(result)}; }
};
struct Environment final : RecoveryEnvironment {
  Json state, ack, transaction;
  Observations obs;
  std::set<std::string> owned = {"arm", "tool", "base"};
  bool qualified = true, completion = true, quiet = true;
  unsigned preparations = 0, commits = 0;
  bool configure(const RecoveryRecipe &, const Json &, const Json &) override { return qualified; }
  void observe(const Observations &value) override {
    obs = value; state["epoch"] = obs.epoch; state["t_ns"] = obs.receipt_ros_ns;
  }
  RecoveryEvidence test(const std::string &predicate) override {
    return {true, predicate != "complete" || completion, obs.epoch, obs.receipt_ros_ns, state.at("scene_revision")};
  }
  Json measured_state() const override { return state; }
  bool owns(const std::string &resource) const override { return owned.contains(resource); }
  bool quiescent() const override { return quiet; }
  bool prepare_client(const Json &t, const Json &) override { ++preparations; transaction = t; return true; }
  Json client_acknowledgement() override { return ack; }
  bool validate_candidate(const Json &t, const Json &) override { return t == transaction; }
  bool commit_candidate(const Json &t) override { ++commits; return t == transaction; }
};
struct Authority final : ControllerAuthority {
  unsigned acquisitions = 0, releases = 0;
  Environment &environment;
  bool locked = false;
  explicit Authority(Environment &e) : environment(e) {}
  bool ready() const override { return true; }
  bool configure(const Json &, const Json &) override { return true; }
  bool acquire() override { ++acquisitions; locked = true; environment.owned = {"arm", "tool", "base"}; return true; }
  bool recovery_owns_commands() override { return locked; }
  bool release() override { ++releases; locked = false; environment.owned.clear(); return true; }
  bool nominal_owns_commands() override { return !locked; }
  bool hold(const std::string &) override { locked = true; return true; }
};
struct Fixture {
  std::string family, model;
  std::vector<std::string> joints;
  Json binding, contract, manifest;
  RecoveryRecipe recipe;
  Transport transport;
  Environment environment;
  std::map<std::string, std::string> hashes;
  AdapterRecoveryLibraryPort port;
  Fixture(std::string robot, std::string variant, std::vector<std::string> names)
      : family(std::move(robot)), model(std::move(variant)), joints(std::move(names)),
        hashes({{"pause_condition_v1", std::string(64, 'a')},
                {"moveit_joint_checkpoint_v1", std::string(64, 'b')},
                {"gripper_gap_v1", std::string(64, 'c')}, {"nav2_pose_checkpoint_v1", std::string(64, 'd')}}),
        port(transport, environment, hashes) {
    const auto n = joints.size();
    auto observed = joints; observed.push_back("test_gripper_joint");
    binding = {{"joint_order", observed}, {"recovery_controller", {{"id", "rearguard_recovery"}}},
      {"recovery_adapters", {{"schema_version", 1}, {"robot_family", family}, {"robot_model", model},
        {"arm", {{"joint_order", joints}, {"resource", "arm"}, {"units", "rad"},
          {"lower", std::vector<double>(n, -1)}, {"upper", std::vector<double>(n, 1)},
          {"velocity", std::vector<double>(n, 1)}, {"acceleration", std::vector<double>(n, 2)},
          {"start_tolerance", 0.01}, {"goal_tolerance", 0.01}, {"stopped_velocity", 0.01},
          {"checkpoints", {{"approach", std::vector<double>(n, 0.1)}}},
          {"trajectory_action", "/rearguard_recovery/follow_joint_trajectory"}}},
        {"moveit", {{"action", "/move_action"}, {"group", "test_arm"}, {"pipeline", "ompl"},
          {"planner", "RRTConnectkConfigDefault"}, {"planning_time_s", 0.2},
          {"velocity_scale", 0.25}, {"acceleration_scale", 0.25}}},
        {"tool", {{"action", "/rearguard_tool/gripper_cmd"}, {"resource", "tool"}, {"units", "m"},
          {"min_gap", 0}, {"max_gap", 0.04}, {"max_effort", 5}, {"tolerance", 0.001}}}}}};
    contract = {{"controller_id", "rearguard_recovery"}, {"retry_budget", 0}, {"timeout_ms", 15000}};
    manifest = {{"schema_version", 1}, {"kind", "recovery_action_library"}, {"id", "test_recipe"}, {"version", "1"},
      {"binding_hash", digest(binding)}, {"controller_id", "rearguard_recovery"}, {"rearm_predicate", "cleared"},
      {"handback", {{"readiness_predicate", "physical"}, {"max_age_ms", 100}, {"stability_ms", 0}, {"timeout_ms", 1000}}},
      {"stages", Json::array()}};
    add("pause_reobserve", "pause_condition_v1", Json::object());
    add("retreat_checkpoint", "moveit_joint_checkpoint_v1", {{"checkpoint", "approach"}});
    add("reset_end_effector", "gripper_gap_v1", {{"operation", "set_gap"}, {"gap_m", 0.02}, {"max_effort_n", 2}});
    environment.state = {{"epoch", 1}, {"t_ns", 1000000000}, {"scene_revision", "scene-1"},
      {"arm_positions", std::vector<double>(n, 0)}, {"arm_velocities", std::vector<double>(n, 0)},
      {"disengaged", true}, {"payload_modelled", true}};
  }
  void add(const std::string &action, const std::string &capability, Json parameters) {
    manifest["stages"].push_back({{"id", std::to_string(manifest["stages"].size())}, {"action", action},
      {"capability", capability}, {"implementation_hash", hashes.at(capability)}, {"parameters", parameters},
      {"entry_predicate", "entry"}, {"completion_predicate", "complete"}, {"timeout_ms", 3000}});
  }
  bool configure() {
    manifest["binding_hash"] = digest(binding);
    recipe = compile_recovery_recipe(manifest, binding, contract);
    const bool result = port.configure(recipe, binding, contract);
    observe(); return result;
  }
  void observe() { Observations o; o.epoch = 1; o.receipt_ros_ns = 1000000000; port.observe(o); }
  Json plan() const {
    const auto n = joints.size();
    return {{"joint_order", joints}, {"points", Json::array({
      {{"positions", std::vector<double>(n, 0)}, {"velocities", std::vector<double>(n, 0)},
       {"accelerations", std::vector<double>(n, 0)}, {"time_s", 0}},
      {{"positions", std::vector<double>(n, 0.1)}, {"velocities", std::vector<double>(n, 0)},
       {"accelerations", std::vector<double>(n, 0)}, {"time_s", 2}}})}};
  }
  void start_moveit() {
    check(configure(), "adapter binding prepares");
    check(port.start(recipe.stages[1]), "MoveIt plan starts");
    check(transport.sent.size() == 1 && transport.sent.back().backend == RecoveryBackend::moveit_plan,
          "planning cannot directly execute a trajectory");
    check(transport.sent.back().request.at("start").size() == joints.size() &&
          transport.sent.back().request.at("joint_order") == joints, "per-arm axes, order and current state are explicit");
  }
};
template <class Fn> void each_arm(Fn fn) {
  Fixture so("so101", "so101", {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"}); fn(so);
  Fixture piper("agilex_piper", "piper", {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"}); fn(piper);
}
void movement_tests() {
  each_arm([](Fixture &f) {
    f.start_moveit();
    check(!f.port.quiescent(), "pending planning is not quiescent");
    f.transport.finish(ActionState::succeeded, f.plan());
    check(f.port.poll() == "running" && f.transport.sent.size() == 2 &&
          f.transport.sent.back().backend == RecoveryBackend::joint_trajectory &&
          f.transport.sent.back().endpoint == "/rearguard_recovery/follow_joint_trajectory",
          "validated MoveIt plan executes exclusively through bound recovery controller");
    check(!f.port.start(f.recipe.stages[2]), "overlapping tool stage refused while trajectory pending");
    f.transport.finish(ActionState::succeeded);
    check(f.port.poll() == "succeeded" && f.port.quiescent(), "trajectory terminal result");
    check(f.port.start(f.recipe.stages[2]), "explicit gripper stage starts after trajectory");
    check(f.transport.sent.back().backend == RecoveryBackend::gripper_gap &&
          f.transport.sent.back().request.at("gap_m") == 0.02, "gripper uses measured metres, not servo angles");
    f.transport.finish(ActionState::succeeded, {{"position", 0.02}, {"effort", 1}, {"reached_goal", true}, {"stalled", false}});
    check(f.port.poll() == "succeeded", "measured gap result accepted");
  });
  for (int fault = 0; fault < 17; ++fault) each_arm([&](Fixture &f) {
    f.start_moveit(); auto plan = f.plan();
    if (fault == 0) plan["joint_order"][0] = "other_arm";
    if (fault == 1) plan["points"][1]["positions"][0] = 2;
    if (fault == 2) plan["points"][0]["positions"][0] = 0.3;
    if (fault == 3) plan["points"][1]["positions"][0] = 0.2;
    if (fault == 4) plan["points"][1]["velocities"][0] = 1;
    if (fault == 5) plan["points"][1]["accelerations"][0] = 1;
    if (fault == 6) plan["points"][1]["time_s"] = 0;
    if (fault == 7) plan["points"][1]["time_s"] = 0.001;
    if (fault == 8) plan["points"][1]["positions"][0] = std::numeric_limits<double>::quiet_NaN();
    if (fault == 9) f.environment.state["scene_revision"] = "changed";
    if (fault == 10) f.environment.state["arm_positions"][0] = 0.2;
    if (fault == 11) f.environment.state["disengaged"] = false;
    if (fault == 12) f.environment.state["payload_modelled"] = false;
    if (fault == 13) f.environment.state["t_ns"] = 0;
    if (fault == 14) plan["points"][1]["time_s"] = 4;
    if (fault == 15) plan["points"][1]["velocities"][0] = 0.1;
    if (fault == 16) plan["points"][0]["time_s"] = 0.01;
    f.transport.finish(ActionState::succeeded, plan);
    check(f.port.poll() == "failed" && f.transport.sent.size() == 1,
          "invalid trajectory or changed physical state prevents execution on every arm");
  });
}
void binding_tests() {
  for (int fault = 0; fault < 15; ++fault) each_arm([&](Fixture &f) {
    auto &c = f.binding["recovery_adapters"];
    if (fault == 0) c["arm"]["joint_order"].push_back("test_gripper_joint");
    if (fault == 1) c["arm"]["units"] = "degree";
    if (fault == 2) c["arm"]["trajectory_action"] = "/nominal/follow_joint_trajectory";
    if (fault == 3) c["arm"]["joint_order"][0] = c["arm"]["joint_order"][1];
    if (fault == 4) c["arm"]["checkpoints"]["approach"][0] = 2;
    if (fault == 5) c["moveit"]["velocity_scale"] = 0;
    if (fault == 6) c["tool"]["units"] = "servo_tick";
    if (fault == 7) c["tool"]["resource"] = "arm";
    if (fault == 8) f.manifest["stages"][1]["implementation_hash"] = std::string(64, 'f');
    if (fault == 9) f.manifest["stages"][1]["parameters"]["checkpoint"] = "unapproved";
    if (fault == 10) f.manifest["stages"][1]["parameters"]["retry"] = true;
    if (fault == 11) f.transport.unavailable.insert(RecoveryBackend::moveit_plan);
    if (fault == 12) f.environment.qualified = false;
    if (fault == 13) f.manifest["stages"][2]["parameters"]["gap_m"] = 0.1;
    if (fault == 14) c["robot_model"] = "unspecified";
    check(!f.configure() && f.transport.sent.empty(), "unsupported binding cannot activate or actuate");
  });
  each_arm([](Fixture &f) {
    check(f.configure(), "configure");
    f.environment.owned.erase("tool");
    check(!f.port.start(f.recipe.stages[2]) && f.transport.sent.empty(), "arm authority cannot substitute for tool authority");
  });
}
void cancellation_tests() {
  each_arm([](Fixture &f) {
    f.start_moveit();
    check(!f.port.cancel("timeout") && f.transport.cancels == 1, "cancel request is not terminal acknowledgement");
    check(!f.port.quiescent() && !f.port.start(f.recipe.stages[2]), "pending cancellation blocks stages and handback");
    f.transport.finish(ActionState::succeeded, f.plan());
    check(f.port.poll() == "failed" && f.transport.sent.size() == 1 && f.port.quiescent(),
          "late planning result after cancellation cannot dispatch motion");
  });
  each_arm([](Fixture &f) {
    f.start_moveit(); f.transport.finish(ActionState::succeeded, f.plan()); f.port.poll();
    f.environment.owned.clear();
    check(f.port.poll() == "failed" && f.transport.cancels == 1 && !f.port.quiescent(),
          "ownership loss cancels in-flight execution without false quiescence");
    f.transport.finish(ActionState::cancelled);
    check(!f.port.quiescent() && f.port.cancel("ownership loss"),
          "terminal cancellation acknowledged while absent authority still blocks handback");
  });
}
void tool_and_base_tests() {
  for (int fault = 0; fault < 4; ++fault) each_arm([&](Fixture &f) {
    check(f.configure() && f.port.start(f.recipe.stages[2]), "tool configure/start");
    Json result = {{"position", 0.02}, {"effort", 1}, {"reached_goal", true}, {"stalled", false}};
    if (fault == 0) result["stalled"] = true;
    if (fault == 1) result["reached_goal"] = false;
    if (fault == 2) result["position"] = 0.03;
    if (fault == 3) result["effort"] = 4;
    f.transport.finish(ActionState::succeeded, result);
    check(f.port.poll() == "failed", "tool action success cannot override invalid measured gripper result");
  });
  each_arm([](Fixture &f) {
    f.add("retreat_checkpoint", "nav2_pose_checkpoint_v1", {{"checkpoint", "base_clear"}});
    check(!f.configure(), "fixed arm has no implicit Nav2 capability");
    f.binding["recovery_adapters"]["nav2"] = {{"action", "/rearguard_base/navigate_to_pose"}, {"resource", "base"},
      {"frame", "map"}, {"behavior_tree", "/qualified/no_retry.xml"}, {"checkpoints", {{"base_clear", {0, 0, 0, 0, 0, 0, 1}}}}};
    check(f.configure(), "explicit base binding");
    f.environment.owned.erase("base");
    check(!f.port.start(f.recipe.stages.back()), "arm lock alone cannot start base");
    f.environment.owned.insert("base");
    check(f.port.start(f.recipe.stages.back()) && f.transport.sent.back().backend == RecoveryBackend::nav2_pose &&
          f.transport.sent.back().request.at("frame") == "map", "Nav2 receives approved pose/frame and behavior tree");
    f.transport.finish(ActionState::failed);
    check(f.port.poll() == "failed", "Nav2 failure does not silently retry");
  });
}
void composition_tests() {
  each_arm([](Fixture &f) {
    check(f.configure(), "configure");
    const auto path = fs::temp_directory_path() / ("adapter-composition-" + unique_id());
    atomic_json(path, f.manifest);
    LibraryRecoveryController executor(f.port);
    Authority authority(f.environment);
    SwitchingRecoveryController controller(authority, executor);
    check(controller.prepare(path, f.binding, f.contract), "recipe with concrete adapters prepares");
    Observations obs; obs.epoch = 1; obs.receipt_ros_ns = 1000000000;
    controller.observe(obs, 1);
    check(controller.begin() && controller.poll() == "running", "pause passes and starts MoveIt under one lock");
    f.transport.finish(ActionState::succeeded, f.plan());
    check(controller.poll() == "running", "validated plan dispatched");
    f.transport.finish(ActionState::succeeded);
    check(controller.poll() == "running", "trajectory completes and tool stage starts");
    f.transport.finish(ActionState::succeeded, {{"position", 0.02}, {"effort", 1}, {"reached_goal", true}, {"stalled", false}});
    f.environment.completion = false;
    check(controller.poll() == "running" && authority.releases == 0,
          "lower-level success still requires physical stage predicate");
    f.environment.completion = true;
    check(controller.poll() == "succeeded", "recipe completed");
    const Json context = {{"episode_id", "episode"}, {"deployment_id", "deployment"}, {"deployment_content_hash", "hash"}};
    check(controller.prepare_handback(context) == ControllerAdapter::Handback::pending && authority.locked,
          "client prepares while locked");
    f.environment.ack = {{"transaction", f.environment.transaction}, {"ready", true}, {"command", std::vector<double>(f.joints.size(), 0.1)}};
    check(controller.prepare_handback(context) == ControllerAdapter::Handback::ready && controller.resume() &&
          authority.acquisitions == 1 && authority.releases == 1 && f.environment.commits == 1,
          "all target arms use one verified handback with no task retry");
    fs::remove(path);
  });
}
}
int main() {
  try {
    movement_tests(); binding_tests(); cancellation_tests(); tool_and_base_tests(); composition_tests();
    std::cout << checks << " multi-arm recovery adapter checks passed\n"; return 0;
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
