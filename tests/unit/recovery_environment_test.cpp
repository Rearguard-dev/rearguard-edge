#include "recovery_environment.hpp"
#include "recovery_executor.hpp"
#include "../../src/tiny_critic/physical_authorization.hpp"
#include "../../include/harness/recovery_client.hpp"
#include <iostream>
using namespace harness::observation;
namespace {
struct Channel final : RecoveryWorkcellChannel {
  RecoveryChannelMessage measured, ack;
  Json messages = Json::array();
  bool owned = true;
  bool configure(const Json &, const Json &) override { return true; }
  RecoveryChannelMessage state() override { return measured; }
  RecoveryChannelMessage acknowledgement() override { return ack; }
  bool owns(const std::string &, const Json &) override { return owned; }
  bool send(const Json &value) override { messages.push_back(value); return true; }
};
void check(bool value) { require(value, "recovery environment regression"); }
template<class F> void rejects(F action) {
  try { action(); } catch (...) { return; }
  throw std::runtime_error("invalid physical approval accepted");
}
struct Executor final : ControllerAdapter {
  int preparations = 0;
  bool accepts = true;
  bool ready() const override { return true; }
  bool prepare(const fs::path &, const Json &, const Json &) override { ++preparations; return accepts; }
  bool begin() override { return true; }
  std::string poll() override { return "running"; }
  bool resume() override { return true; }
  bool hold(const std::string &) override { return true; }
};
}
int main() {
  Channel channel;
  ChannelRecoveryEnvironment environment(channel);
  Json binding = {{"joint_order", {"j1", "j2"}}, {"command_interface", "position"},
    {"recovery_adapters", {{"arm", {{"resource", "arm"}}}, {"contact", {{"resource", "arm"}, {"procedures", {
      {"qualified_exit", {{"implementation_hash", std::string(64, 'b')}, {"max_force_n", 5}, {"max_distance_m", 0.01}, {"timeout_s", 2}}}}}}}}},
    {"recovery_environment", {{"schema_version", 1}, {"provider_id", "cell"},
      {"predicate_ids", {"entry", "complete", "ready", "rearm"}},
      {"resources", {{"arm", {{"controller_id", "rearguard_recovery"}, {"claimed_interfaces", {"j1/position", "j2/position"}}}}}},
      {"command_lower", {-1, -1}}, {"command_upper", {1, 1}}, {"max_start_delta", 0.05}}}};
  RecoveryRecipe recipe; recipe.readiness = "ready"; recipe.rearm = "rearm"; recipe.max_age_s = 0.1;
  RecoveryStage stage; stage.entry = "entry"; stage.completion = "complete"; stage.capability = "pause_condition_v1";
  recipe.stages.push_back(stage);
  check(environment.configure(recipe, binding, Json::object()));
  Observations observations; observations.epoch = 3; observations.receipt_ros_ns = 1000000000;
  environment.observe(observations);
  check(channel.messages.back().at("kind") == "observation_context" && channel.messages.back().at("epoch") == 3);
  Json state = {{"binding_hash", digest(binding)}, {"provider_id", "cell"}, {"epoch", 3}, {"t_ns", observations.receipt_ros_ns},
    {"state_revision", "r1"}, {"quiescent", true}, {"nominal_positions", {0., 0.}},
    {"predicates", {{"ready", {{"observable", true}, {"satisfied", true}}}}}};
  channel.measured = {state, monotonic_ns()};
  environment.observe(observations);
  check(environment.test("ready").satisfied); check(!environment.test("missing").observable);
  auto wrong = state; wrong["epoch"] = 2; channel.measured.payload = wrong; environment.observe(observations); check(!environment.quiescent());
  wrong = state; wrong["binding_hash"] = "other"; channel.measured.payload = wrong; environment.observe(observations); check(!environment.quiescent());
  channel.measured = {state, monotonic_ns() - 200000000}; environment.observe(observations); check(!environment.quiescent());
  channel.measured = {state, monotonic_ns()};
  environment.observe(observations);
  channel.measured.payload["t_ns"] = observations.receipt_ros_ns + 1;
  environment.observe(observations); check(environment.test("ready").t_ns == observations.receipt_ros_ns);
  channel.measured.payload = state;
  auto changed_epoch = observations; changed_epoch.epoch = 4;
  environment.observe(changed_epoch);
  check(channel.messages.back().at("kind") == "observation_context" && channel.messages.back().at("epoch") == 4 && !environment.quiescent());
  environment.observe(observations);
  Json transaction = {{"transaction_id", "t1"}, {"command_epoch", "c1"}, {"state_revision", "r1"},
    {"epoch", 3}, {"prepared_after_ns", observations.receipt_ros_ns}};
  check(environment.prepare_client(transaction, Json::object()));
  harness::RecoveryClientGate client(digest(binding), "cell", "old-policy", {-1, -1}, {1, 1});
  check(client.submit("old-policy", Json::array({0.2, 0.2})));
  check(client.handle(channel.messages.back())); check(!client.take());
  check(!client.stage("old-policy", Json::array({0.01, 0.02}), observations.receipt_ros_ns));
  check(!client.stage("c1", Json::array({0.01, 0.02}), observations.receipt_ros_ns-1));
  check(client.stage("c1", Json::array({0.01, 0.02}), observations.receipt_ros_ns));
  Json ack = state; ack.update({{"transaction", transaction}, {"phase", "staged"}, {"queue_invalidated", true}, {"command", {0.01, 0.02}}});
  channel.ack = {ack, monotonic_ns()};
  environment.observe(observations);
  check(environment.client_acknowledgement().at("ready"));
  check(environment.validate_candidate(transaction, ack.at("command")));
  check(!environment.validate_candidate(transaction, Json::array({0.5, 0.02})));
  channel.owned = false; check(!environment.commit_candidate(transaction)); channel.owned = true;
  channel.ack.payload["transaction"]["command_epoch"] = "stale"; environment.observe(observations); check(!environment.commit_candidate(transaction));
  channel.ack.payload = ack;
  environment.observe(observations);
  check(environment.commit_candidate(transaction));
  check(channel.messages.back().at("kind") == "commit");
  client.handle(channel.messages.back()); check(!client.take());
  environment.handback_confirmed(transaction); check(channel.messages.back().at("kind") == "authority_transferred");
  client.handle(channel.messages.back()); check(client.take().value() == Json::array({0.01, 0.02})); check(!client.take());
  check(!client.submit("old-policy", Json::array({0.2, 0.2})));
  check(environment.cancel("test")); check(!environment.commit_candidate(transaction));
  client.handle(channel.messages.back()); check(client.blocked());
  client.handle({{"schema_version", 1}, {"kind", "authority_transferred"}, {"binding_hash", digest(binding)}, {"transaction", transaction}});
  check(client.blocked());
  channel.measured.payload["contact_capabilities"] = {{"qualified_exit", std::string(64, 'b')}};
  channel.measured.payload["contact_feedback"] = {{"t_ns", observations.receipt_ros_ns}, {"force_n", 1.0}};
  environment.observe(observations);
  RecoveryStage contact; contact.action = RecoveryAction::unload_disengage; contact.capability = "qualified_contact_exit_v1";
  contact.timeout_s = 1; contact.parameters = {{"procedure", "qualified_exit"}, {"max_force_n", 3}, {"max_distance_m", 0.005}};
  check(environment.start_contact(contact)); check(environment.poll_contact() == "running"); check(!environment.contact_quiescent());
  const auto operation_id = channel.messages.back().at("operation").at("request_id");
  channel.measured.payload["contact_exit"] = {{"request_id", operation_id}, {"t_ns", observations.receipt_ros_ns},
    {"force_n", 1.0}, {"distance_m", 0.001}, {"phase", "succeeded"}, {"settled", true}, {"no_late_work", true}, {"disengaged", true}};
  environment.observe(observations);
  check(environment.poll_contact() == "succeeded"); check(environment.contact_quiescent());
  check(environment.start_contact(contact));
  channel.measured.payload["contact_feedback"]["force_n"] = 4;
  environment.observe(observations);
  check(environment.poll_contact() == "failed"); check(channel.messages.back().at("kind") == "cancel");
  check(!environment.contact_quiescent());
  Executor legacy, library; RecoveryExecutorRouter router(legacy, library);
  const auto path = fs::temp_directory_path() / ("recovery-router-" + unique_id() + ".json");
  atomic_json(path, {{"kind", "recovery_action_library"}});
  library.accepts = false; check(!router.prepare(path, binding, {})); check(legacy.preparations == 0);
  library.accepts = true; check(router.prepare(path, binding, {}));
  atomic_json(path, {{"ros2_policy", Json::object()}}); check(router.prepare(path, binding, {})); check(legacy.preparations == 1);
  atomic_json(path, {{"kind", "unknown"}, {"ros2_policy", Json::object()}}); check(!router.prepare(path, binding, {}));
  fs::remove(path);
  Json proposition = {{"id", "manipulation.grasp_not_established"}, {"definition", "grasp attempt completed without established hold"}};
  const auto id = proposition.at("id");
  Json selection = {{"model_id", "physical-model"}, {"profile_id", "profile"}, {"active_proposition_ids", {id}}};
  Json gate = {{"kind", "proposition_clear"}, {"proposition_id", id}, {"query_id", "grasp-clear"}, {"recovery_id", "recovery"}};
  Json reference = {{"schema_version", 1}, {"kind", "physical_critic_reference"}, {"model_id", "physical-model"},
    {"profile_id", "profile"}, {"proposition_id", id}, {"proposition_hash", digest(proposition)},
    {"manifest_sha256", "manifest-hash"}, {"binding_hash", digest(binding)}, {"exit_gate", gate}};
  Json contract = {{"confidence_min", 0.9}, {"evidence_class", id}, {"exit_gate", gate}, {"runtime_activation", {
    {"deployment_id", "deployment"}, {"profile_id", "profile"}, {"proposition_id", id}, {"content_hash", "approval"}, {"binding_hash", digest(binding)}}}};
  Json model_manifest = {{"propositions", {proposition}}};
  const auto authorize = [&] { return harness::tiny::authorize_physical_critic(reference, selection, model_manifest, binding, contract, "manifest-hash"); };
  check(authorize().at("deployment_id") == "deployment");
  reference["model_id"] = "other"; rejects(authorize); reference["model_id"] = "physical-model";
  reference["proposition_hash"] = "changed"; rejects(authorize); reference["proposition_hash"] = digest(proposition);
  contract["exit_gate"]["kind"] = "guessed_readiness"; rejects(authorize); contract["exit_gate"] = gate;
  contract.erase("runtime_activation"); rejects(authorize);
  std::cout << "recovery environment and executor routing checks passed\n";
}
