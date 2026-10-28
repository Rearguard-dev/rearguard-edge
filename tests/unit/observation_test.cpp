#include "evidence.hpp"
#include "binding.hpp"
#include "events.hpp"
#include "runtime.hpp"
#include "controller_handover.hpp"
#include "sync.hpp"
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>

using namespace harness::observation;
namespace {
unsigned checks = 0;
void check(bool condition, const std::string &message) {
  ++checks;
  require(condition, "test failed: " + message);
}
template <class F> void rejects(F action, const std::string &fragment = "") {
  try {
    action();
  } catch (const std::exception &error) {
    check(std::string(error.what()).find(fragment) != std::string::npos,
          "unexpected error: " + std::string(error.what()));
    return;
  }
  throw std::runtime_error("expected rejection: " + fragment);
}
struct Fixture {
  fs::path root =
      fs::temp_directory_path() / ("harness-native-test-" + unique_id());
  Store store = Store::create(root / "store", HARNESS_OBSERVATION_CONFIG);
  Json binding = store.snapshot()["binding"];
  ~Fixture() { fs::remove_all(root); }
  Json apply(const std::string &operation, const Json &payload) {
    return store.apply({{"schema_version", 1},
                        {"request_id", unique_id()},
                        {"actor", "test-operator"},
                        {"expected_revision", store.snapshot()["revision"]},
                        {"operation", operation},
                        {"payload", payload}})["result"];
  }
  Json artifact(const std::string &kind, const Json &body = nullptr) {
    const auto path = root / unique_id();
    {
      std::ofstream stream(path, std::ios::binary);
      if (kind.ends_with("mcap"))
        stream << std::string("\x89MCAP0\r\n", 8)
               << "envelope-only test fixture, not a demonstration"
               << std::string("\x89MCAP0\r\n", 8);
      else
        stream << (body.is_null() ? "opaque test model" : body.dump());
    }
    return store.import_artifact(path, kind);
  }
  Json candidate(bool attach = true) {
    Json coverage = Json::object();
    for (const auto &t : binding["topics"])
      coverage[t["name"].get<std::string>()] = {{"valid_count", 5}};
    auto c = apply("candidate", {{"window",
                                  {{"id", unique_id()},
                                   {"epoch", 2},
                                   {"start_ns", 1},
                                   {"end_ns", 100},
                                   {"status", "ready"},
                                   {"coverage", coverage}}},
                                 {"session_id", "session"},
                                 {"binding_hash", digest(binding)},
                                 {"detection",
                                  {{"detector_id", "fixture"},
                                   {"source", "test_injection"},
                                   {"confidence", 0.8},
                                   {"evidence_class", "target_moved"}}}});
    if (attach)
      apply("attach_evidence", {{"candidate_id", c["id"]},
                                {"artifact", artifact("evidence_mcap")}});
    return c;
  }
  Json bundle(bool with_proposition = false) {
    const auto c = candidate();
    Json acceptance = {{"candidate_id", c["id"]}, {"description", "failed grasp"},
                       {"guidance", "hold and reacquire"}};
    if (with_proposition) {
      apply("add_proposition", {{"proposition_id", "target_moved"},
          {"description", "Target moved"},
          {"scope", {{"required_hz", 10}, {"max_input_age_ms", 100},
                     {"required_roles", {"joint_state"}}}}});
      acceptance["proposition_id"] = "target_moved";
    }
    const auto f = apply("accept_failure", acceptance);
    const auto j = apply(
        "request_demonstration",
        {{"failure_id", f["id"]}, {"requested_model", "external-fixture"}});
    Json content = {{"job_input_hash", j["input_hash"]},
                    {"binding_hash", digest(binding)},
                    {"generator", "external-fixture"},
                    {"demonstration", artifact("demonstration_mcap")},
                    {"critic", artifact("critic_model")},
                    {"recovery", artifact("recovery_model")},
                    {"contract",
                     {{"evidence_class", "target_moved"},
                      {"entry_predicate", "target_displaced"},
                      {"completion_predicate", "target_reacquired"},
                      {"fallback", "hold"},
                      {"controller_id", "so101"},
                      {"confidence_min", 0.7},
                      {"timeout_ms", 1000},
                      {"retry_budget", 1}}}};
    content["validation"] =
        artifact("validation_report",
                 {{"passed", true},
                  {"validator", "fixture"},
                  {"binding_hash", digest(binding)},
                  {"job_input_hash", j["input_hash"]},
                  {"contract_hash", digest(content["contract"])},
                  {"demonstration_sha256", content["demonstration"]["sha256"]},
                  {"critic_sha256", content["critic"]["sha256"]},
                  {"recovery_sha256", content["recovery"]["sha256"]}});
    return apply("submit_bundle", {{"job_id", j["id"]}, {"bundle", content}});
  }
  Json deploy(bool with_proposition = false) {
    const auto b = bundle(with_proposition);
    apply("approve_bundle",
          {{"bundle_id", b["id"]}, {"content_hash", b["content_hash"]}});
    if (with_proposition)
      apply("link_proposition_recovery", {{"proposition_id", "target_moved"},
            {"bundle_id", b["id"]}, {"content_hash", b["content_hash"]}});
    return apply("deploy", {{"bundle_id", b["id"]}});
  }
};

void binding_tests() {
  const Json idle_domain = {
      {"domain_id", 0},
      {"topics", Json::array({{{"name", "/rosout"},
                                {"types", {"rcl_interfaces/msg/Log"}},
                                {"sample", {{"status", "timeout"}}}}})}};
  const Json robot_domain = {
      {"domain_id", 71},
      {"topics", Json::array({
          {{"name", "/joint_states"},
           {"types", {"sensor_msgs/msg/JointState"}},
           {"publisher_count", 1},
           {"sample", {{"status", "captured"},
                       {"data", {{"name", {"j2", "j1"}}}}}}},
          {{"name", "/controller/commands"},
           {"types", {"std_msgs/msg/Float64MultiArray"}},
           {"publisher_count", 0},
           {"sample", {{"status", "unavailable"}}}},
          {{"name", "/front/image_raw/compressed"},
           {"types", {"sensor_msgs/msg/CompressedImage"}},
           {"publisher_count", 1},
           {"sample", {{"status", "captured"}}}},
          {{"name", "/wrist/image_raw/compressed"},
           {"types", {"sensor_msgs/msg/CompressedImage"}},
           {"publisher_count", 1},
           {"sample", {{"status", "captured"}}}},
          {{"name", "/front/image_raw/zstd"},
           {"types", {"sensor_msgs/msg/CompressedImage"}},
           {"publisher_count", 1},
           {"sample", {{"status", "captured"}}}},
          {{"name", "/clock"},
           {"types", {"rosgraph_msgs/msg/Clock"}},
           {"publisher_count", 0},
           {"sample", {{"status", "timeout"}}}}
      })}};
  const Json inventory = {{"captured_wall_ns", 123},
                          {"domains", Json::array({idle_domain, robot_domain})}};
  check(select_observation_domain(inventory) == 71,
        "binding selects the structurally usable ROS domain");
  const auto binding = compile_observation_binding(inventory, 71, "custom-arm");
  check(binding["robot"] == "custom-arm" && binding["clock"] == "ros_system",
        "binding uses assignment as metadata and infers a system clock");
  check(binding["joint_order"] == Json::array({"j2", "j1"}),
        "binding preserves observed joint order");
  check(binding["topics"].size() == 5 &&
            binding["topics"][1]["role"] == "action" &&
            !binding["topics"][1]["required"].get<bool>() &&
            binding["topics"][2]["name"] == "/front/image_raw/compressed",
        "unpublished commands remain optional and camera transports stay observable");
  const auto groups = required_topic_groups(binding);
  check(groups.size() == 3 && groups[1].size() == 2,
        "legacy camera transport variants satisfy one physical-camera requirement");
  auto simulated = inventory;
  simulated["domains"][1]["topics"][5]["sample"]["status"] = "captured";
  check(compile_observation_binding(simulated, 71, "sim")["clock"] == "ros_sim",
        "only a sampled advancing clock selects simulation time");
}
struct FakeCritic final : CriticAdapter {
  std::optional<Detection> detection = Detection{"tiny", "target_moved", 0.9};
  bool ready() const override { return true; }
  bool load(const fs::path &, const Json &, const Json &) override {
    return true;
  }
  std::optional<Detection> evaluate(const Observations &) override {
    return detection;
  }
};
struct FakeController final : ControllerAdapter {
  std::string outcome = "running";
  bool hold_ack = true, begin_ack = true, resume_ack = true;
  int prepares = 0, begins = 0, holds = 0;
  bool ready() const override { return true; }
  bool prepare(const fs::path &, const Json &, const Json &) override {
    ++prepares;
    return true;
  }
  bool begin() override { ++begins; return begin_ack; }
  std::string poll() override { return outcome; }
  bool resume() override { return resume_ack; }
  bool hold(const std::string &) override { ++holds; return hold_ack; }
};
struct FakeWorkflowTransport final : WorkflowTransport {
  Json commands = Json::array();
  std::vector<Json> uploads;
  std::vector<Json> enforcement_uploads;
  bool fail_upload = false;
  void upload(const std::string &robot_id, const Json &snapshot,
              const Json &receipts) override {
    if (fail_upload) throw std::runtime_error("offline");
    uploads.push_back({{"robot_id", robot_id},
                       {"snapshot", snapshot},
                       {"receipts", receipts}});
  }
  Json pending(const std::string &) override {
    const auto result = commands;
    commands = Json::array();
    return result;
  }
  void upload_enforcements(const std::string &robot_id, const Json &records) override {
    if (fail_upload) throw std::runtime_error("offline");
    enforcement_uploads.push_back({{"robot_id", robot_id}, {"records", records}});
  }
};
void evidence_tests() {
  const auto config = load_config(HARNESS_OBSERVATION_CONFIG);
  Evidence e(config, "test");
  e.tick(10000000000LL, 100);
  rejects([&] { e.check_live(100); }, "topic");
  const auto id = e.capture(5, 3, "cube displacement")["id"].get<std::string>();
  rejects([&] { e.promote(id, "failure", "operator"); }, "finish");
  for (const auto &t : config["topics"])
    for (Ns seconds : {6, 10, 12})
      e.sample(t["name"], seconds * 1000000000LL, nullptr, 100);
  e.check_live(100);
  e.tick(13000000000LL, 100);
  check(e.state["windows"][0]["status"] == "ready", "window completes");
  check(e.state["failures"].empty(), "no automatic failure confirmation");
  const auto f = e.promote(id, "failure", "operator");
  check(e.promote(id, "again", "operator") == f, "legacy promotion idempotent");
  check(e.state["proposals"].size() == 2 &&
            e.state["proposals"][0]["executable"] == false,
        "legacy templates nonexecutable");
  for (double invalid :
       {0.0, -1.0, 21.0, std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()})
    rejects([&] { e.capture(invalid, 3, "bad"); });
  const auto &history = e.capture(10, 0, "history-only");
  check(history["end_ns"] == history["mark_ns"] &&
            history["mark_ns"].get<Ns>() - history["start_ns"].get<Ns>() ==
                10000000000LL,
        "history-only capture ends at the exact trigger timestamp");
  const auto &incident = e.capture(10, 5, "incident-contract");
  check(incident["mark_ns"].get<Ns>() - incident["start_ns"].get<Ns>() ==
                10000000000LL &&
            incident["end_ns"].get<Ns>() - incident["mark_ns"].get<Ns>() ==
                5000000000LL,
        "incident capture spans exactly T-10s through T+5s");
  e.capture(1, 2, "reset");
  e.tick(1, 100);
  check(e.state["windows"].back()["status"] == "clock_reset",
        "reset invalidates collecting window");
  e.capture(1, 2, "stalled");
  rejects([&] { e.check_live(20000000100LL); }, "clock stopped");
  e.expire(20000000100LL);
  check(e.state["windows"].back()["status"] == "incomplete",
        "stalled post-window expires");
  Evidence bad(config, "bad");
  bad.tick(10000000000LL);
  const auto bad_id =
      bad.capture(5, 3, "invalid action")["id"].get<std::string>();
  for (const auto &t : config["topics"])
    for (Ns seconds : {6, 12})
      bad.sample(t["name"], seconds * 1000000000LL, nullptr, monotonic_ns(),
                 t["role"] != "action");
  bad.tick(13000000000LL);
  rejects([&] { bad.promote(bad_id, "failure", "operator"); }, "insufficient");
}
void workflow_tests() {
  {
    const auto root = fs::temp_directory_path() /
                      ("harness-bound-profile-test-" + unique_id());
    const auto bound = Store::create(root / "bound-store",
                                     HARNESS_OBSERVATION_CONFIG,
                                     "hp_browser_authored");
    check(bound.snapshot()["profile_id"] == "hp_browser_authored",
          "edge store can inherit the browser-authored profile identity");
    fs::remove_all(root);
  }
  {
    Fixture f;
    const Json scope = {{"required_hz", 200}, {"max_input_age_ms", 10},
                         {"required_roles", {"joint_state", "action"}}};
    const auto proposition = f.apply("add_proposition",
        {{"proposition_id", "tracking_divergence"},
         {"description", "Commanded motion diverges from measured motion"}, {"scope", scope}});
    check(proposition["snapshots"].empty() && proposition["recovery"].is_null(),
          "guard can exist before any failure or recovery");
    check(!f.store.snapshot()["critic_profile_ready"],
          "proposition alone does not activate inference");
    f.apply("set_critic_context",
            {{"context",
              {{"schema_version", 1}, {"task", "Pick and place"},
               {"embodiment", {{"class_id", "so101"},
                                {"joint_names", f.binding["joint_order"]}}}}}});
    const auto first_compilation = f.store.snapshot();
    check(first_compilation["critic_profile_ready"] &&
              first_compilation["critic_deployment"]["runtime"] == "proposition_bank_v1" &&
              first_compilation["critic_compilation"]["generation"] == 1 &&
              first_compilation["critic_runtime_status"] == "awaiting_examples" &&
              first_compilation["critic_deployment"]["executable"] == false,
          "profile setup creates the first edge-compiled proposition model");
    const auto first_model_hash =
        first_compilation["critic_compilation"]["model_hash"];
    for (int i = 0; i < 2; ++i) {
      const auto candidate = f.candidate();
      f.apply("accept_failure", {{"candidate_id", candidate["id"]},
          {"description", "divergence"}, {"guidance", "hold"},
          {"proposition_id", "tracking_divergence"}});
    }
    const auto snapshots = f.store.snapshot()["propositions"]["tracking_divergence"]["snapshots"];
    check(snapshots.size() == 2 && snapshots[0]["evidence"]["kind"] == "evidence_mcap" &&
              snapshots[0]["failure_id"] != snapshots[1]["failure_id"],
          "accepted moments accumulate under one proposition with immutable MCAP evidence");
    const auto trained_input = f.store.snapshot();
    const auto compile_status =
        trained_input["critic_runtime_status"].get<std::string>();
    check(trained_input["critic_compilation"]["generation"] == 3 &&
              trained_input["critic_compilation"]["model_hash"] != first_model_hash &&
              (compile_status == "awaiting_ros_runtime" ||
               compile_status == "compile_failed") &&
              trained_input["critic_deployment"]["executable"] == false,
          "each accepted proposition moment rebuilds a new model generation");
    f.apply("add_proposition",
            {{"proposition_id", "collision"}, {"description", "Avoid collision"},
             {"scope", scope}});
    check(f.store.snapshot()["critic_compilation"]["generation"] == 4 &&
              f.store.snapshot()["critic_deployment"]["classes"].size() == 2,
          "adding a proposition rebuilds the edge model without reinstalling");
  }
  {
    Fixture empty_profile;
    const auto state = empty_profile.store.snapshot();
    check(state["critic_deployment"].is_null() &&
              state["critic_compilation"].is_null() &&
              state["enforcement_status"] == "unavailable" &&
              state["critic_runtime_status"] == "awaiting_profile_setup" &&
              std::find(state["placeholders"].begin(),
                        state["placeholders"].end(),
                        "critic_inference") != state["placeholders"].end(),
          "installation carries no bootstrap model and an empty profile cannot infer");
  }
  {
    const auto config_path = fs::path(HARNESS_OBSERVATION_CONFIG);
    require(::setenv("REARGUARD_OBSERVATION_CONFIG_DIR",
                     config_path.parent_path().c_str(), 1) == 0,
            "could not set config path for test");
    check(load_config("so101")["schema_version"] == 1,
          "installed config resolves by logical name");
    ::unsetenv("REARGUARD_OBSERVATION_CONFIG_DIR");
  }
  {
    Fixture sync;
    FakeWorkflowTransport transport;
    WorkflowConnector connector(sync.store,
                                "00000000-0000-4000-8000-000000000100",
                                transport);
    connector.cycle();
    check(transport.uploads.size() == 1 &&
              transport.uploads.back()["snapshot"]["revision"] == 0,
          "connector uploads initial snapshot");
    connector.cycle();
    check(transport.uploads.size() == 1,
          "connector does not duplicate unchanged snapshots");
    transport.commands.push_back(
        {{"schema_version", 1},
         {"request_id", "00000000-0000-4000-8000-000000000001"},
         {"actor", "web-user"},
         {"expected_revision", 0},
         {"operation", "deactivate"},
         {"payload", {{"reason", "connector test"}}}});
    const auto applied = connector.cycle();
    check(applied["commands_received"] == 1 &&
              transport.uploads.size() == 2 &&
              transport.uploads.back()["snapshot"]["revision"] == 1 &&
              transport.uploads.back()["receipts"][0]["status"] == "applied",
          "connector applies command and uploads receipt with new snapshot");
    transport.commands.push_back(
        {{"schema_version", 1},
         {"request_id", "00000000-0000-4000-8000-000000000002"},
         {"actor", "web-user"},
         {"expected_revision", 0},
         {"operation", "deactivate"},
         {"payload", {{"reason", "stale command"}}}});
    connector.cycle();
    check(transport.uploads.size() == 3 &&
              transport.uploads.back()["snapshot"]["revision"] == 1 &&
              transport.uploads.back()["receipts"][0]["status"] == "rejected",
          "connector rejects stale command without changing profile");
    transport.commands.push_back(
        {{"schema_version", 1},
         {"request_id", "00000000-0000-4000-8000-000000000003"},
         {"actor", "web-user"},
         {"expected_revision", 1},
         {"operation", "deactivate"},
         {"payload", {{"reason", "offline receipt test"}}}});
    transport.fail_upload = true;
    rejects([&] { connector.cycle(); }, "offline");
    check(sync.store.snapshot()["revision"] == 2 &&
              sync.store.pending_sync_receipts().size() == 1,
          "applied command and receipt survive an upload failure");
    transport.fail_upload = false;
    connector.cycle();
    check(transport.uploads.size() == 4 &&
              transport.uploads.back()["snapshot"]["revision"] == 2 &&
              transport.uploads.back()["receipts"][0]["request_id"] ==
                  "00000000-0000-4000-8000-000000000003" &&
              sync.store.pending_sync_receipts().empty(),
          "durable command receipt uploads and clears after reconnecting");
    const auto profile = sync.store.snapshot();
    sync.store.record_enforcement(
        {{"schema_version", 1}, {"event_id", "enforcement-sync-test"},
         {"profile_id", profile["profile_id"]}, {"deployment_id", nullptr},
         {"failure_id", nullptr}, {"mode", "active"}, {"outcome", "fallback"},
         {"started_wall_ns", wall_ns()}, {"ended_wall_ns", wall_ns()},
         {"integrity", Json::object()}, {"record", Json::object()}});
    connector.cycle();
    check(transport.enforcement_uploads.size() == 1 &&
              sync.store.pending_enforcements().empty(),
          "sync uploads and acknowledges durable enforcement records");
  }
  {
    Fixture offline;
    FakeWorkflowTransport transport;
    transport.fail_upload = true;
    WorkflowConnector connector(offline.store,
                                "00000000-0000-4000-8000-000000000101",
                                transport);
    const auto local_profile = offline.store.snapshot();
    rejects([&] { connector.cycle(); }, "offline");
    check(offline.store.snapshot() == local_profile,
          "network failure leaves the local harness unchanged");
    transport.fail_upload = false;
    connector.cycle();
    check(transport.uploads.size() == 1 &&
              transport.uploads.back()["snapshot"] == local_profile,
          "connector uploads the retained harness after reconnecting");
  }
  {
    Fixture incident;
    FakeWorkflowTransport transport;
    transport.commands.push_back(
        {{"schema_version", 1},
         {"request_id", "00000000-0000-4000-8000-000000000010"},
         {"actor", "web-user"},
         {"expected_revision", 0},
         {"operation", "report_incident"},
         {"payload", {{"label", "operator report"}, {"before_s", 10},
                      {"after_s", 5}}}});
    int capture_calls = 0;
    WorkflowConnector connector(
        incident.store, "00000000-0000-4000-8000-000000000102", transport,
        [&](const Json &) {
          ++capture_calls;
          return Json{{"id", "w-web-report"}, {"epoch", 2},
                      {"start_ns", 100}, {"mark_ns", 10000000100LL},
                      {"end_ns", 15000000100LL},
                      {"label", "operator report"}, {"status", "collecting"},
                      {"trigger", "operator_mark"}};
        });
    connector.cycle();
    const auto profile = incident.store.snapshot();
    check(capture_calls == 1 && profile["revision"] == 1 &&
              profile["incident_requests"]["w-web-report"]["window"]["mark_ns"] == 10000000100LL &&
              transport.uploads.back()["receipts"][0]["status"] == "applied",
          "web incident command is timestamped by the observer and durably mirrored");
    transport.commands.push_back(
        {{"schema_version", 1},
         {"request_id", "00000000-0000-4000-8000-000000000010"},
         {"actor", "web-user"}, {"expected_revision", 0},
         {"operation", "report_incident"},
         {"payload", {{"label", "operator report"}, {"before_s", 10},
                      {"after_s", 5}}}});
    connector.cycle();
    check(capture_calls == 1 && incident.store.snapshot()["revision"] == 1,
          "incident command retry replays its receipt without marking a second window");
  }
  {
    Fixture invalid_window;
    FakeWorkflowTransport transport;
    transport.commands.push_back(
        {{"schema_version", 1},
         {"request_id", "00000000-0000-4000-8000-000000000011"},
         {"actor", "web-user"},
         {"expected_revision", 0},
         {"operation", "report_incident"},
         {"payload", {{"label", "operator report"}, {"before_s", 10},
                      {"after_s", 5}}}});
    WorkflowConnector connector(
        invalid_window.store, "00000000-0000-4000-8000-000000000103",
        transport, [&](const Json &) {
          return Json{{"id", "w-invalid-window"}, {"epoch", 2},
                      {"start_ns", 100}, {"mark_ns", 10000000100LL},
                      {"end_ns", 10000000100LL},
                      {"label", "operator report"}, {"status", "ready"},
                      {"trigger", "operator_mark"}};
        });
    connector.cycle();
    const auto profile = invalid_window.store.snapshot();
    const auto receipt = transport.uploads.back()["receipts"][0];
    check(profile["revision"] == 0 && profile["incident_requests"].empty() &&
              receipt["status"] == "rejected" &&
              receipt["error"].get<std::string>().find("T-10s") !=
                  std::string::npos,
          "edge rejects incident evidence outside the T-10s through T+5s contract");
  }
  Fixture f;
  {
    Fixture created;
    const auto candidate = created.candidate();
    const auto failure = created.apply(
        "accept_failure",
        {{"candidate_id", candidate["id"]}, {"description", "missed grasp"},
         {"guidance", "hold and retry"},
         {"new_proposition",
          {{"proposition_id", "incident_missed_grasp"},
           {"description", "The gripper must not miss the target"}}}});
    const auto profile = created.store.snapshot();
    check(failure["proposition_id"] == "incident_missed_grasp" &&
              profile["propositions"]["incident_missed_grasp"]["snapshots"].size() == 1,
          "incident review can create and attach a proposition atomically");
  }
  Json retry = {{"schema_version", 1},       {"request_id", "retry"},
                {"actor", "operator"},       {"expected_revision", 0},
                {"operation", "deactivate"}, {"payload", {{"reason", "test"}}}};
  const auto first = f.store.apply(retry);
  check(Store(f.store.root).apply(retry) == first, "durable idempotency");
  check(f.store.outbox().size() == 2, "retry does not duplicate event");
  check(f.store.outbox().front()["event"]["operation"] == "initialize",
        "profile creation enters sync outbox");
  retry["payload"]["reason"] = "changed";
  rejects([&] { f.store.apply(retry); }, "different content");
  retry["request_id"] = "conflict";
  const auto before = f.store.snapshot();
  rejects([&] { f.store.apply(retry); }, "revision conflict");
  check(f.store.snapshot() == before, "conflict rolls back");
  rejects([&] { Store::create(f.store.root, HARNESS_OBSERVATION_CONFIG); },
          "already exists");
  rejects(
      [&] { f.apply("candidate", {{"window", {{"status", "collecting"}}}}); },
      "completed");
  auto c = f.candidate(false);
  rejects(
      [&] {
        f.apply("accept_failure", {{"candidate_id", c["id"]},
                                   {"description", "failure"},
                                   {"guidance", "hold"}});
      },
      "artifact");
  f.apply("reject_candidate",
          {{"candidate_id", c["id"]}, {"reason", "normal motion"}});
  rejects([&] { f.apply("accept_failure", {{"candidate_id", c["id"]}}); },
          "already reviewed");
  auto b = f.bundle();
  rejects([&] { f.apply("deploy", {{"bundle_id", b["id"]}}); }, "approved");
  rejects(
      [&] {
        f.apply("approve_bundle",
                {{"bundle_id", b["id"]}, {"content_hash", "wrong"}});
      },
      "exact");
  f.apply("approve_bundle",
          {{"bundle_id", b["id"]}, {"content_hash", b["content_hash"]}});
  Json coverage = Json::object();
  for (const auto &topic : f.binding["topics"])
    coverage[topic["name"].get<std::string>()] = {{"valid_count", 5}};
  auto routed = f.apply(
      "candidate",
      {{"window", {{"id", unique_id()}, {"epoch", 3}, {"status", "ready"},
                    {"coverage", coverage}}},
       {"session_id", "trained-session"}, {"binding_hash", digest(f.binding)},
       {"detection", {{"detector_id", "validated-critic"},
                      {"deployment_id", "validated-deployment"},
                      {"deployment_validated", true}, {"source", "trained_critic"},
                      {"confidence", 0.95}, {"evidence_class", "target_moved"}}}});
  routed = f.apply("attach_evidence",
                   {{"candidate_id", routed["id"]},
                    {"artifact", f.artifact("evidence_mcap")}});
  check(routed["status"] == "pending_review" &&
            routed["approved_recovery_match"].is_null(),
        "detector claims cannot bypass the edge-compiled deployment identity");
  const auto new_job =
      f.apply("request_demonstration", {{"failure_id", b["failure_id"]},
                                        {"requested_model", "external"}});
  auto content = b["content"];
  content["job_input_hash"] = new_job["input_hash"];
  content["contract"]["timeout_ms"] = 1234;
  const auto snapshot = f.store.snapshot();
  rejects(
      [&] {
        f.apply("submit_bundle",
                {{"job_id", new_job["id"]}, {"bundle", content}});
      },
      "validation report");
  check(f.store.snapshot() == snapshot, "validation rollback");
  f.apply("set_guidance",
          {{"failure_id", b["failure_id"]}, {"guidance", "new guidance"}});
  rejects(
      [&] {
        f.apply("approve_bundle",
                {{"bundle_id", b["id"]}, {"content_hash", b["content_hash"]}});
      },
      "superseded");
  const auto d = f.deploy();
  check(Store(f.store.root).snapshot()["installed_deployment"] == d["id"],
        "deployment survives reopen");
  check(d["status"] == "installed_not_enforcing", "honest installed status");
  check(f.store.outbox().back()["profile"] == f.store.snapshot(),
        "outbox holds exact snapshot");
  rejects(
      [&] {
        f.apply("set_guidance", {{"failure_id", d["failure_id"]},
                                 {"guidance", "cannot change installed"}});
      },
      "deactivate");
  CriticInferencePlaceholder critic;
  RecoveryControllerPlaceholder controller;
  Runtime runtime(f.store, critic, controller);
  rejects([&] { runtime.activate(); }, "adapters");
  b = f.bundle();
  f.apply("approve_bundle",
          {{"bundle_id", b["id"]}, {"content_hash", b["content_hash"]}});
  {
    std::ofstream stream(f.store.root / "artifacts" /
                         b["content"]["critic"]["sha256"].get<std::string>());
    stream << "tampered";
  }
  rejects([&] { f.apply("deploy", {{"bundle_id", b["id"]}}); }, "hash/size");
}
void event_tests() {
  const auto root =
      fs::temp_directory_path() / ("harness-event-test-" + unique_id());
  require(::setenv("REARGUARD_STATE_DIR", root.c_str(), 1) == 0,
          "could not set event state path");
  record_event("setup_scan", "complete", "success", {{"domains", 1}});
  record_event("workflow_sync", "cycle", "failure", {{"message", "offline"}});
  const auto events = read_events();
  check(events.size() == 2 && events[0]["status"] == "success" &&
            events[1]["status"] == "failure" && events[1]["index"] == 2,
        "durable event journal preserves order and status");
  check(read_events(1).size() == 1,
        "event journal supports incremental inspection");
  ::unsetenv("REARGUARD_STATE_DIR");
  fs::remove_all(root);
}
struct FakeAuthority final : ControllerAuthority {
  bool acquire_ack = true, release_ack = true, owns_recovery = true,
       owns_nominal = true, configure_ack = true;
  int configurations = 0, acquisitions = 0, releases = 0, holds = 0;
  bool ready() const override { return true; }
  bool configure(const Json &, const Json &) override {
    ++configurations;
    return configure_ack;
  }
  bool acquire() override { ++acquisitions; return acquire_ack; }
  bool recovery_owns_commands() override { return owns_recovery; }
  bool release() override { ++releases; return release_ack; }
  bool nominal_owns_commands() override { return owns_nominal; }
  bool hold(const std::string &) override { ++holds; return true; }
};
void handover_tests() {
  FakeAuthority authority;
  FakeController executor;
  SwitchingRecoveryController controller(authority, executor);
  check(!controller.begin() && authority.acquisitions == 0,
        "unprepared controller cannot acquire authority");
  check(controller.prepare({}, {}, {}), "handover prepared");
  check(authority.configurations == 1,
        "handover binds authority to the approved deployment");
  check(controller.begin() && executor.begins == 1, "exclusive takeover starts executor");
  check(controller.begin() && authority.acquisitions == 1 && executor.begins == 2,
        "retries retain exclusive authority");
  executor.resume_ack = false;
  check(!controller.resume() && authority.releases == 0,
        "unready nominal reentry cannot release authority");
  executor.resume_ack = true;
  check(controller.resume() && authority.releases == 1, "verified handback");
  check(!controller.hold("monitoring disabled") && authority.holds == 0,
        "nominal controller does not acquire fallback authority");
  check(controller.begin(), "second takeover");
  authority.owns_recovery = false;
  check(controller.poll() == "authority_lost", "ownership loss is detected");
  check(!controller.resume(), "ownership loss forbids blind handback");
  check(controller.hold("ownership lost"), "ownership loss requests configured fallback");
  check(!controller.begin() && !controller.prepare({}, {}, {}),
        "held controller cannot automatically rearm");
  for (int failure = 0; failure < 2; ++failure) {
    FakeAuthority port;
    FakeController runner;
    SwitchingRecoveryController c(port, runner);
    c.prepare({}, {}, {});
    port.acquire_ack = failure != 0;
    port.owns_recovery = failure == 0;
    check(!c.begin() && runner.begins == 0,
          "switch failure or unconfirmed ownership prevents recovery commands");
    check(c.hold("takeover failed") && port.holds == 1,
          "partial acquisition enters fallback");
  }
  {
    FakeAuthority port;
    FakeController runner;
    SwitchingRecoveryController c(port, runner);
    port.configure_ack = false;
    check(!c.prepare({}, {}, {}) && runner.prepares == 0,
          "unbound authority prevents recovery preparation");
  }
  {
    FakeAuthority port;
    FakeController runner;
    SwitchingRecoveryController c(port, runner);
    c.prepare({}, {}, {});
    c.begin();
    port.owns_nominal = false;
    check(!c.resume(), "service success alone does not confirm handback");
    check(c.hold("handback failed"), "unconfirmed handback enters fallback");
  }
}
void authorization_tests() {
  {
    Fixture f;
    f.deploy();
    FakeCritic critic;
    FakeController controller;
    Runtime runtime(f.store, critic, controller);
    rejects([&] { runtime.activate(); }, "accepted failure proposition");
    check(controller.begins == 0 && controller.holds == 0,
          "approved legacy bundle alone cannot intervene");
  }
  {
    Fixture f;
    const auto b = f.bundle(true);
    f.apply("approve_bundle", {{"bundle_id", b["id"]},
                               {"content_hash", b["content_hash"]}});
    f.apply("deploy", {{"bundle_id", b["id"]}});
    FakeCritic critic;
    FakeController controller;
    Runtime runtime(f.store, critic, controller);
    rejects([&] { runtime.activate(); }, "attached approved recovery");
    check(controller.begins == 0 && controller.holds == 0,
          "approval without proposition attachment cannot intervene");
  }
  {
    Fixture f;
    const auto d = f.deploy(true);
    FakeCritic critic;
    FakeController controller;
    Runtime runtime(f.store, critic, controller);
    runtime.activate();
    check(runtime.step({}, 0).is_null(), "unvalidated detection cannot intervene");
    critic.detection->deployment_validated = true;
    critic.detection->deployment_id = d["id"];
    critic.detection->proposition_id = "unrelated";
    check(runtime.step({}, 0.1).is_null(), "unrelated proposition cannot borrow approval");
    critic.detection->proposition_id = "target_moved";
    critic.detection->deployment_id = "old-deployment";
    check(runtime.step({}, 0.2).is_null(), "stale deployment cannot intervene");
    critic.detection->deployment_id = d["id"];
    critic.detection->source = "test_injection";
    check(runtime.step({}, 0.3).is_null(), "injected evidence cannot intervene");
    critic.detection->source = "trained_critic";
    critic.detection->confidence = std::numeric_limits<double>::quiet_NaN();
    check(runtime.step({}, 0.4)["kind"] == "blocked" && controller.holds == 0 &&
              controller.begins == 0, "invalid evidence does not acquire hold authority");
    runtime.deactivate();
    runtime.activate();
    runtime.deactivate();
    check(controller.holds == 0, "disarming monitoring does not stop the robot");
  }
}
void authorized_handover_test() {
  Fixture f;
  const auto d = f.deploy(true);
  FakeCritic critic;
  FakeController executor;
  FakeAuthority authority;
  SwitchingRecoveryController controller(authority, executor);
  Runtime runtime(f.store, critic, controller);
  runtime.activate();
  runtime.step({}, 0);
  check(authority.acquisitions == 0, "arming alone does not switch controllers");
  critic.detection->deployment_validated = true;
  critic.detection->deployment_id = d["id"];
  critic.detection->proposition_id = "target_moved";
  check(runtime.step({}, 0.1)["kind"] == "recovery_started" &&
            authority.acquisitions == 1 && executor.begins == 1,
        "approved proposition drives verified controller takeover");
  executor.outcome = "succeeded";
  check(runtime.step({}, 0.2)["kind"] == "recovery_completed" &&
            authority.releases == 1, "runtime confirms complete handover round trip");
  critic.detection.reset();
  runtime.step({}, 0.3);
  critic.detection = Detection{"tiny", "target_moved", 0.9};
  critic.detection->deployment_validated = true;
  critic.detection->deployment_id = d["id"];
  critic.detection->proposition_id = "target_moved";
  runtime.step({}, 0.4);
  f.apply("deactivate", {{"reason", "operator revoked deployment"}});
  check(runtime.step({}, 0.5)["kind"] == "fallback" && authority.holds == 1 &&
            authority.releases == 1,
        "revocation during recovery retains fallback instead of blind release");
}
void runtime_tests() {
  Fixture f;
  const auto deployment = f.deploy(true);
  FakeCritic critic;
  FakeController controller;
  Runtime r(f.store, critic, controller);
  critic.detection->deployment_id = deployment["id"];
  critic.detection->deployment_validated = true;
  critic.detection->proposition_id = "target_moved";
  r.activate();
  check(r.step({}, 0)["kind"] == "recovery_started", "recovery begins");
  const auto started = f.store.pending_enforcements();
  check(started.size() == 1 && started[0]["outcome"] == "unknown",
        "enforcement start is durably queued");
  controller.outcome = "succeeded";
  check(r.step({}, 0.1)["kind"] == "recovery_completed",
        "completion acknowledged");
  const auto completed = f.store.pending_enforcements();
  check(completed.size() == 1 && completed[0]["outcome"] == "completed" &&
            completed[0]["ended_wall_ns"].is_number_integer(),
        "terminal enforcement outcome replaces the pending start record");
  f.store.acknowledge_enforcements(started);
  check(f.store.pending_enforcements().size() == 1 &&
            f.store.pending_enforcements()[0]["outcome"] == "completed",
        "acknowledging an in-flight start cannot erase its terminal update");
  f.store.acknowledge_enforcements(completed);
  check(f.store.pending_enforcements().empty(),
        "acknowledged enforcement leaves the durable outbox");
  check(r.step({}, 0.2).is_null(), "latched evidence does not retrigger");
  critic.detection.reset();
  r.step({}, 0.3);
  critic.detection = Detection{"tiny", "target_moved", 0.9};
  critic.detection->deployment_id = deployment["id"];
  critic.detection->deployment_validated = true;
  critic.detection->proposition_id = "target_moved";
  check(r.step({}, 0.4)["kind"] == "recovery_started",
        "new rising evidence triggers");
  check(r.step({}, 1.4)["kind"] == "fallback", "monotonic timeout");
  r.deactivate();
  r.activate();
  r.step({}, 2);
  controller.outcome = "failed";
  check(r.step({}, 2.1)["kind"] == "recovery_retry", "bounded retry");
  check(r.step({}, 2.2)["kind"] == "fallback", "retry exhaustion");
  r.deactivate();
  r.activate();
  check(r.step({}, 3, false)["kind"] == "blocked",
        "stale observations cannot initiate intervention");
  r.deactivate();
  r.activate();
  controller.hold_ack = false;
  r.step({}, 3.9);
  check(r.step({}, 4, false)["hold_acknowledged"] == false &&
            r.status == "fault_unconfirmed_hold",
        "no false hold claim");
  controller.hold_ack = true;
  r.deactivate();
  r.activate();
  f.apply("deactivate", {{"reason", "operator"}});
  check(r.step({}, 5)["kind"] == "blocked", "deployment change blocks without takeover");
}
} // namespace
int main() {
  try {
    event_tests();
    check(encoded(Json::parse(
              "{\"z\":1e-5,\"a\":1e15,\"x\":1e16,\"unicode\":\"α\"}")) ==
              "{\"a\":1000000000000000.0,\"unicode\":\"\\u03b1\",\"x\":1e+16,"
              "\"z\":1e-05}",
          "legacy hash encoding");
    check(
        digest(Json::object()) ==
            "44136fa355b3678a1146ad16f7e8649e94fb4fc21fe77e8310c060f61caaff8a",
        "SHA256 known vector");
    binding_tests();
    evidence_tests();
    workflow_tests();
    handover_tests();
    authorization_tests();
    authorized_handover_test();
    runtime_tests();
    std::cout << checks << " native observation/workflow checks passed\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
