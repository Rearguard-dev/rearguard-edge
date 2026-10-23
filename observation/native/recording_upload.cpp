#include "recording_upload.hpp"
#include "events.hpp"

#include <algorithm>
#include <curl/curl.h>
#include <csignal>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <set>
#include <thread>

namespace harness::observation {
namespace {
volatile sig_atomic_t uploader_stopping = 0;

void uploader_stop_handler(int) { uploader_stopping = 1; }

fs::path default_credentials_path() {
  if (const auto *value = std::getenv("XDG_CONFIG_HOME"); value && *value)
    return fs::path(value) / "rearguard" / "device.json";
  const auto *home = std::getenv("HOME");
  require(home && *home, "HOME is not set; pass --credentials");
  return fs::path(home) / ".config" / "rearguard" / "device.json";
}

std::size_t append_response(char *data, std::size_t size, std::size_t count,
                            void *output) {
  const auto bytes = size * count;
  static_cast<std::string *>(output)->append(data, bytes);
  return bytes;
}

Json backend_request(const DeviceCredentials &credentials,
                     const std::string &path, const Json &payload) {
  CURL *curl = curl_easy_init();
  require(curl != nullptr, "could not initialize recording upload request");
  std::string response;
  char error[CURL_ERROR_SIZE]{};
  const auto body = payload.dump();
  const auto url = credentials.backend_url + path;
  const auto secret = "X-Rearguard-Device-Secret: " + credentials.secret;
  curl_slist *headers = nullptr;
  headers = curl_slist_append(headers, "Content-Type: application/json");
  headers = curl_slist_append(headers, secret.c_str());
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_POST, 1L);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
  curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(body.size()));
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, append_response);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
  curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, error);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);
  long status = 0;
  const auto result = curl_easy_perform(curl);
  if (result == CURLE_OK) curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
  const auto failure = std::string(error[0] ? error : curl_easy_strerror(result));
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);
  require(result == CURLE_OK, "recording backend request failed: " + failure);
  require(status >= 200 && status < 300,
          "recording backend returned HTTP " + std::to_string(status) +
              ": " + response.substr(0, 1000));
  return response.empty() ? Json::object() : Json::parse(response);
}

void put_file(const fs::path &file, const Json &grant) {
  FILE *input = std::fopen(file.c_str(), "rb");
  require(input != nullptr, "cannot open MCAP for upload");
  CURL *curl = curl_easy_init();
  require(curl != nullptr, "could not initialize R2 upload");
  curl_slist *headers = nullptr;
  for (auto it = grant.at("required_headers").begin();
       it != grant.at("required_headers").end(); ++it) {
    const auto header = it.key() + ": " + it.value().get<std::string>();
    headers = curl_slist_append(headers, header.c_str());
  }
  std::string response;
  const auto upload_url = grant.at("upload_url").get<std::string>();
  char error[CURL_ERROR_SIZE]{};
  curl_easy_setopt(curl, CURLOPT_URL, upload_url.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);
  curl_easy_setopt(curl, CURLOPT_READDATA, input);
  curl_easy_setopt(curl, CURLOPT_INFILESIZE_LARGE,
                   static_cast<curl_off_t>(fs::file_size(file)));
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, append_response);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
  curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, error);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 600L);
  long status = 0;
  const auto result = curl_easy_perform(curl);
  if (result == CURLE_OK) curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
  const auto failure = std::string(error[0] ? error : curl_easy_strerror(result));
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);
  std::fclose(input);
  require(result == CURLE_OK, "R2 MCAP upload failed: " + failure);
  // A retry may race with or follow a successful PUT whose completion call
  // was interrupted. If-None-Match protects the immutable object; 412 means
  // the exact content-addressed key already exists and can be verified.
  require((status >= 200 && status < 300) || status == 412,
          "R2 MCAP upload returned HTTP " + std::to_string(status));
}

std::string modality_kind(const Json &topic) {
  const auto role = topic.value("role", "custom");
  if (role == "joint_state") return "joint_state";
  if (role == "camera") return "image";
  if (role == "action") return "trajectory";
  return "custom";
}

Json load_state(const fs::path &path) {
  if (!fs::is_regular_file(path))
    return {{"schema_version", 2}, {"operations", Json::object()}};
  auto state = read_json(path);
  if (state.value("schema_version", 1) == 1) {
    Json operations = Json::object();
    for (const auto &relative : state.value("uploaded", Json::array()))
      operations[relative.get<std::string>()] = {{"status", "submitted"}};
    state = {{"schema_version", 2}, {"operations", operations}};
  }
  if (!state.contains("operations") || !state["operations"].is_object())
    state["operations"] = Json::object();
  return state;
}

void journal(const fs::path &path, Json &state, const std::string &relative,
             const std::string &status, Json values = Json::object()) {
  auto &operation = state["operations"][relative];
  if (!operation.is_object()) operation = Json::object();
  operation["status"] = status;
  operation["updated_wall_ns"] = wall_ns();
  operation.update(values);
  atomic_json(path, state);
}

unsigned upload_attempts(const Json &state, const std::string &relative) {
  const auto &operations = state.at("operations");
  if (!operations.contains(relative) || !operations.at(relative).is_object())
    return 0U;
  return operations.at(relative).value("attempts", 0U);
}

std::uintmax_t directory_bytes(const fs::path &directory) {
  std::uintmax_t bytes = 0;
  if (!fs::is_directory(directory)) return bytes;
  for (const auto &entry : fs::recursive_directory_iterator(directory))
    if (entry.is_regular_file()) bytes += entry.file_size();
  return bytes;
}

Json retire_inactive_pre_roll(const fs::path &session, const Json &recording,
                              const Json &snapshot,
                              const std::string &session_id) {
  const auto state_path = session / "state.json";
  if (!fs::is_regular_file(state_path))
    return {{"pre_roll_retired", false}, {"reason", "session state missing"}};
  const auto observation = read_json(state_path);
  const auto heartbeat = observation.value("heartbeat_wall_ns", Ns{0});
  const auto heartbeat_age = wall_ns() - heartbeat;
  if (observation.value("status", "") == "observing" && heartbeat > 0 &&
      heartbeat_age >= 0 && heartbeat_age < 30000000000LL)
    return {{"pre_roll_retired", false}, {"reason", "observer active"}};

  for (const auto &window : observation.value("windows", Json::array())) {
    if (window.value("status", "") == "collecting")
      return {{"pre_roll_retired", false},
              {"reason", "incident window still requires export"}};
    if (window.value("status", "") != "ready") continue;
    bool exported = false;
    for (auto it = snapshot.at("candidates").begin();
         it != snapshot.at("candidates").end(); ++it)
      if (it.value().value("session_id", "") == session_id &&
          it.value().value("window", Json::object()).value("id", "") ==
              window.value("id", "") &&
          it.value().value("evidence", Json(nullptr)).is_object()) {
        exported = true;
        break;
      }
    if (!exported)
      return {{"pre_roll_retired", false},
              {"reason", "incident window still requires export"}};
  }

  for (auto it = snapshot.at("candidates").begin();
       it != snapshot.at("candidates").end(); ++it)
    if (it.value().value("session_id", "") == session_id &&
        !it.value().value("evidence", Json(nullptr)).is_object())
      return {{"pre_roll_retired", false},
              {"reason", "candidate still requires evidence export"}};

  const auto pre_roll = session / "rosbag2";
  const auto reclaimed = directory_bytes(pre_roll);
  std::error_code error;
  fs::remove_all(pre_roll, error);
  require(!error, "could not retire inactive incident pre-roll: " +
                      error.message());
  atomic_json(session / "recording.json",
              {{"schema_version", 2},
               {"mode", "incident_pre_roll"},
               {"retention_ns", recording.value("retention_ns", 30000000000LL)},
               {"segments", Json::array()},
               {"retired_wall_ns", wall_ns()},
               {"reclaimed_bytes", reclaimed}});
  return {{"pre_roll_retired", true}, {"reclaimed_bytes", reclaimed}};
}

} // namespace

Json upload_closed_recordings(const fs::path &store, const fs::path &session,
                              const DeviceCredentials &credentials) {
  const auto recording_path = session / "recording.json";
  const auto session_path = session / "session.json";
  if (!fs::is_regular_file(recording_path) || !fs::is_regular_file(session_path))
    return {{"uploaded", 0}};
  const auto recording = read_json(recording_path);
  const auto session_meta = read_json(session_path);
  if (!session_meta.contains("workflow_store") || session_meta["workflow_store"].is_null() ||
      fs::weakly_canonical(session_meta["workflow_store"].get<std::string>()) !=
          fs::weakly_canonical(store))
    return {{"uploaded", 0}, {"skipped", "session belongs to another workflow store"}};
  const auto config = session_meta.at("config");
  const auto snapshot = Store(store).snapshot();
  const auto profile_id = snapshot.at("profile_id").get<std::string>();
  // Retention is local policy, not an upload acknowledgement policy. Retire
  // inactive flight-recorder chunks before any network request so a cloud or
  // profile failure can never grow the pre-roll into an archive again.
  const auto retired = retire_inactive_pre_roll(
      session, recording, snapshot,
      session_meta.at("session_id").get<std::string>());
  const auto state_path = session / "cloud-recordings.json";
  auto state = load_state(state_path);
  std::set<std::string> uploaded;
  for (auto it = state["operations"].begin(); it != state["operations"].end(); ++it)
    if (it.value().value("status", "") == "submitted") uploaded.insert(it.key());
  unsigned count = 0;
  // Prioritize incident evidence before rolling segments for general playback. Candidates,
  // however, reference a content-addressed MCAP exported to the exact evidence
  // window. Publish that artifact separately so the backend can join the
  // candidate evidence SHA to the recording and render the actual incident.
  for (auto it = snapshot.at("candidates").begin();
       it != snapshot.at("candidates").end(); ++it) {
    const auto &candidate = it.value();
    const auto evidence = candidate.value("evidence", Json(nullptr));
    if (candidate.value("session_id", "") !=
            session_meta.at("session_id").get<std::string>() ||
        !evidence.is_object() || evidence.value("kind", "") != "evidence_mcap")
      continue;
    const auto sha = evidence.at("sha256").get<std::string>();
    const auto relative = "incident/" + it.key() + ".mcap";
    if (uploaded.contains(relative)) continue;
    const auto file = store / "artifacts" / sha;
    require(fs::is_regular_file(file) && file_hash(file) == sha,
            "incident evidence artifact is missing or corrupt");
    const auto &window = candidate.at("window");
    const auto coverage = window.value("coverage", Json::object());
    Json modalities = Json::array();
    for (const auto &topic : config.at("topics")) {
      const auto name = topic.at("name").get<std::string>();
      const auto covered = coverage.value(name, Json::object());
      const auto message_count =
          covered.value("valid_count", covered.value("count", 0U));
      if (message_count == 0U) continue;
      modalities.push_back({
          {"modality_key", digest({topic.at("name"), topic.at("type")})},
          {"kind", modality_kind(topic)}, {"topic", topic.at("name")},
          {"ros2_type", topic.at("type")}, {"message_encoding", "cdr"},
          {"schema_encoding", "ros2msg"}, {"fields", Json::array()},
          {"entities", Json::array()},
          {"metadata", {{"role", topic.value("role", "custom")},
                        {"evidence_candidate_id", it.key()}}},
          {"message_count", message_count},
          {"start_time_ns", window.at("start_ns")},
          {"end_time_ns", window.at("end_ns")}});
    }
    const auto recording_id = digest({{"profile_id", profile_id},
                                      {"candidate_id", it.key()},
                                      {"sha256", sha}});
    const Json payload = {
        {"device_id", credentials.device_id}, {"profile_id", profile_id},
        {"recording_id", recording_id}, {"sha256", sha},
        {"size_bytes", fs::file_size(file)},
        {"start_time_ns", window.at("start_ns")},
        {"end_time_ns", window.at("end_ns")}, {"modalities", modalities},
        {"manifest", {{"schema_version", 1}, {"kind", "incident_evidence"},
                       {"session_id", session_meta.at("session_id")},
                       {"candidate_id", it.key()}, {"window", window}}}};
    try {
      const auto previous_attempts = upload_attempts(state, relative);
      journal(state_path, state, relative, "pending",
              {{"attempts", previous_attempts + 1U}, {"recording_id", recording_id},
               {"sha256", sha}, {"size_bytes", fs::file_size(file)},
               {"local_path", file.string()}, {"last_error", nullptr}});
      bool already_complete = false;
      try {
        backend_request(credentials, "/harness-profiles/edge/recordings/complete", payload);
        already_complete = true;
      } catch (const std::exception &) {
      }
      if (!already_complete) {
        const auto grant = backend_request(
            credentials, "/harness-profiles/edge/recordings/upload", payload);
        journal(state_path, state, relative, "uploading",
                {{"object_key", grant.at("object_key")},
                 {"grant_expires_at", grant.at("expires_at")}});
        put_file(file, grant);
        journal(state_path, state, relative, "uploaded_to_r2");
        backend_request(credentials, "/harness-profiles/edge/recordings/complete", payload);
      }
      journal(state_path, state, relative, "submitted");
    } catch (const std::exception &error) {
      journal(state_path, state, relative, "retrying",
              {{"last_error", error.what()}});
      throw;
    }
    uploaded.insert(relative);
    ++count;
  }
  return {{"uploaded", count},
          {"incident_artifacts_only", true},
          {"pre_roll", retired}};
}

Json run_recording_uploader(const Options &options) {
  options.allow("--store --session --sessions-root --credentials --backend-url --interval-ms");
  const auto store = options.path("--store");
  require(options.values.contains("--session") != options.values.contains("--sessions-root"),
          "pass exactly one of --session or --sessions-root");
  const auto session = options.values.contains("--session")
                           ? options.path("--session") : fs::path();
  const auto sessions_root = options.values.contains("--sessions-root")
                                 ? options.path("--sessions-root") : fs::path();
  const auto credentials_path = options.values.contains("--credentials")
                                    ? options.path("--credentials")
                                    : default_credentials_path();
  const auto credentials = load_device_credentials(
      credentials_path, options.get("--backend-url", ""));
  const auto interval_ms = options.integer("--interval-ms", 2000);
  require(interval_ms >= 250 && interval_ms <= 60000,
          "interval-ms must be between 250 and 60000");
  uploader_stopping = 0;
  ::signal(SIGINT, uploader_stop_handler);
  ::signal(SIGTERM, uploader_stop_handler);
  while (!uploader_stopping) {
    std::vector<fs::path> sessions;
    if (!session.empty()) sessions.push_back(session);
    else if (fs::is_directory(sessions_root))
      for (const auto &entry : fs::directory_iterator(sessions_root))
        if (entry.is_directory()) sessions.push_back(entry.path());
    std::sort(sessions.rbegin(), sessions.rend());
    for (const auto &candidate : sessions) {
      try {
        const auto result = upload_closed_recordings(store, candidate, credentials);
        if (result.value("uploaded", 0U) > 0)
          record_event("recording_uploader", "submitted", "success",
                       {{"session", candidate.string()}, {"result", result}});
      } catch (const std::exception &error) {
        record_event("recording_uploader", "retry", "failure",
                     {{"message", error.what()}, {"session", candidate.string()}});
      }
    }
    const auto slices = static_cast<unsigned>(interval_ms / 100);
    for (unsigned index = 0; index < slices && !uploader_stopping; ++index)
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return {{"stopped", true},
          {"source", session.empty() ? sessions_root.string() : session.string()}};
}

} // namespace harness::observation
