#include "recording_upload.hpp"
#include "events.hpp"
#include "execution_sampling.hpp"

#include <algorithm>
#include <curl/curl.h>
#include <csignal>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <set>
#include <regex>
#include <thread>
#include <sys/resource.h>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

namespace harness::observation {
Ns recording_retry_delay_ns(unsigned attempts) {
  return std::min<Ns>(300,5LL*(1LL<<std::min(attempts?attempts-1:0U,6U)))*1000000000LL;
}
namespace {
volatile sig_atomic_t uploader_stopping = 0;

void uploader_stop_handler(int) { uploader_stopping = 1; }

struct HttpSession {
  CURL *handle=curl_easy_init();
  ~HttpSession(){if(handle)curl_easy_cleanup(handle);}
  CURL *reset(){require(handle!=nullptr,"cannot initialize upload HTTP session");curl_easy_reset(handle);return handle;}
};

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
  static thread_local HttpSession session;
  CURL *curl = session.reset();
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
  require(result == CURLE_OK, "recording backend request failed: " + failure);
  require(status >= 200 && status < 300,
          "recording backend returned HTTP " + std::to_string(status) +
              ": " + response.substr(0, 1000));
  return response.empty() ? Json::object() : Json::parse(response);
}

void put_file(const fs::path &file, const Json &grant) {
  FILE *input = std::fopen(file.c_str(), "rb");
  require(input != nullptr, "cannot open MCAP for upload");
  static thread_local HttpSession session;
  CURL *curl = session.reset();
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

// Spool selected windows before any network call. Unavailable pre-roll is
// rejected, never replaced by a different interval or called a negative.
void spool_execution_samples_unlocked(const fs::path &store,const fs::path &session,const Json &meta,const Json &snapshot,const Json &observation) {
  const auto directory=store/"execution-samples";
  fs::create_directories(directory);
  for (const auto &window:observation.value("windows",Json::array())) {
    if (!window.contains("execution_sample") || window.value("status","")!="ready") continue;
    const auto selection=window.at("execution_sample");
    const auto id=selection.at("id").get<std::string>();
    require(std::regex_match(id,std::regex("sample-[A-Za-z0-9-]+")),"invalid execution sample identity");
    if (fs::exists(directory/(id+".json")) || fs::exists(directory/(id+".rejected.json"))) continue;
    const auto temporary=directory/(id+".export");
    try {
      Json overlaps=Json::array();
      for (const auto &other:observation.at("windows"))
        if (!other.contains("execution_sample") && other.at("epoch")==window.at("epoch") &&
            other.at("start_ns").get<Ns>()<=window.at("end_ns").get<Ns>() &&
            other.at("end_ns").get<Ns>()>=window.at("start_ns").get<Ns>()) {
          require(unconfirmed_critic_window(other,snapshot),
                  "sample overlaps reported or confirmed incident; excluded from ordinary-operation pool");
          overlaps.push_back({{"window_id",other.at("id")},{"detection",other.at("detection")}});
        }
      fs::remove_all(temporary);
      const auto exported=export_window(session,window,temporary);
      const auto path=directory/(id+".mcap");
      fs::rename(exported,path);
      fs::remove_all(temporary);
      atomic_json(directory/(id+".json"),{{"schema_version",1},{"kind","execution_sample"},
        {"sample_id",id},{"session_id",meta.at("session_id")},{"window",window},
        {"sha256",file_hash(path)},{"size_bytes",fs::file_size(path)},
        {"label_status","unlabeled"},{"training_eligible",false},
        {"binding_hash",digest(meta.at("config"))},{"unconfirmed_alert_overlaps",overlaps}});
      try {record_event("execution_sampling","spooled","success",{{"sample_id",id}});}
      catch(const std::exception &) {} // A logging error cannot delete a committed sample.
    } catch (const std::exception &error) {
      fs::remove_all(temporary);
      fs::remove(directory/(id+".mcap"));
      atomic_json(directory/(id+".rejected.json"),{{"reason",error.what()},{"sample_id",id}});
      record_event("execution_sampling","rejected","failure",{{"sample_id",id},{"reason",error.what()}});
    }
  }
}

unsigned upload_execution_samples(const fs::path &store,const Json &config,const std::string &profile,
                                  const DeviceCredentials &credentials) {
  const auto directory=store/"execution-samples";
  if (!fs::is_directory(directory)) return 0;
  unsigned uploaded=0;
  for (const auto &entry:fs::directory_iterator(directory)) {
    const auto filename=entry.path().filename().string();
    if (!std::regex_match(filename,std::regex("sample-[A-Za-z0-9-]+\\.json"))) continue;
    const auto stem=entry.path().stem().string();
    if (fs::exists(directory/(stem+".submitted.json"))) continue;
    const auto sample=read_json(entry.path());
    if (sample.value("kind","")!="execution_sample" || sample.at("binding_hash")!=digest(config)) continue;
    const auto id=sample.at("sample_id").get<std::string>();
    const auto submitted=directory/(id+".submitted.json");
    if (fs::exists(submitted)) continue;
    const auto retry_path=directory/(id+".retry.json");
    auto retry=fs::exists(retry_path)?read_json(retry_path):Json{{"attempts",0}};
    if (retry.value("next_retry_wall_ns",Ns{0})>wall_ns()) continue;
    const auto attempts=retry.value("attempts",0U)+1U;
    try {
    const auto path=directory/(id+".mcap");
    require(fs::is_regular_file(path) && fs::file_size(path)==sample.at("size_bytes").get<std::uintmax_t>() &&
            file_hash(path)==sample.at("sha256").get<std::string>(),"sample MCAP missing or changed");
    Json modalities=Json::array();
    const auto &window=sample.at("window");
    for (const auto &topic:config.at("topics")) {
      const auto name=topic.at("name").get<std::string>();
      const auto count=window.at("coverage").value(name,Json::object()).value("valid_count",0);
      if (count==0) continue;
      modalities.push_back({{"modality_key",digest({topic.at("name"),topic.at("type")})},
        {"kind",modality_kind(topic)},{"topic",name},{"ros2_type",topic.at("type")},
        {"message_encoding","cdr"},{"schema_encoding","ros2msg"},{"fields",Json::array()},{"entities",Json::array()},
        {"metadata",{{"role",topic.value("role","custom")}}},{"message_count",count},
        {"start_time_ns",window.at("start_ns")},{"end_time_ns",window.at("end_ns")}});
    }
    const auto recording=digest({{"profile_id",profile},{"sample_id",id},{"sha256",sample.at("sha256")}});
    const Json payload={{"device_id",credentials.device_id},{"profile_id",profile},{"recording_id",recording},
      {"sha256",sample.at("sha256")},{"size_bytes",sample.at("size_bytes")},
      {"start_time_ns",window.at("start_ns")},{"end_time_ns",window.at("end_ns")},{"modalities",modalities},
      {"manifest",sample}};
    bool complete=false;
    if (attempts>1) {
      try {backend_request(credentials,"/harness-profiles/edge/recordings/complete",payload);complete=true;}
      catch (const std::exception &) {}
    }
    atomic_json(retry_path,{{"attempts",attempts},{"next_retry_wall_ns",wall_ns()+recording_retry_delay_ns(attempts)}});
    if (!complete) {
      const auto grant=backend_request(credentials,"/harness-profiles/edge/recordings/upload",payload);
      put_file(path,grant);
      backend_request(credentials,"/harness-profiles/edge/recordings/complete",payload);
    }
    atomic_json(submitted,{{"recording_id",recording},{"submitted_wall_ns",wall_ns()},{"status","submitted_for_verification"}});
    ++uploaded;
    } catch (const std::exception &error) {
      atomic_json(retry_path,{{"attempts",attempts},{"next_retry_wall_ns",wall_ns()+recording_retry_delay_ns(attempts)},
                             {"error",error.what()}});
      record_event("execution_sampling","upload_retry","failure",{{"sample_id",id},{"attempts",attempts}});
    }
  }
  return uploaded;
}

Json retire_inactive_pre_roll(const fs::path &store, const fs::path &session, const Json &recording,
                              const Json &snapshot,
                              const std::string &session_id) {
  if (recording.contains("retired_wall_ns") && recording.at("segments").empty() && !fs::exists(session/"rosbag2"))
    return {{"pre_roll_retired",true},{"already_retired",true}};
  const auto state_path = session / "state.json";
  if (!fs::is_regular_file(state_path))
    return {{"pre_roll_retired", false}, {"reason", "session state missing"}};
  const auto observation = read_json(state_path);
  const auto heartbeat_value =
      observation.value("heartbeat_wall_ns", Json(nullptr));
  const auto heartbeat = heartbeat_value.is_number_integer()
                             ? heartbeat_value.get<Ns>()
                             : Ns{0};
  const auto heartbeat_age = wall_ns() - heartbeat;
  if (observation.value("status", "") == "observing" && heartbeat > 0 &&
      heartbeat_age >= 0 && heartbeat_age < 30000000000LL)
    return {{"pre_roll_retired", false}, {"reason", "observer active"}};

  for (const auto &window : observation.value("windows", Json::array())) {
    if (window.contains("execution_sample")) {
      const auto id=window.at("execution_sample").at("id").get<std::string>();
      require(std::regex_match(id,std::regex("sample-[A-Za-z0-9-]+")),"invalid execution sample identity");
      const auto directory=store/"execution-samples";
      if (!fs::exists(directory/(id+".json")) && !fs::exists(directory/(id+".rejected.json")))
        return {{"pre_roll_retired",false},{"reason","execution sample still requires local export"}};
      continue;
    }
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

bool compact_recording_journal(const fs::path &path) {
  if (!fs::is_regular_file(path)) return false;
  const auto original=read_json(path);
  auto state=load_state(path);
  bool changed=false;
  for (auto &operation:state["operations"])
    if (operation.value("status","")=="submitted" && operation.size()>1) {
      operation={{"status","submitted"}};changed=true;
    }
  if (!changed) return false;
  // Archive once before replacement. Pending/retrying operations are untouched;
  // submitted tombstones stay in the active journal to prohibit retransmission.
  const auto archive=fs::path(path.string()+".before-compaction-"+digest(original)+".json");
  if (!fs::exists(archive)) atomic_json(archive,original);
  atomic_json(path,state);
  return true;
}

void spool_execution_samples(const fs::path &store,const fs::path &session) {
  if (!fs::is_regular_file(session/"session.json") || !fs::is_regular_file(session/"state.json")) return;
  const auto meta=read_json(session/"session.json");
  if (!meta.contains("workflow_store") || !meta["workflow_store"].is_string() ||
      fs::weakly_canonical(meta["workflow_store"].get<std::string>())!=fs::weakly_canonical(store)) return;
  const auto observation=read_json(session/"state.json");
  bool pending=false;
  for (const auto &window:observation.value("windows",Json::array())) {
    if (!window.contains("execution_sample") || window.value("status","")!="ready") continue;
    const auto id=window.at("execution_sample").at("id").get<std::string>();
    require(std::regex_match(id,std::regex("sample-[A-Za-z0-9-]+")),"invalid execution sample identity");
    const auto directory=store/"execution-samples";
    if (!fs::exists(directory/(id+".json")) && !fs::exists(directory/(id+".rejected.json"))) pending=true;
  }
  if (!pending) return; // Don't read a multi-MB profile snapshot on every heartbeat.
  const auto fd=::open((store/"execution-spool.lock").c_str(),O_CREAT|O_RDWR|O_CLOEXEC,0600);
  require(fd>=0,"cannot open execution spool lock");
  if (::flock(fd,LOCK_EX|LOCK_NB)!=0) {::close(fd);throw std::runtime_error("execution spool busy");}
  try {
    spool_execution_samples_unlocked(store,session,meta,Store(store).snapshot(),observation);
  } catch (...) {::close(fd);throw;}
  ::close(fd);
}

ExecutionSampleSpooler::ExecutionSampleSpooler(fs::path store,fs::path sessions_root)
  :worker_([store=std::move(store),sessions_root=std::move(sessions_root)](std::stop_token stop) {
    std::map<fs::path,fs::file_time_type> processed;
    while (!stop.stop_requested()) {
      try {
        if (fs::is_directory(sessions_root)) for (const auto &entry:fs::directory_iterator(sessions_root)) {
          if (stop.stop_requested()) break;
          const auto state=entry.path()/"state.json";
          if (!entry.is_directory() || !fs::is_regular_file(state)) continue;
          const auto modified=fs::last_write_time(state);
          if (processed.contains(state) && processed.at(state)==modified) continue;
          try {
            spool_execution_samples(store,entry.path());
            processed[state]=modified;
          } catch (const std::exception &error) {
            record_event("execution_sampling","spool_retry","failure",{{"session",entry.path().string()},{"message",error.what()}});
          }
        }
      } catch (const std::exception &error) {
        try {record_event("execution_sampling","spool_scan","failure",{{"message",error.what()}});} catch (...) {}
      }
      for (unsigned i=0;i<5 && !stop.stop_requested();++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }) {}

Json upload_closed_recordings(const fs::path &store, const fs::path &session,
                              const DeviceCredentials &credentials, bool include_execution_samples, bool spool_local, const Json *profile_snapshot) {
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
  const auto owned_snapshot=profile_snapshot?std::shared_ptr<const Json>{}:Store(store).snapshot_shared();
  const auto &snapshot=profile_snapshot?*profile_snapshot:*owned_snapshot;
  const auto profile_id = snapshot.at("profile_id").get<std::string>();
  if (spool_local) spool_execution_samples(store,session);
  // Retention is local policy, not an upload acknowledgement policy. Retire
  // inactive flight-recorder chunks before any network request so a cloud or
  // profile failure can never grow the pre-roll into an archive again.
  const auto retired = retire_inactive_pre_roll(
      store, session, recording, snapshot,
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
    if (state["operations"].contains(relative) && state["operations"][relative].value("next_retry_wall_ns",Ns{0})>wall_ns()) continue;
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
      if (previous_attempts>0) try {
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
              {{"last_error", error.what()},
               {"next_retry_wall_ns",wall_ns()+recording_retry_delay_ns(upload_attempts(state,relative))}});
      throw;
    }
    uploaded.insert(relative);
    ++count;
  }
  const auto samples=include_execution_samples?upload_execution_samples(store,config,profile_id,credentials):0U;
  return {{"uploaded", count+samples},{"incident_artifacts",count},{"execution_samples",samples},
          {"incident_artifacts_only", false},
          {"pre_roll", retired}};
}

Json run_recording_uploader(const Options &options) {
  // Export, hashing and TLS are background work; don't compete equally with
  // robot observation/inference when the edge CPU is busy.
  (void)::setpriority(PRIO_PROCESS,0,10);
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
  // Start local preservation independently of the network drain. A 600-second
  // PUT must never prevent a selected window from leaving the 30-second ring.
  ExecutionSampleSpooler spooler(store,session.empty()?sessions_root:session.parent_path());
  ::signal(SIGINT, uploader_stop_handler);
  ::signal(SIGTERM, uploader_stop_handler);
  std::set<fs::path> compacted;
  Store workflow(store);
  while (!uploader_stopping) {
    std::shared_ptr<const Json> profile_snapshot;
    try {profile_snapshot=workflow.snapshot_shared();}
    catch (const std::exception &error) {
      record_event("recording_uploader","profile_retry","failure",{{"message",error.what()}});
      for (Ns elapsed=0;elapsed<interval_ms && !uploader_stopping;elapsed+=100)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue; // Local spooling stays independent; never drain using stale authorization metadata.
    }
    std::vector<fs::path> sessions;
    if (!session.empty()) sessions.push_back(session);
    else if (fs::is_directory(sessions_root))
      for (const auto &entry : fs::directory_iterator(sessions_root))
        if (entry.is_directory()) sessions.push_back(entry.path());
    std::sort(sessions.rbegin(), sessions.rend());
    std::set<std::string> sample_bindings;
    for (const auto &candidate : sessions) {
      try {
        if (!fs::is_regular_file(candidate/"session.json")) continue;
        const auto meta=read_json(candidate/"session.json");
        if (!meta.contains("workflow_store") || !meta["workflow_store"].is_string() ||
            fs::weakly_canonical(meta["workflow_store"].get<std::string>())!=fs::weakly_canonical(store)) continue;
        const auto binding=digest(meta.at("config"));
        const bool samples=!sample_bindings.contains(binding);
        if (!compacted.contains(candidate)) {
          compact_recording_journal(candidate/"cloud-recordings.json");
          compacted.insert(candidate);
        }
        const auto result = upload_closed_recordings(store, candidate, credentials, samples, false,profile_snapshot.get());
        if (!result.contains("skipped")) sample_bindings.insert(binding);
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
