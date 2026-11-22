#include "sync.hpp"
#include "events.hpp"
#include "model_cache.hpp"
#include "runtime.hpp"

#include <curl/curl.h>
#include <signal.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <regex>
#include <thread>
#include <fstream>
#include <cctype>

namespace harness::observation {
namespace {

constexpr const char *default_backend_url = "https://info-32291--api.modal.run";
volatile sig_atomic_t sync_interrupted = 0;

void stop_sync(int) { sync_interrupted = 1; }

bool uuid_like(const std::string &value) {
  static const std::regex pattern(
      "^[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-"
      "[0-9a-fA-F]{4}-[0-9a-fA-F]{12}$");
  return std::regex_match(value, pattern);
}

std::string default_credentials_path() {
  const auto *home = std::getenv("HOME");
  require(home != nullptr && *home != '\0',
          "HOME is not set; pass --credentials explicitly");
  const auto *xdg = std::getenv("XDG_CONFIG_HOME");
  return (xdg && *xdg ? fs::path(xdg) : fs::path(home) / ".config") /
         "rearguard/device.json";
}

std::size_t append_response(char *data, std::size_t size, std::size_t count,
                            void *output) {
  const auto bytes = size * count;
  if (bytes > 4*1024*1024 || static_cast<std::string *>(output)->size() > 4*1024*1024-bytes) return 0;
  static_cast<std::string *>(output)->append(data, bytes);
  return bytes;
}

class CurlGlobal {
public:
  CurlGlobal() {
    require(curl_global_init(CURL_GLOBAL_DEFAULT) == CURLE_OK,
            "could not initialize HTTPS support");
  }
  ~CurlGlobal() { curl_global_cleanup(); }
};

class BackendTransport final : public WorkflowTransport {
public:
  explicit BackendTransport(DeviceCredentials credentials)
      : credentials_(std::move(credentials)), connection_(curl_easy_init()) {
    require(connection_ != nullptr, "could not initialize HTTPS request");
  }
  ~BackendTransport() override { curl_easy_cleanup(connection_); }
  BackendTransport(const BackendTransport &) = delete;
  BackendTransport &operator=(const BackendTransport &) = delete;

  void upload(const std::string &robot_id, const Json &snapshot,
              const Json &receipts) override {
    request("POST", "/harness-profiles/edge/snapshot",
            {{"device_id", credentials_.device_id},
             {"robot_id", robot_id},
             {"snapshot", snapshot},
             {"receipts", receipts}});
  }

  Json pending(const std::string &profile_id) override {
    CURL *curl = curl_easy_init();
    require(curl != nullptr, "could not initialize HTTPS request");
    char *profile = curl_easy_escape(curl, profile_id.c_str(),
                                     static_cast<int>(profile_id.size()));
    char *device = curl_easy_escape(curl, credentials_.device_id.c_str(),
                                    static_cast<int>(credentials_.device_id.size()));
    require(profile != nullptr && device != nullptr,
            "could not encode workflow command request");
    const std::string path = "/harness-profiles/edge/" +
                             std::string(profile) + "/commands?device_id=" +
                             std::string(device);
    curl_free(profile);
    curl_free(device);
    curl_easy_cleanup(curl);
    const auto started_wall = wall_ns(), started_mono = monotonic_ns();
    auto result = request("GET", path, nullptr);
    const auto ended_wall = wall_ns(), elapsed = monotonic_ns() - started_mono;
    require(result.is_array(), "backend returned a non-array command queue");
    for (auto &command : result) {
      if (command.value("operation", "") != "report_incident" ||
          !command.at("payload").contains("trigger")) continue;
      try {
        command["_aligned_trigger"] = align_incident_trigger(command, started_wall, ended_wall, elapsed);
      } catch (const std::exception &error) {
        command["_trigger_error"] = error.what();
      }
      // A slow queue request must not be the only clock sample. Probe the
      // lightweight endpoint before rejecting, keeping the original click.
      for (int attempt = 0; attempt < 2 && command.contains("_trigger_error") &&
           command.at("_trigger_error").get<std::string>().find(
               "incident clock uncertainty exceeds") == 0; ++attempt) {
        try {
          const auto probe_wall = wall_ns(), probe_mono = monotonic_ns();
          const auto clock = request("GET", "/harness-profiles/clock", nullptr);
          const auto probe_end = wall_ns(), probe_elapsed = monotonic_ns() - probe_mono;
          auto sampled = command;
          sampled["_server_clock"] = {
              {"received_ns", std::to_string(static_cast<Ns>(
                  std::llround(clock.at("received_ms").get<double>() * 1e6)))},
              {"sent_ns", std::to_string(static_cast<Ns>(
                  std::llround(clock.at("sent_ms").get<double>() * 1e6)))}};
          command["_aligned_trigger"] = align_incident_trigger(
              sampled, probe_wall, probe_end, probe_elapsed);
          command.erase("_trigger_error");
        } catch (const std::exception &error) {
          command["_trigger_error"] = error.what();
        }
      }
    }
    return result;
  }

  void upload_enforcements(const std::string &robot_id,
                           const Json &records) override {
    request("POST", "/harness-profiles/edge/enforcements",
            {{"device_id", credentials_.device_id},
             {"robot_id", robot_id}, {"records", records}});
  }

  Json assignment() {
    CURL *curl = curl_easy_init();
    require(curl != nullptr, "could not initialize HTTPS request");
    char *device = curl_easy_escape(curl, credentials_.device_id.c_str(),
                                    static_cast<int>(credentials_.device_id.size()));
    require(device != nullptr, "could not encode assignment request");
    const std::string path = "/harness-profiles/edge/assignment?device_id=" +
                             std::string(device);
    curl_free(device);
    curl_easy_cleanup(curl);
    return request("GET", path, nullptr);
  }

  Json model(const std::string &etag) {
    return request("GET","/harness-profiles/edge/critic-model?device_id="+
      credentials_.device_id+"&tensor_contracts=2",nullptr,etag);
  }

private:
  DeviceCredentials credentials_;
  CURL *connection_;

  Json request(const std::string &method, const std::string &path,
               const Json &payload,const std::string &etag={}) {
    // Reset request options, retaining libcurl's connection/DNS/TLS caches.
    // Clock probes should not pay for a new handshake on every sample.
    CURL *curl = connection_;
    curl_easy_reset(curl);
    std::string response;
    char error[CURL_ERROR_SIZE]{};
    const std::string url = credentials_.backend_url + path;
    const std::string body = payload.is_null() ? "" : payload.dump();
    const std::string secret_header =
        "X-Rearguard-Device-Secret: " + credentials_.secret;
    curl_slist *headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    headers = curl_slist_append(headers, secret_header.c_str());
    if (!etag.empty()) {
      require(std::regex_match(etag,std::regex("\"[0-9a-f]{64}\"")),"invalid model ETag");
      headers=curl_slist_append(headers,("If-None-Match: "+etag).c_str());
    }
    std::string response_etag;
    curl_easy_setopt(curl,CURLOPT_HEADERFUNCTION,+[](char *p,std::size_t s,std::size_t n,void *target)->std::size_t {
      const auto bytes=s*n;
      std::string line(p,bytes);std::string lower=line;
      std::transform(lower.begin(),lower.end(),lower.begin(),[](unsigned char c){return static_cast<char>(std::tolower(c));});
      if (lower.starts_with("etag:")) {
        auto value=line.substr(5);const auto first=value.find_first_not_of(" \t\r\n");
        if (first!=std::string::npos) value=value.substr(first,value.find_last_not_of(" \t\r\n")-first+1);
        if (value.size()<256) *static_cast<std::string *>(target)=value;
      }
      return bytes;
    });
    curl_easy_setopt(curl,CURLOPT_HEADERDATA,&response_etag);
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, append_response);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, error);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "rearguard-workflow-sync/0.3");
    if (method == "POST") {
      curl_easy_setopt(curl, CURLOPT_POST, 1L);
      curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
      curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE,
                       static_cast<long>(body.size()));
    }
    const auto code = curl_easy_perform(curl);
    long status = 0;
    if (code == CURLE_OK)
      curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
    const std::string failure =
        error[0] != '\0' ? error : curl_easy_strerror(code);
    // Drop pointers to this request's stack data before their lifetime ends.
    curl_easy_reset(curl);
    curl_slist_free_all(headers);
    require(code == CURLE_OK, "workflow backend request failed: " + failure);
    if (status==304) return {{"status","not_modified"}};
    if (status < 200 || status >= 300) {
      std::string detail = response;
      try {
        const auto parsed = Json::parse(response);
        detail = parsed.value("detail", parsed.value("message", response));
      } catch (const Json::exception &) {
      }
      if (detail.size() > 1000) detail.resize(1000);
      throw std::runtime_error("workflow backend returned HTTP " +
                               std::to_string(status) + ": " + detail);
    }
    if (response.empty()) return Json::object();
    try {
      auto parsed=Json::parse(response);
      if (!response_etag.empty() && parsed.is_object()) parsed["_etag"]=response_etag;
      return parsed;
    } catch (const Json::exception &) {
      throw std::runtime_error("workflow backend returned invalid JSON");
    }
  }
};

void download_model_file(const Json &file,const fs::path &path,std::stop_token stop) {
  const auto url=file.at("url").get<std::string>();
  require(url.starts_with("https://") && file.at("headers").empty(),"unsupported model download URL or headers");
  struct Output {std::ofstream stream;std::size_t remaining;std::stop_token stop;};
  Output output{std::ofstream(path,std::ios::binary|std::ios::trunc),file.at("size").get<std::size_t>(),stop};
  require(output.stream.good(),"cannot stage model download");
  CURL *curl=curl_easy_init();require(curl!=nullptr,"cannot initialize model download");
  curl_easy_setopt(curl,CURLOPT_URL,url.c_str());
  curl_easy_setopt(curl,CURLOPT_CONNECTTIMEOUT,10L);
  curl_easy_setopt(curl,CURLOPT_TIMEOUT,60L);
  curl_easy_setopt(curl,CURLOPT_FOLLOWLOCATION,0L);
#if LIBCURL_VERSION_NUM >= 0x075500
  curl_easy_setopt(curl,CURLOPT_PROTOCOLS_STR,"https");
#else
  curl_easy_setopt(curl,CURLOPT_PROTOCOLS,CURLPROTO_HTTPS);
#endif
  curl_easy_setopt(curl,CURLOPT_WRITEFUNCTION,+[](char *p,std::size_t s,std::size_t n,void *target)->std::size_t {
    auto &out=*static_cast<Output *>(target);const auto bytes=s*n;
    if (out.stop.stop_requested() || bytes>out.remaining) return 0;
    out.stream.write(p,static_cast<std::streamsize>(bytes));out.remaining-=bytes;
    return out.stream.good()?bytes:0;
  });
  curl_easy_setopt(curl,CURLOPT_WRITEDATA,&output);
  curl_easy_setopt(curl,CURLOPT_NOPROGRESS,0L);
  curl_easy_setopt(curl,CURLOPT_XFERINFOFUNCTION,+[](void *p,curl_off_t,curl_off_t,curl_off_t,curl_off_t)->int {
    return static_cast<Output *>(p)->stop.stop_requested()?1:0;
  });
  curl_easy_setopt(curl,CURLOPT_XFERINFODATA,&output);
  const auto code=curl_easy_perform(curl);long status=0;
  curl_easy_getinfo(curl,CURLINFO_RESPONSE_CODE,&status);curl_easy_cleanup(curl);
  output.stream.close();
  require(code==CURLE_OK && status==200 && output.remaining==0 && output.stream.good(),"model download failed or size differs");
}

void sync_models(const DeviceCredentials &credentials,const fs::path &root,std::stop_token stop) {
  BackendTransport transport(credentials);ModelCache cache(root);std::string etag;
  while (!stop.stop_requested()) {
    try {
      auto response=transport.model(etag);
      if (response.value("status","")!="not_modified") {
        const auto next_etag=response.value("_etag","");response.erase("_etag");
        const auto result=cache.refresh(response,[&](const Json &file,const fs::path &path){
          download_model_file(file,path,stop);
        },[&](const fs::path &path,const Json &selection){
          const auto profile=Store(root.parent_path()).snapshot();
          require(selection.at("profile_id")==profile.at("profile_id") && selection.at("binding_hash")==profile.at("binding_hash"),"resolved physical model belongs to another profile or binding");
          return make_physical_critic(path,profile.at("binding"),selection)->ready();
        });
        etag=next_etag;
        record_event("physical_model","resolve",result.value("status","")=="loaded"?"success":"pending",result);
      }
    } catch (const std::exception &error) {
      if (!stop.stop_requested()) {
        atomic_json(root/"status.json",{{"status","update_failed"},{"error",error.what()},{"active",cache.active()}});
        record_event("physical_model","update","failure",{{"message",error.what()}});
      }
    }
    for (int i=0;i<600 && !stop.stop_requested();++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

} // namespace

Json align_incident_trigger(const Json &command, Ns started_wall, Ns ended_wall,
                            Ns elapsed_monotonic) {
  require(std::abs((ended_wall - started_wall) - elapsed_monotonic) <= 10000000LL,
          "edge wall clock changed during trigger alignment; retry capture");
  const auto &clock = command.at("_server_clock");
  const Ns received = std::stoll(clock.at("received_ns").get<std::string>());
  const Ns sent = std::stoll(clock.at("sent_ns").get<std::string>());
  const auto processing = sent - received;
  require(processing >= 0 && elapsed_monotonic >= processing - 2000000LL,
          "invalid server clock sample");
  auto trigger = command.at("payload").at("trigger");
  require(trigger.value("source", "") == "ui_click", "unsupported incident trigger");
  const auto click_ms = trigger.at("server_trigger_ms").get<double>();
  const auto browser_uncertainty = trigger.at("clock_uncertainty_ms").get<double>();
  require(std::isfinite(click_ms) && std::isfinite(browser_uncertainty) &&
              browser_uncertainty >= 0 && browser_uncertainty <= 250,
          "invalid browser trigger clock");
  const auto uncertainty = browser_uncertainty +
      static_cast<double>(std::max<Ns>(0, elapsed_monotonic - processing)) / 2000000.0 + 2;
  require(uncertainty <= 500, "incident clock uncertainty exceeds 500 ms; retry capture");
  require(click_ms > static_cast<double>(received) / 1e6 - 10000 &&
              click_ms <= static_cast<double>(sent) / 1e6 + browser_uncertainty,
          "incident trigger expired or is in the future");
  const auto local_midpoint = started_wall + (ended_wall - started_wall) / 2;
  const auto server_midpoint = received + (sent - received) / 2;
  const Ns mark = static_cast<Ns>(std::llround(click_ms * 1e6)) + local_midpoint - server_midpoint;
  trigger["edge_trigger_wall_ns"] = std::to_string(mark);
  trigger["edge_received_wall_ns"] = std::to_string(ended_wall);
  trigger["total_clock_uncertainty_ms"] = uncertainty;
  trigger["server_to_edge_offset_ns"] = std::to_string(local_midpoint - server_midpoint);
  return trigger;
}

WorkflowConnector::WorkflowConnector(Store store, std::string robot_id,
                                     WorkflowTransport &transport,
                                     IncidentCapture incident_capture)
    : store_(std::move(store)), robot_id_(std::move(robot_id)),
      transport_(transport), incident_capture_(std::move(incident_capture)) {
  require(uuid_like(robot_id_), "--robot-id must be a UUID");
}

Json WorkflowConnector::cycle(bool heartbeat) {
  auto snapshot = store_.snapshot();
  const auto physical=store_.root/"models"/"runtime.json";
  if(fs::is_regular_file(physical)) {
    try {snapshot["physical_critic"]=read_json(physical);} catch(const std::exception &) {}
  }
  auto receipts = store_.pending_sync_receipts();
  const auto revision = snapshot.at("revision").get<Ns>();
  const bool initial_upload = uploaded_revision_ < 0;
  const bool changed = revision != uploaded_revision_;
  const bool heartbeat_due =
      heartbeat &&
      (last_upload_monotonic_ns_ == 0 ||
       monotonic_ns() - last_upload_monotonic_ns_ >= 30000000000LL);
  if (changed || heartbeat_due || !receipts.empty()) {
    transport_.upload(robot_id_, snapshot, receipts);
    store_.acknowledge_sync_receipts(receipts);
    uploaded_revision_ = revision;
    last_upload_monotonic_ns_ = monotonic_ns();
  }

  const auto commands =
      transport_.pending(snapshot.at("profile_id").get<std::string>());
  for (const auto &command : commands) {
    require(command.is_object(), "workflow command must be an object");
    if (command.value("operation", "") == "report_incident") {
      const auto request_id = required_text(command.at("request_id"), "request_id");
      if (store_.replay_remote_receipt(request_id))
        continue;
      auto applied = command;
      try {
        require(command.at("expected_revision") == store_.snapshot().at("revision"),
                "profile revision changed before incident capture");
        require(!command.contains("_trigger_error"), command.value("_trigger_error", "trigger alignment failed"));
        require(!command.at("payload").contains("trigger") || command.contains("_aligned_trigger"),
                "click-anchored capture requires an upgraded edge connector");
        require(static_cast<bool>(incident_capture_),
                "incident capture requires the active edge observer");
        applied["payload"]["window"] = incident_capture_(command);
      } catch (const std::exception &error) {
        applied["payload"]["capture_error"] = error.what();
      }
      store_.apply_remote(applied);
    } else {
      store_.apply_remote(command);
    }
  }
  if (!commands.empty()) {
    snapshot = store_.snapshot();
    receipts = store_.pending_sync_receipts();
    require(!receipts.empty(), "workflow command did not produce a receipt");
    transport_.upload(robot_id_, snapshot, receipts);
    store_.acknowledge_sync_receipts(receipts);
    uploaded_revision_ = snapshot.at("revision").get<Ns>();
    last_upload_monotonic_ns_ = monotonic_ns();
  }
  const auto enforcements = store_.pending_enforcements();
  if (!enforcements.empty()) {
    transport_.upload_enforcements(robot_id_, enforcements);
    store_.acknowledge_enforcements(enforcements);
  }
  return {{"profile_id", snapshot.at("profile_id")},
          {"revision", snapshot.at("revision")},
          {"initial_upload", initial_upload},
          {"snapshot_uploaded", changed || heartbeat_due || !receipts.empty() ||
                                    !commands.empty()},
          {"commands_received", commands.size()},
          {"enforcements_uploaded", enforcements.size()},
          {"critic_deployment", snapshot.value("critic_deployment", Json(nullptr))},
          {"receipts", receipts}};
}

DeviceCredentials load_device_credentials(
    const fs::path &path, const std::string &backend_override) {
  const auto credentials = read_json(path);
  DeviceCredentials result{
      required_text(credentials.at("device_id"), "device_id"),
      required_text(credentials.at("credential"), "credential"),
      backend_override.empty()
          ? credentials.value("backend_url", std::string(default_backend_url))
          : backend_override};
  require(result.secret.size() == 64 &&
              result.secret.find_first_not_of("0123456789abcdef") ==
                  std::string::npos,
          "device credential must be 64 lowercase hexadecimal characters");
  require(uuid_like(result.device_id), "paired device ID must be a UUID");
  while (!result.backend_url.empty() && result.backend_url.back() == '/')
    result.backend_url.pop_back();
  require(result.backend_url.starts_with("https://"),
          "workflow backend URL must use HTTPS");
  return result;
}

Json fetch_device_assignment(const DeviceCredentials &credentials) {
  CurlGlobal curl;
  return BackendTransport(credentials).assignment();
}

Json run_workflow_sync(const Options &options, bool once) {
  options.allow("--store --robot-id --credentials --backend-url --interval-ms");
  const auto credential_value = options.get("--credentials");
  const auto credentials_path = credential_value.empty()
                                    ? fs::path(default_credentials_path())
                                    : fs::path(credential_value);
  const auto credentials = load_device_credentials(
      credentials_path, options.get("--backend-url"));
  const auto interval_ms = options.integer("--interval-ms", 2000);
  require(interval_ms >= 250 && interval_ms <= 60000,
          "--interval-ms must be between 250 and 60000");
  CurlGlobal curl;
  BackendTransport transport(credentials);
  auto capture_incident = [](const Json &command) {
    const auto status = read_json(state_root() / "service" / "status.json");
    if (status.value("state", "") != "running" ||
        !status.contains("session") || !status["session"].is_string()) {
      // Surface the supervisor's actionable reason. A generic missing-session
      // error hid conditions such as the disk safety floor from the operator.
      const auto reason = status.value(
          "error", status.value(
                       "message",
                       "incident capture requires a running observer session"));
      throw std::runtime_error(reason);
    }
    const auto session = fs::path(status["session"].get<std::string>());
    const auto meta = read_json(session / "session.json");
    const auto &payload = command.at("payload");
    const auto capture_request_id =
        "incident-" + command.at("request_id").get<std::string>();
    // The observer persists state before acknowledging the ROS request. This
    // closes the connector-crash gap: a retry recovers the original edge mark
    // instead of creating another incident window.
    const auto observation = read_json(session / "state.json");
    for (const auto &window : observation.value("windows", Json::array()))
      if (window.value("capture_request_id", "") == capture_request_id)
        return window;
    Json request = {{"schema_version", 1}, {"request_id", capture_request_id},
                    {"operation", "incident_capture"},
                    {"label", required_text(payload.at("label"), "label")}};
    if (command.contains("_aligned_trigger")) request["trigger"] = command.at("_aligned_trigger");
    if (payload.contains("capture_context")) request["capture_context"] = payload.at("capture_context");
    return ros_request_payload(request, meta);
  };
  WorkflowConnector connector(Store(options.path("--store")),
                              options.need("--robot-id"), transport,
                              capture_incident);
  if (once) {
    try {
      const auto result = connector.cycle(true);
      record_event("workflow_sync", "cycle", "success", result);
      return result;
    } catch (const std::exception &error) {
      record_event("workflow_sync", "cycle", "failure", {{"message", error.what()}});
      throw;
    }
  }

  sync_interrupted = 0;
  std::jthread model_sync([&](std::stop_token stop){sync_models(credentials,options.path("--store")/"models",stop);});
  const auto previous_interrupt = ::signal(SIGINT, stop_sync);
  const auto previous_terminate = ::signal(SIGTERM, stop_sync);
  Ns failures = 0;
  while (!sync_interrupted) {
    try {
      const auto result = connector.cycle(true);
      failures = 0;
      if (result.value("snapshot_uploaded", false) ||
          result.value("commands_received", 0U) > 0 ||
          result.value("enforcements_uploaded", 0U) > 0) {
        record_event("workflow_sync", "cycle", "success", result);
        std::cout << result.dump() << std::endl;
      }
    } catch (const std::exception &error) {
      ++failures;
      record_event("workflow_sync", "cycle", "failure",
                   {{"message", error.what()}, {"consecutive_failures", failures}});
      std::cerr << "workflow sync: " << error.what() << std::endl;
    }
    const auto backoff = std::min<Ns>(interval_ms * (1LL << std::min<Ns>(failures, 5)),
                                     60000);
    for (Ns waited = 0; waited < backoff && !sync_interrupted; waited += 100)
      std::this_thread::sleep_for(
          std::chrono::milliseconds(std::min<Ns>(100, backoff - waited)));
  }
  ::signal(SIGINT, previous_interrupt);
  ::signal(SIGTERM, previous_terminate);
  return {{"status", "stopped"}};
}

} // namespace harness::observation
