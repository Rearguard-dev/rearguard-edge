#include "service.hpp"

#include "binding.hpp"
#include "events.hpp"
#include "recording_upload.hpp"
#include "store.hpp"
#include "sync.hpp"

#include <csignal>
#include <cerrno>
#include <cstdlib>
#include <fcntl.h>
#include <fstream>
#include <sys/file.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <optional>

namespace harness::observation {
namespace {
fs::path executable_path;
volatile sig_atomic_t service_stopping = 0;
pid_t observer_pid = -1, sync_pid = -1, uploader_pid = -1;

fs::path service_root() { return state_root() / "service"; }
fs::path config_path() { return service_root() / "config.json"; }
fs::path status_path() { return service_root() / "status.json"; }
fs::path pid_path() { return service_root() / "service.pid"; }

fs::path default_credentials_path() {
  if (const auto *value = std::getenv("XDG_CONFIG_HOME"); value && *value)
    return fs::path(value) / "rearguard" / "device.json";
  const auto *home = std::getenv("HOME");
  require(home && *home, "HOME is not set; pass --credentials");
  return fs::path(home) / ".config" / "rearguard" / "device.json";
}

bool alive(pid_t pid) {
  return pid > 0 && (::kill(pid, 0) == 0 || errno == EPERM);
}

pid_t read_pid() {
  std::ifstream in(pid_path());
  long long value = -1;
  in >> value;
  return static_cast<pid_t>(value);
}

void stop_handler(int) {
  service_stopping = 1;
  if (observer_pid > 0) ::kill(observer_pid, SIGTERM);
  if (sync_pid > 0) ::kill(sync_pid, SIGTERM);
  if (uploader_pid > 0) ::kill(uploader_pid, SIGTERM);
}

std::vector<char *> argv_for(std::vector<std::string> &args) {
  std::vector<char *> result;
  result.reserve(args.size() + 1);
  for (auto &arg : args) result.push_back(arg.data());
  result.push_back(nullptr);
  return result;
}

pid_t spawn(const std::vector<std::string> &input) {
  auto args = input;
  const pid_t pid = ::fork();
  require(pid >= 0, "could not start harness child process");
  if (pid == 0) {
    auto pointers = argv_for(args);
    ::execv(executable_path.c_str(), pointers.data());
    _exit(127);
  }
  return pid;
}

int wait_child(pid_t pid) {
  int status = 0;
  while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {}
  return WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status);
}

int run(const std::vector<std::string> &args) { return wait_child(spawn(args)); }

void write_status(const std::string &state, const Json &config, Json extra = Json::object()) {
  Json value{{"schema_version", 1}, {"state", state}, {"pid", ::getpid()},
             {"updated_wall_ns", wall_ns()}, {"config", config},
             {"observer_pid", observer_pid > 0 ? Json(observer_pid) : Json(nullptr)},
             {"sync_pid", sync_pid > 0 ? Json(sync_pid) : Json(nullptr)},
             {"uploader_pid", uploader_pid > 0 ? Json(uploader_pid) : Json(nullptr)}};
  value.update(extra);
  atomic_json(status_path(), value);
}

Json load_or_empty(const fs::path &path) {
  try { return read_json(path); } catch (...) { return Json::object(); }
}

bool legacy_preset(const Json &value) {
  if (!value.is_string()) return false;
  const auto name = value.get<std::string>();
  return name == "so101" || name == "so101_hardware" ||
         name == "g1_warehouse";
}

bool pristine_workflow(const Json &snapshot) {
  if (snapshot.value("revision", -1LL) != 0) return false;
  for (const auto *field : {"incident_requests", "candidates", "failures",
                            "jobs", "bundles", "deployments"})
    if (snapshot.value(field, Json::object()).size() != 0) return false;
  return true;
}

std::optional<Ns> selected_domain(const Json &config) {
  if (config.contains("domain_id") && config["domain_id"].is_number_integer())
    return config["domain_id"].get<Ns>();
  const auto inventory = load_or_empty(service_root() / "setup-inventory.json");
  return select_observation_domain(inventory);
}

void reap(pid_t &pid, const std::string &component) {
  if (pid <= 0) return;
  int status = 0;
  const auto result = ::waitpid(pid, &status, WNOHANG);
  if (result != pid) return;
  const int code = WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status);
  record_event(component, "stopped", code == 0 ? "success" : "failure",
               {{"exit_code", code}});
  pid = -1;
}

void scan_setup(const Json &config) {
  std::vector<std::string> args{executable_path.string(), "scan", "setup", "--out",
                                (service_root() / "setup-inventory.json").string(),
                                "--domain-min", "0", "--domain-max", "232",
                                "--settle-ms", "750", "--parallelism", "16"};
  const int code = run(args);
  record_event("setup_scan", "complete", code == 0 ? "success" : "failure",
               {{"exit_code", code}});
  write_status(code == 0 ? "configuring" : "waiting_for_ros", config);
}

void export_pending_windows(const fs::path &store, const fs::path &session) {
  try {
    const auto candidates = Store(store).snapshot().at("candidates");
    for (auto it = candidates.begin(); it != candidates.end(); ++it) {
      const auto &candidate = it.value();
      if (!candidate.value("evidence", Json(nullptr)).is_null()) continue;
      if (candidate.value("session_id", "") !=
          load_or_empty(session / "session.json").value("session_id", "")) continue;
      const std::vector<std::string> args{
          executable_path.string(), "workflow", "export-window", "--store", store.string(),
          "--session", session.string(), "--candidate", it.key()};
      const int code = run(args);
      record_event("window_export", it.key(), code == 0 ? "success" : "failure",
                   {{"exit_code", code}, {"session", session.string()}});
    }
  } catch (const std::exception &error) {
    record_event("window_export", "inspect", "failure", {{"message", error.what()}});
  }
}

[[noreturn]] void daemon_loop(Json config) {
  service_stopping = 0;
  fs::create_directories(service_root());
  const int lock = ::open((service_root() / "service.lock").c_str(), O_CREAT | O_RDWR, 0600);
  require(lock >= 0 && ::flock(lock, LOCK_EX | LOCK_NB) == 0,
          "another harness service is already running");
  ::fcntl(lock, F_SETFD, FD_CLOEXEC);
  atomic_json(pid_path(), ::getpid());
  ::signal(SIGINT, stop_handler);
  ::signal(SIGTERM, stop_handler);
  record_event("service", "started", "success", {{"pid", ::getpid()}});
  write_status("starting", config);

  // Migrate releases that persisted a bundled embodiment preset. Those names
  // are no longer runtime bindings; a fresh structural binding is compiled
  // below from the authoritative setup inventory.
  if (legacy_preset(config.value("observation_config", Json(nullptr)))) {
    config["observation_config"] = nullptr;
    atomic_json(config_path(), config);
    record_event("observation_binding", "legacy_preset_cleared", "success");
  }

  if (!fs::is_regular_file(service_root() / "setup-inventory.json"))
    scan_setup(config);
  Ns next_assignment = 0, next_scan = monotonic_ns() + 30000000000LL;
  fs::path active_session;
  std::string active_critic_id;
  while (!service_stopping) {
    reap(observer_pid, "observer");
    reap(sync_pid, "workflow_sync");
    reap(uploader_pid, "recording_uploader");

    if ((!config.contains("robot_id") || config["robot_id"].is_null()) &&
        monotonic_ns() >= next_assignment) {
      try {
        const auto credentials = load_device_credentials(
            config.value("credentials", default_credentials_path().string()),
            config.value("backend_url", ""));
        const auto assignment = fetch_device_assignment(credentials);
        if (assignment.value("assigned", false)) {
          const auto embodiment = assignment.at("embodiment_id").get<std::string>();
          config["robot_id"] = assignment.at("robot_id");
          config["profile_id"] = assignment.at("profile_id");
          config["embodiment"] = embodiment;
          atomic_json(config_path(), config);
          record_event("assignment", "resolved", "success", assignment);
        } else {
          write_status("waiting_for_assignment", config,
                       {{"message", "Assign this device to a robot."}});
        }
      } catch (const std::exception &error) {
        record_event("assignment", "poll", "failure", {{"message", error.what()}});
        write_status("waiting_for_assignment", config, {{"message", error.what()}});
      }
      next_assignment = monotonic_ns() + 5000000000LL;
    }

    auto domain = selected_domain(config);
    if (!domain && monotonic_ns() >= next_scan) {
      scan_setup(config);
      next_scan = monotonic_ns() + 30000000000LL;
      domain = selected_domain(config);
    }
    if (domain && config.contains("robot_id") && config["robot_id"].is_string() &&
        (!config.contains("observation_config") ||
         config["observation_config"].is_null())) {
      try {
        const auto inventory = read_json(service_root() / "setup-inventory.json");
        const auto binding = compile_observation_binding(
            inventory, *domain, config.value("embodiment", "discovered"));
        const auto path = service_root() / "bindings" /
                          (config.value("profile_id", "profile") + ".json");
        fs::create_directories(path.parent_path());
        atomic_json(path, binding);
        // Loading through the runtime validator makes the generated binding
        // obey exactly the same contract as an explicitly supplied config.
        (void)load_config(path);
        config["observation_config"] = path.string();
        const auto store = service_root() / "profiles" /
                           config["robot_id"].get<std::string>();
        if (fs::exists(store / "workflow.sqlite3")) {
          const auto snapshot = Store(store).snapshot();
          if (snapshot.value("binding_hash", "") != digest(binding)) {
            require(pristine_workflow(snapshot),
                    "accepted workflow state uses a different observation binding; explicit profile migration is required");
            const auto backup = fs::path(store.string() + ".legacy-binding-" +
                                         std::to_string(wall_ns()));
            fs::rename(store, backup);
            record_event("observation_binding", "pristine_store_migrated", "success",
                         {{"backup", backup.string()},
                          {"previous_binding_hash", snapshot.value("binding_hash", "")},
                          {"binding_hash", digest(binding)}});
          }
        }
        atomic_json(config_path(), config);
        record_event("observation_binding", "compiled", "success",
                     {{"path", path.string()}, {"domain_id", *domain},
                      {"binding_hash", digest(binding)}});
      } catch (const std::exception &error) {
        record_event("observation_binding", "compile", "failure",
                     {{"message", error.what()}, {"domain_id", *domain}});
        write_status("waiting_for_ros", config, {{"message", error.what()}});
      }
    }
    const bool configured = config.contains("robot_id") && config["robot_id"].is_string() &&
                            config.contains("observation_config") &&
                            config["observation_config"].is_string() && domain.has_value();
    if (configured) {
      const auto store = service_root() / "profiles" / config["robot_id"].get<std::string>();
      if (!fs::exists(store / "workflow.sqlite3")) {
        const int code = run({executable_path.string(), "workflow", "init", "--store",
                              store.string(), "--config",
                              config["observation_config"].get<std::string>(),
                              "--profile-id", config.value("profile_id", "")});
        record_event("workflow", "initialize", code == 0 ? "success" : "failure",
                     {{"exit_code", code}, {"store", store.string()}});
        if (code != 0) {
          write_status("failed", config, {{"message", "workflow initialization failed"}});
          std::this_thread::sleep_for(std::chrono::seconds(5));
          continue;
        }
      }
      if (sync_pid <= 0) {
        std::vector<std::string> args{executable_path.string(), "workflow", "sync", "--store",
          store.string(), "--robot-id", config["robot_id"].get<std::string>()};
        if (config.contains("credentials"))
          args.insert(args.end(), {"--credentials", config["credentials"].get<std::string>()});
        if (config.contains("backend_url"))
          args.insert(args.end(), {"--backend-url", config["backend_url"].get<std::string>()});
        sync_pid = spawn(args);
        record_event("workflow_sync", "started", "success", {{"pid", sync_pid}});
      }
      const auto profile = Store(store).snapshot();
      const auto critic = profile.value("critic_deployment", Json(nullptr));
      // A compiled contract that is still waiting for examples/embeddings is
      // visible in status, but must not disrupt observation or masquerade as
      // a live model. Only an executable generation reaches the observer.
      const auto critic_id =
          critic.is_object() && critic.value("executable", false)
              ? critic.value("id", "")
              : "";
      if (observer_pid > 0 && critic_id != active_critic_id) {
        ::kill(observer_pid, SIGTERM);
        wait_child(observer_pid);
        observer_pid = -1;
        record_event("observer", "reconfigure_critic", "success",
                     {{"previous", active_critic_id}, {"next", critic_id}});
        if (!critic_id.empty())
          record_event("critic", "deployed", "success",
                       {{"deployment_id", critic_id},
                        {"mode", critic.value("mode", "shadow")},
                        {"runtime", critic.value("runtime", "unknown")},
                        {"trained", critic.value("trained", false)}});
      }
      if (observer_pid <= 0) {
        const auto available = fs::space(service_root()).available;
        if (available < 1000000000ULL) {
          write_status("storage_full", config,
                       {{"available_bytes", available},
                        {"error", "Recording paused: less than 1 GB free disk space. Free space before reporting an incident."}});
          std::this_thread::sleep_for(std::chrono::seconds(2));
          continue;
        }
        active_session = service_root() / "sessions" /
                         (std::to_string(wall_ns()) + "-d" + std::to_string(*domain));
        observer_pid = spawn({executable_path.string(), "observe", "start", "--config",
          config["observation_config"].get<std::string>(), "--session", active_session.string(),
          "--store", store.string(), "--storage", config.value("storage", "mcap"),
          "--domain-id", std::to_string(*domain), "--wall-timeout", "86400"});
        record_event("observer", "started", "success",
                     {{"pid", observer_pid}, {"domain_id", *domain},
                      {"session", active_session.string()}});
        active_critic_id = critic_id;
      }
      if (uploader_pid <= 0) {
        std::vector<std::string> args{
            executable_path.string(), "service", "uploader", "--store", store.string(),
            "--sessions-root", (service_root() / "sessions").string(), "--credentials",
            config.value("credentials", default_credentials_path().string()),
            "--interval-ms", "2000"};
        if (config.contains("backend_url"))
          args.insert(args.end(), {"--backend-url", config["backend_url"].get<std::string>()});
        uploader_pid = spawn(args);
        record_event("recording_uploader", "started", "success",
                     {{"pid", uploader_pid},
                      {"sessions_root", (service_root() / "sessions").string()}});
      }
      export_pending_windows(store, active_session);
      write_status("running", config, {{"domain_id", *domain},
                   {"store", store.string()}, {"session", active_session.string()},
                   {"critic", critic},
                   {"critic_profile_ready", profile.value("critic_profile_ready", false)},
                   {"critic_runtime_status",
                    profile.value("critic_runtime_status", "awaiting_profile_setup")}});
    }
    for (int i = 0; i < 10 && !service_stopping; ++i)
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  if (observer_pid > 0) wait_child(observer_pid);
  if (sync_pid > 0) wait_child(sync_pid);
  if (uploader_pid > 0) wait_child(uploader_pid);
  observer_pid = sync_pid = uploader_pid = -1;
  write_status("stopped", config);
  record_event("service", "stopped", "success");
  std::error_code ec;
  fs::remove(pid_path(), ec);
  ::flock(lock, LOCK_UN);
  ::close(lock);
  _exit(0);
}

Json start(Json changes) {
  fs::create_directories(service_root());
  if (alive(read_pid())) return {{"status", "already_running"}, {"pid", read_pid()}};
  auto config = load_or_empty(config_path());
  config.update(changes);
  config["schema_version"] = 1;
  atomic_json(config_path(), config);
  const pid_t pid = ::fork();
  require(pid >= 0, "could not start harness service");
  if (pid == 0) {
    require(::setsid() >= 0, "could not create harness service session");
    const int input = ::open("/dev/null", O_RDONLY);
    const int log = ::open((service_root() / "service.log").c_str(),
                           O_CREAT | O_APPEND | O_WRONLY, 0600);
    if (input >= 0) { ::dup2(input, STDIN_FILENO); ::close(input); }
    if (log >= 0) { ::dup2(log, STDOUT_FILENO); ::dup2(log, STDERR_FILENO); ::close(log); }
    daemon_loop(config);
  }
  for (int i = 0; i < 20 && !alive(read_pid()); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  return {{"status", "started"}, {"pid", pid}, {"state_dir", service_root().string()}};
}
} // namespace

void set_service_executable(const fs::path &path) {
#if defined(__linux__)
  std::error_code ec;
  const auto self = fs::read_symlink("/proc/self/exe", ec);
  executable_path = ec ? fs::absolute(path) : self;
#else
  executable_path = fs::absolute(path);
#endif
}

const fs::path &service_executable() { return executable_path; }

Json start_service_after_connect() {
  Json changes = Json::object();
  const auto credentials = load_or_empty(default_credentials_path());
  if (credentials.contains("device_id") && credentials["device_id"].is_string()) {
    const auto device_id = credentials["device_id"].get<std::string>();
    const auto previous = load_or_empty(config_path());
    if (previous.value("device_id", "") != device_id) {
      stop_service_if_running();
      changes = {{"device_id", device_id}, {"robot_id", nullptr},
                 {"profile_id", nullptr},
                 {"embodiment", nullptr}, {"observation_config", nullptr}};
    }
  }
  return start(changes);
}

Json stop_service_if_running() {
  const auto pid = read_pid();
  if (!alive(pid)) return {{"status", "not_running"}};
  require(::kill(pid, SIGTERM) == 0, "could not stop harness service");
  for (int i = 0; i < 100 && alive(pid); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  return {{"status", alive(pid) ? "stop_requested" : "stopped"}, {"pid", pid}};
}

Json service_command(const Options &options) {
  if (options.command == "uploader") return run_recording_uploader(options);
  if (options.command == "start") {
    options.allow("--config --robot-id --domain-id --storage --credentials --backend-url");
    Json changes = Json::object();
    if (options.values.contains("--config")) changes["observation_config"] = options.need("--config");
    if (options.values.contains("--robot-id")) changes["robot_id"] = options.need("--robot-id");
    if (options.values.contains("--domain-id")) changes["domain_id"] = options.integer("--domain-id", -1);
    if (options.values.contains("--storage")) changes["storage"] = options.need("--storage");
    if (options.values.contains("--credentials")) changes["credentials"] = options.need("--credentials");
    if (options.values.contains("--backend-url")) changes["backend_url"] = options.need("--backend-url");
    return start(changes);
  }
  if (options.command == "status") {
    options.allow("");
    auto result = load_or_empty(status_path());
    if (result.empty()) result = {{"state", "not_started"}};
    result["running"] = alive(read_pid());
    result["state_dir"] = service_root().string();
    return result;
  }
  if (options.command == "stop") {
    options.allow("");
    return stop_service_if_running();
  }
  throw std::runtime_error("unknown service command: " + options.command);
}

} // namespace harness::observation
