#include "execution_sampling.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

namespace harness::observation {
namespace {
constexpr Ns second = 1000000000LL;
bool changed(const std::vector<double> &a, const std::vector<double> &b, double delta) {
  if (a.size() != b.size() || a.empty()) return false;
  for (std::size_t i=0;i<a.size();++i)
    if (std::abs(a[i]-b[i]) >= delta) return true;
  return false;
}
bool valid(const std::vector<double> &a) {
  return !a.empty() && std::all_of(a.begin(),a.end(),[](double x){return std::isfinite(x);});
}
}
bool unconfirmed_critic_window(const Json &window,const Json &snapshot) {
  const auto detection=window.value("detection",Json::object());
  if (detection.value("source","")!="experimental_critic" ||
      detection.value("deployment_validated",false)) return false;
  for (const auto &candidate:snapshot.value("candidates",Json::object()))
    if (candidate.value("status","")=="accepted_failure" &&
        candidate.value("window",Json::object()).value("id","")==window.value("id","")) return false;
  return true;
}
ExecutionSampler::ExecutionSampler(fs::path root, unsigned seed):root_(std::move(root)),random_(seed) {
  policy_={{"schema_version",1},{"enabled",true},{"mean_interval_s",30.},
    {"joint_delta",.01},{"daily_limit",12},{"bootstrap_limit",40},{"failure_ratio",3},
    {"pool_limit",200},{"max_window_bytes",64*1024*1024},
    {"pool_bytes",4294967296LL},{"pending_bytes",268435456LL}};
  const auto path=root_/"execution-sampling-policy.json";
  if (fs::exists(path)) {
    const auto configured=read_json(path);
    for (auto it=configured.begin();it!=configured.end();++it) {
      require(policy_.contains(it.key()),"unknown execution sampling policy field");
      policy_[it.key()]=it.value();
    }
  }
  require(policy_["schema_version"]==1 && policy_["enabled"].is_boolean(),"invalid sampling schema/enabled");
  for (const auto *key:{"mean_interval_s","joint_delta"})
    require(policy_[key].is_number() && std::isfinite(policy_[key].get<double>()) && policy_[key].get<double>()>0,
            "sampling intervals/deltas must be positive finite numbers");
  require(policy_["mean_interval_s"].get<double>()>=15,"sampling interval must be at least 15 seconds");
  for (const auto *key:{"daily_limit","bootstrap_limit","failure_ratio","pool_limit","max_window_bytes","pool_bytes","pending_bytes"})
    require(policy_[key].is_number_integer() && policy_[key].get<Ns>()>0 && policy_[key].get<Ns>()<=4294967296LL,
            "invalid sampling budget");
  require(policy_["pool_limit"].get<Ns>()<=200 && policy_["daily_limit"].get<Ns>()<=200 &&
          policy_["bootstrap_limit"].get<Ns>()<=policy_["pool_limit"].get<Ns>() &&
          policy_["max_window_bytes"].get<Ns>()<=64*1024*1024,"sampling hard ceiling exceeded");
  const auto fd=::open((root_/"execution-sampling.lock").c_str(),O_CREAT|O_RDWR|O_CLOEXEC,0600);
  require(fd>=0,"cannot open sampling lock");
  if (::flock(fd,LOCK_EX|LOCK_NB)!=0) {::close(fd);throw std::runtime_error("another sampler owns this profile");}
  lock_=fd;
  try {
  const auto ledger=root_/"execution-sampling-ledger.json";
  ledger_=fs::exists(ledger)?read_json(ledger):Json{{"schema_version",1},{"reservations",Json::array()}};
  require(ledger_["schema_version"]==1 && ledger_["reservations"].is_array(),"invalid sampling ledger");
    next_s_=15.; // Each new session gets a prompt first draw, even with old reservations.
  } catch (...) {::close(lock_);lock_=-1;throw;}
}
ExecutionSampler::~ExecutionSampler() {if(lock_>=0) ::close(lock_);}
void ExecutionSampler::next() {
  // Later draws within ongoing work are bounded and score-independent.
  // First draws belong to activity/session state, never to the persistent quota ledger.
  const auto mean=policy_["mean_interval_s"].get<double>();
  std::uniform_real_distribution<double> distribution(15.,2.*mean-15.);
  next_s_=distribution(random_);
  eligible_s_=0;
}
void ExecutionSampler::reset() {
  anchor_.clear();target_.clear();anchor_at_=last_joint_=activity_at_=motion_at_=last_tick_=last_end_=-1;
  eligible_s_=0;next_s_=15.; // Reset scheduling, never persistent quotas.
}
void ExecutionSampler::joints(const std::vector<double> &positions,Ns now) {
  if (!valid(positions)) return;
  last_joint_=now;
  if (anchor_at_<0 || now-anchor_at_>second || positions.size()!=anchor_.size()) {
    anchor_=positions;anchor_at_=now;return;
  }
  if (now-anchor_at_<100000000LL) return;
  if (changed(anchor_,positions,policy_["joint_delta"].get<double>())) activity_at_=motion_at_=now;
  if (now-anchor_at_>=500000000LL) {anchor_=positions;anchor_at_=now;}
}
void ExecutionSampler::command(const std::vector<double> &target,Ns now) {
  if (!valid(target)) return;
  if (changed(target_,target,policy_["joint_delta"].get<double>())) activity_at_=now;
  target_=target; // A first target or repeated heartbeat is not evidence of work.
}
std::optional<Json> ExecutionSampler::select(Ns ros,Ns now,Ns wall,const Json &snapshot,const Json &observation) {
  const auto elapsed=last_tick_<0?0.:static_cast<double>(now-last_tick_)/1e9;
  last_tick_=now;
  state_="waiting_for_activity";
  if (activity_at_<0 || now-activity_at_>20*second) {
    eligible_s_=0;next_s_=15.; // A new operation after inactivity gets a complete first window.
  }
  if (!policy_["enabled"].get<bool>() || elapsed<=0 || elapsed>2 || last_joint_<0 ||
      now-last_joint_>500000000LL || activity_at_<0 || now-activity_at_>20*second) return std::nullopt;
  eligible_s_+=elapsed;
  state_="waiting_for_random_draw";
  if (eligible_s_<next_s_ || ros<15*second || (last_end_>=0 && ros-15*second<last_end_)) return std::nullopt;
  next();
  for (const auto &w:observation.at("windows"))
    if (!unconfirmed_critic_window(w,snapshot) && w.at("epoch")==observation.at("epoch") &&
        w.at("start_ns").get<Ns>()<=ros && w.at("end_ns").get<Ns>()>=ros-15*second) return std::nullopt;
  std::set<std::string> failures;
  for (const auto &f:snapshot.at("failures")) {
    const auto id=f.value("candidate_id","");
    if (snapshot.at("candidates").contains(id)) {
      const auto &c=snapshot.at("candidates").at(id);
      if (c.value("status","")=="accepted_failure" && c.value("evidence",Json(nullptr)).is_object())
        failures.insert(c.at("evidence").at("sha256").get<std::string>());
    }
  }
  const auto capacity=std::min(policy_["pool_limit"].get<std::size_t>(),std::max(
      policy_["bootstrap_limit"].get<std::size_t>(),policy_["failure_ratio"].get<std::size_t>()*failures.size()));
  const auto day=wall/(86400*second);
  Ns daily=0,bytes=0,pending=0;
  for (const auto &r:ledger_.at("reservations")) {
    const auto amount=r.at("reserved_bytes").get<Ns>();bytes+=amount;
    if (r.at("day").get<Ns>()==day) ++daily;
    if (!fs::exists(root_/"execution-samples"/(r.at("id").get<std::string>()+".submitted.json")) &&
        !fs::exists(root_/"execution-samples"/(r.at("id").get<std::string>()+".rejected.json"))) pending+=amount;
  }
  const auto reserve=policy_["max_window_bytes"].get<Ns>();
  if (fs::space(root_).available<static_cast<std::uintmax_t>(250000000+2*reserve)) {
    state_="disk_budget_paused";return std::nullopt;
  }
  if (ledger_["reservations"].size()>=capacity || daily>=policy_["daily_limit"].get<Ns>() ||
      bytes+reserve>policy_["pool_bytes"].get<Ns>() || pending+reserve>policy_["pending_bytes"].get<Ns>()) {
    state_="budget_paused";return std::nullopt;
  }
  const auto id="sample-"+unique_id();
  ledger_["reservations"].push_back({{"id",id},{"day",day},{"reserved_bytes",reserve}});
  atomic_json(root_/"execution-sampling-ledger.json",ledger_); // Reserve before capture; crashes cannot exceed quotas.
  last_end_=ros;
  next(); // The reservation now exists, so subsequent draws use the bounded cadence.
  state_="selected_unlabeled_window";
  return Json{{"id",id},{"reason",motion_at_>=0 && now-motion_at_<second?"measured_motion":"execution_pause_or_command_change"},
    {"label_status","unlabeled"},{"max_bytes",reserve},{"policy",policy_},{"accepted_real_failure_sources",failures.size()}};
}
Json ExecutionSampler::status() const {
  return {{"enabled",policy_["enabled"]},{"status",state_},{"policy",policy_},{"reserved_windows",ledger_.at("reservations").size()},
    {"eligible_seconds_since_draw",eligible_s_},{"next_draw_seconds",next_s_},
    {"pool_behavior","pause_at_capacity_no_automatic_deletion"}};
}
bool sample_coverage(const Json &binding,const Json &coverage,Ns start,Ns end) {
  for (const auto &t:binding.at("topics")) {
    if (t.value("role","")=="action") continue; // Commands are events, not a fixed-rate stream.
    const auto name=t.at("name").get<std::string>();
    if (!coverage.contains(name)) return false;
    const auto &c=coverage.at(name);
    if (c.value("valid_count",0)<2 || c.value("count",0)!=c.value("valid_count",0) ||
        c.value("max_receipt_gap_ns",end-start)>second || c.at("first").is_null() || c.at("last").is_null() ||
        c.at("first").at("receipt_ros_ns").get<Ns>()-start>second ||
        end-c.at("last").at("receipt_ros_ns").get<Ns>()>second) return false;
  }
  return true;
}
} // namespace harness::observation
