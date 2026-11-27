#include "execution_sampling.hpp"
#include <algorithm>
#include <iostream>
#include <limits>
using namespace harness::observation;
namespace {
constexpr Ns second=1000000000LL;
void check(bool ok,const std::string &why){require(ok,"sampling test: "+why);}
struct Fixture {
  fs::path root=fs::temp_directory_path()/("sampling-test-"+unique_id());
  Fixture(){fs::create_directories(root);atomic_json(root/"execution-sampling-policy.json",{{"mean_interval_s",15.}});}
  ~Fixture(){fs::remove_all(root);}
};
Json snapshot(){return {{"failures",Json::object()},{"candidates",Json::object()}};}
Json observation(){return {{"epoch",0},{"windows",Json::array()}};}
std::optional<Json> operate(ExecutionSampler &s,int seconds,const Json &profile=snapshot(),const Json &state=observation(),bool move=true) {
  std::optional<Json> selected;
  for(int i=0;i<seconds*10;++i){
    const auto now=static_cast<Ns>(i)*100000000LL;
    s.joints({move?static_cast<double>(i)*.01:0.},now);
    const auto result=s.select(100*second+now,now,20000*86400*second,profile,state);
    if(result)selected=result;
  }
  return selected;
}
void activity(){
  // Exercise real defaults: one short operation must not wait minutes for a draw.
  Fixture prompt;fs::remove(prompt.root/"execution-sampling-policy.json");
  ExecutionSampler first(prompt.root,5);
  check(first.status()["next_draw_seconds"]==15.,"first draw is one complete window");
  check(!operate(first,15),"cannot select before 15 eligible seconds");
  first.reset();
  check(operate(first,17).has_value(),"short operation retained with default policy");
  check(first.status()["next_draw_seconds"].get<double>()>=15. &&
        first.status()["next_draw_seconds"].get<double>()<=45.,"default subsequent draw bounded");
  Fixture short_operation;ExecutionSampler short_sampler(short_operation.root,5);
  std::optional<Json> short_window;
  for(int i=0;i<170;++i) {
    const auto now=static_cast<Ns>(i)*100000000LL;
    // Ten seconds of actual motion, then live stationary joint observations.
    short_sampler.joints({static_cast<double>(std::min(i,100))*.01},now);
    if(auto selected=short_sampler.select(100*second+now,now,20000*86400*second,snapshot(),observation()))
      short_window=selected;
  }
  check(short_window.has_value(),"ten-second operation must be sampled during its live post-motion context");
  // Genuine inactivity resets only scheduling, not the durable reservation count.
  for (int i=170;i<501;++i) {
    const auto now=static_cast<Ns>(i)*100000000LL;
    first.joints({1.69},now);
    first.select(100*second+now,now,20000*86400*second,snapshot(),observation());
  }
  check(first.status()["next_draw_seconds"]==15. && first.status()["eligible_seconds_since_draw"]==0.,
        "next operation after inactivity must not inherit a long random wait");
  check(first.status()["reserved_windows"]==1,"idle reset cannot clear reservations");
  first.reset();
  check(first.status()["next_draw_seconds"]==15.,"clock reset gets prompt first window");
  Fixture restarted;fs::remove(restarted.root/"execution-sampling-policy.json");
  {
    ExecutionSampler prior(restarted.root,5);check(operate(prior,17).has_value(),"prior reservation created");
    const auto ledger=read_json(restarted.root/"execution-sampling-ledger.json");
    fs::create_directories(restarted.root/"execution-samples");
    atomic_json(restarted.root/"execution-samples"/(ledger["reservations"][0]["id"].get<std::string>()+".rejected.json"),
                {{"reason","prior rejected export"}});
  }
  {
    ExecutionSampler fresh(restarted.root,5);
    check(fresh.status()["next_draw_seconds"]==15.,"rejected prior reservation cannot delay a new session");
    check(operate(fresh,17).has_value() && fresh.status()["reserved_windows"]==2,
          "new short operation captured without clearing prior quota");
  }
  Fixture f;ExecutionSampler s(f.root,5);
  check(!operate(s,120,snapshot(),observation(),false),"stationary joint traffic cannot trigger");
  s.reset();
  check(operate(s,120).has_value(),"measured movement triggers random sampling");
  s.reset();
  s.command({0.},0);
  for(int i=0;i<1200;++i){
    const auto now=static_cast<Ns>(i)*100000000LL;
    s.joints({0.},now);s.command({0.},now);
    check(!s.select(500*second+now,now,20000*86400*second,snapshot(),observation()),"unchanged commands are not work");
  }
}
void clocks_and_coverage(){
  Fixture f;ExecutionSampler s(f.root,5);
  s.joints({std::numeric_limits<double>::quiet_NaN()},0);
  check(!s.select(100*second,second,0,snapshot(),observation()),"invalid joints rejected");
  Json binding={{"topics",Json::array({{{"name","camera"},{"role","camera"}},{{"name","joint"},{"role","joint_state"}},{{"name","command"},{"role","action"}}})}};
  Json point={{"count",151},{"valid_count",151},{"first",{{"receipt_ros_ns",100*second}}},
              {"last",{{"receipt_ros_ns",115*second}}},{"max_receipt_gap_ns",100000000LL}};
  Json coverage={{"camera",point},{"joint",point}};
  check(sample_coverage(binding,coverage,100*second,115*second),"no command event frequency requirement");
  coverage["camera"]["max_receipt_gap_ns"]=2*second;
  check(!sample_coverage(binding,coverage,100*second,115*second),"missing middle frames rejected");
  coverage["camera"]=point;coverage["joint"]["valid_count"]=150;
  check(!sample_coverage(binding,coverage,100*second,115*second),"invalid joint rejected");
}
void budgets_and_restart(){
  Fixture f;atomic_json(f.root/"execution-sampling-policy.json",{{"mean_interval_s",15.},{"daily_limit",1}});
  {ExecutionSampler s(f.root,5);check(operate(s,120).has_value(),"first daily window");}
  {ExecutionSampler s(f.root,5);check(!operate(s,120),"restart cannot reset daily budget");s.reset();check(!operate(s,120),"clock reset cannot reset budget");}
  Fixture pending;atomic_json(pending.root/"execution-sampling-policy.json",{{"mean_interval_s",15.},{"pending_bytes",1}});
  {ExecutionSampler s(pending.root,5);check(!operate(s,120),"pending byte ceiling");}
  Fixture pool;atomic_json(pool.root/"execution-sampling-policy.json",{{"mean_interval_s",15.},{"pool_bytes",1}});
  {ExecutionSampler s(pool.root,5);check(!operate(s,120),"total byte ceiling");}
}
void incidents_and_labels(){
  Fixture f;ExecutionSampler s(f.root,5);
  auto state=observation();state["windows"].push_back({{"epoch",0},{"start_ns",0},{"end_ns",1000*second}});
  check(!operate(s,120,snapshot(),state),"incident overlap excluded");
  auto alert=state;
  alert["windows"][0]["id"]="experimental-window";
  alert["windows"][0]["detection"]={{"source","experimental_critic"},{"deployment_validated",false}};
  s.reset();check(operate(s,17,snapshot(),alert).has_value(),"unconfirmed alert cannot suppress collection");
  auto confirmed=snapshot();confirmed["candidates"]["confirmed"]={{"status","accepted_failure"},{"window",alert["windows"][0]}};
  check(!unconfirmed_critic_window(alert["windows"][0],confirmed),"accepted failure still blocks ordinary samples");
  check(unconfirmed_critic_window(alert["windows"][0],snapshot()),"experimental alert is review context");
  s.reset();const auto result=operate(s,120);
  check(result && result->at("label_status")=="unlabeled","no automatic negative labels");
  const auto ledger=read_json(f.root/"execution-sampling-ledger.json");
  check(ledger["reservations"].size()<=8,"selection cooldown/nonoverlap");
  Fixture ratio;atomic_json(ratio.root/"execution-sampling-policy.json",{{"mean_interval_s",15.},
    {"bootstrap_limit",1},{"failure_ratio",1},{"pool_limit",2}});
  auto profile=snapshot();profile["failures"]["f1"]={{"candidate_id","c1"}};
  profile["failures"]["f2"]={{"candidate_id","c2"}};
  profile["candidates"]["c1"]={{"status","accepted_failure"},{"evidence",{{"sha256","same-source"}}}};
  profile["candidates"]["c2"]=profile["candidates"]["c1"];
  {ExecutionSampler bounded(ratio.root,5);operate(bounded,120,profile);
    check(bounded.status()["reserved_windows"]==1,"duplicate failures do not inflate ratio");}
  profile["candidates"]["c2"]["evidence"]["sha256"]="another-real-source";
  {ExecutionSampler bounded(ratio.root,5);operate(bounded,120,profile);
    check(bounded.status()["reserved_windows"]==2,"distinct failure grows pool to hard cap");}
}
void policy_and_single_owner(){
  Fixture f;ExecutionSampler s(f.root,5);bool rejected=false;
  try{ExecutionSampler duplicate(f.root,5);}catch(const std::exception &){rejected=true;}
  check(rejected,"profile-wide exclusive sampler lock");
  Fixture disabled;atomic_json(disabled.root/"execution-sampling-policy.json",{{"enabled",false}});
  ExecutionSampler off(disabled.root,5);check(!operate(off,120),"disabled sampling");
  Fixture invalid;atomic_json(invalid.root/"execution-sampling-policy.json",{{"daily_limit",1000}});
  rejected=false;try{ExecutionSampler bad(invalid.root,5);}catch(const std::exception &){rejected=true;}
  check(rejected,"hard ceilings validated");
}
}
int main(){try{activity();clocks_and_coverage();budgets_and_restart();incidents_and_labels();policy_and_single_owner();
  std::cout<<"5 execution-sampling check groups passed\n";return 0;}catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}}
