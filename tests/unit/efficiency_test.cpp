#include "store.hpp"
#include "runtime.hpp"
#include <sqlite3.h>
#include <chrono>
#include <iostream>
#include <limits>
using namespace harness::observation;
namespace {
struct Fixture {
  fs::path root=fs::temp_directory_path()/("edge-efficiency-"+unique_id());
  Store store=Store::create(root/"store",HARNESS_OBSERVATION_CONFIG);
  ~Fixture(){fs::remove_all(root);}
};
void write_profile(const fs::path &root,const Json &body) {
  sqlite3 *db=nullptr;require(sqlite3_open((root/"workflow.sqlite3").c_str(),&db)==SQLITE_OK,"fixture database open");
  sqlite3_stmt *statement=nullptr;
  require(sqlite3_prepare_v2(db,"UPDATE profile SET body=? WHERE id=1",-1,&statement,nullptr)==SQLITE_OK,"fixture prepare");
  const auto text=body.dump();sqlite3_bind_text(statement,1,text.c_str(),static_cast<int>(text.size()),SQLITE_TRANSIENT);
  const auto result=sqlite3_step(statement);sqlite3_finalize(statement);sqlite3_close(db);
  require(result==SQLITE_DONE,"fixture update");
}
void cache_updates() {
  Fixture f;Store other(f.store.root);
  const auto first=f.store.snapshot_shared();
  require(f.store.snapshot_shared()==first,"unchanged read reparsed profile");
  auto next=*first;next["enforcement_status"]="blocked"; // No revision bump: data_version is authoritative.
  write_profile(f.store.root,next);
  const auto updated=f.store.snapshot_shared();
  require(updated!=first && updated->at("enforcement_status")=="blocked","external commit not observed immediately");
  require(first->at("enforcement_status")!="blocked","retained immutable snapshot changed");
  require(other.snapshot_shared()->at("enforcement_status")=="blocked","other reader did not observe commit");
  require(f.store.snapshot()==*updated,"legacy snapshot contract changed");
  write_profile(f.store.root,Json("invalid profile"));
  bool failed=false;try{f.store.snapshot_shared();}catch(const std::exception &){failed=true;}
  require(failed,"corrupt committed profile served old cached authorization");
  write_profile(f.store.root,next);
  require(f.store.snapshot_shared()->at("enforcement_status")=="blocked","reader did not recover after repair");
}
void cache_fail_closed_and_replacement() {
  Fixture f;const auto initial=f.store.snapshot_shared();
  fs::rename(f.store.root,f.root/"original");
  bool failed=false;try{f.store.snapshot_shared();}catch(const std::exception &){failed=true;}
  require(failed,"missing database served stale cached authorization");
  auto replacement=Store::create(f.store.root,HARNESS_OBSERVATION_CONFIG);
  auto body=replacement.snapshot();body["profile_id"]="replacement-profile";write_profile(replacement.root,body);
  require(f.store.snapshot_shared()->at("profile_id")=="replacement-profile","inode replacement retained old authorization");
  require(initial->at("profile_id")!="replacement-profile","old reader reference mutated");
}
void transports() {
  const Json jpeg={{"name","/camera/image_raw/compressed"},{"role","camera"},{"type","sensor_msgs/msg/CompressedImage"}};
  const Json zstd={{"name","/camera/image_raw/zstd"},{"role","camera"},{"type","sensor_msgs/msg/CompressedImage"}};
  const Json depth={{"name","/camera/image_raw/compressedDepth"},{"role","camera"},{"type","sensor_msgs/msg/CompressedImage"}};
  const Json other={{"name","/other/image_raw/zstd"},{"role","camera"},{"type","sensor_msgs/msg/CompressedImage"}};
  Json binding={{"topics",Json::array({zstd,jpeg,depth,other})}};const auto hash=digest(binding);
  require(observation_topics(binding)==Json::array({jpeg,depth,other}),"color alias deduplication changed other streams");
  require(digest(binding)==hash,"runtime projection mutated frozen binding");
  binding["topics"]=Json::array({zstd});require(observation_topics(binding)==binding["topics"],"standalone Zstd stream removed");
}
void inference_inputs() {
  Observations input;input.receipt_ros_ns=1000;
  require(!has_recent_input(input,250),"empty history schedules inference");
  input.messages.push_back({"joints","state",{},750});require(has_recent_input(input,250),"freshness boundary dropped");
  input.receipt_ros_ns=1001;require(!has_recent_input(input,250),"expired history schedules inference");
  input.messages.push_back({"camera","image",{},1100});require(!has_recent_input(input,250),"future packet accepted");
  input.messages.push_back({"joints","state",{},1001});require(has_recent_input(input,250),"new input did not resume inference");
}
void benchmark() {
  Fixture f;auto body=f.store.snapshot();
  for(unsigned i=0;i<1024;++i)body["candidates"][std::to_string(i)]={{"status","pending_review"},{"payload",std::string(2048,'x')}};
  write_profile(f.store.root,body);const auto retained=f.store.snapshot_shared();
  const auto start=std::chrono::steady_clock::now();
  for(unsigned i=0;i<20;++i){Store cold(f.store.root);require(cold.snapshot().at("candidates").size()==1024,"cold benchmark read");}
  const auto middle=std::chrono::steady_clock::now();
  for(unsigned i=0;i<20;++i)require(f.store.snapshot_shared()==retained,"steady state allocated another snapshot");
  const auto end=std::chrono::steady_clock::now();
  std::cout<<Json{{"iterations",20},{"synthetic_candidates",1024},
    {"repeated_store_and_copy_ms",std::chrono::duration<double,std::milli>(middle-start).count()},
    {"shared_snapshot_ms",std::chrono::duration<double,std::milli>(end-middle).count()}}.dump()<<'\n';
}
}
int main(){try{cache_updates();cache_fail_closed_and_replacement();transports();inference_inputs();benchmark();
 std::cout<<"5 efficiency check groups passed\n";return 0;}catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}}
