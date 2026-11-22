// Synthetic loopback transport fixture. Never connects to a real backend.
#include "recording_upload.hpp"
#include <fstream>
#include <iostream>
using namespace harness::observation;
int main(int argc,char **argv) {
  if(argc!=2 || !std::string(argv[1]).starts_with("http://127.0.0.1:")) return 2;
  const auto root=fs::temp_directory_path()/("recording-upload-test-"+unique_id());
  try {
    auto store=Store::create(root/"store",HARNESS_OBSERVATION_CONFIG);
    const auto binding=store.snapshot().at("binding");
    const auto session=root/"session";fs::create_directories(session);
    atomic_json(session/"session.json",{{"config",binding},{"workflow_store",store.root.string()},{"session_id","synthetic"}});
    atomic_json(session/"recording.json",{{"segments",Json::array()}});
    atomic_json(session/"state.json",{{"status","observing"},{"heartbeat_wall_ns",wall_ns()},{"windows",Json::array()}});
    const auto directory=store.root/"execution-samples";fs::create_directories(directory);
    auto sample=[&](const std::string &id){
      const auto path=directory/(id+".mcap");{std::ofstream out(path);out<<"synthetic transport fixture; not robot footage";}
      Json coverage=Json::object();for(const auto &topic:binding.at("topics"))coverage[topic.at("name").get<std::string>()]={{"valid_count",2}};
      atomic_json(directory/(id+".json"),{{"kind","execution_sample"},{"sample_id",id},
        {"sha256",file_hash(path)},{"size_bytes",fs::file_size(path)},{"binding_hash",digest(binding)},
        {"window",{{"start_ns",1000000000LL},{"end_ns",16000000000LL},{"coverage",coverage}}}});
    };
    DeviceCredentials credentials{"synthetic-device","synthetic-secret",argv[1]};
    sample("sample-fresh");
    require(upload_closed_recordings(store.root,session,credentials)["execution_samples"]==1,"fresh transfer failed");
    // Submitted entries must be skipped before even parsing their manifests.
    {std::ofstream out(directory/"sample-fresh.json");out<<"not JSON";}
    require(upload_closed_recordings(store.root,session,credentials)["execution_samples"]==0,"submitted transfer repeated");
    sample("sample-retry");
    require(upload_closed_recordings(store.root,session,credentials)["execution_samples"]==0,"lost completion should defer");
    const auto retry_path=directory/"sample-retry.retry.json";auto retry=read_json(retry_path);
    require(retry["attempts"]==1 && retry["next_retry_wall_ns"].get<Ns>()>wall_ns(),"backoff was not durable");
    require(upload_closed_recordings(store.root,session,credentials)["execution_samples"]==0,"retry should wait");
    retry["next_retry_wall_ns"]=0;atomic_json(retry_path,retry);
    require(upload_closed_recordings(store.root,session,credentials)["execution_samples"]==1,"completion reconciliation failed");
    require(recording_retry_delay_ns(1)==5000000000LL && recording_retry_delay_ns(2)==10000000000LL &&
            recording_retry_delay_ns(1000)==300000000000LL,"retry bounds");
    fs::remove_all(root);std::cout<<"Transport fixture passed\n";return 0;
  }catch(const std::exception &e){fs::remove_all(root);std::cerr<<e.what()<<'\n';return 1;}
}
