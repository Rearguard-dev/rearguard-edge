// Synthetic loopback transport fixture. Never connects to a real backend.
#include "recording_upload.hpp"
#include <fstream>
#include <iostream>
using namespace harness::observation;
int main(int argc,char **argv) {
  // Synthetic-only pipeline soak helper. The Python test owns a loopback mock
  // and isolated store; this mode never reads device credentials.
  if(argc==5 && std::string(argv[1])=="--pipeline") {
    const fs::path store_root=argv[2],sessions=argv[3];
    try {
      require(std::string(argv[4]).starts_with("http://127.0.0.1:"),"pipeline fixture requires loopback mock");
      Store store(store_root);
      DeviceCredentials credentials{"00000000-0000-4000-8000-000000000001",std::string(64,'0'),argv[4]};
      ExecutionSampleSpooler spooler(store_root,sessions);
      const auto deadline=monotonic_ns()+240000000000LL;
      while(!fs::exists(store_root/"test.stop") && monotonic_ns()<deadline) {
        const auto profile=store.snapshot_shared();
        for(const auto &entry:fs::directory_iterator(sessions))
          if(fs::exists(entry.path()/"session.json"))
            try{upload_closed_recordings(store_root,entry.path(),credentials,true,false,profile.get());}
            catch(const std::exception &e){std::cerr<<e.what()<<'\n';}
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
      return 0;
    }catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
  }
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
    const auto profile=store.snapshot_shared();
    require(upload_closed_recordings(store.root,session,credentials,true,true,profile.get())["execution_samples"]==1,"fresh transfer failed");
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
    // The mock holds the HTTP grant until the local worker processes a window
    // which appears after the request starts. No real MCAP/credentials involved.
    sample("sample-blocked");
    auto blocked=read_json(directory/"sample-blocked.json");
    blocked["test_state_path"]=(session/"state.json").string();
    blocked["test_marker_path"]=(directory/"sample-local-blocked.rejected.json").string();
    atomic_json(directory/"sample-blocked.json",blocked);
    {
      ExecutionSampleSpooler spooler(store.root,root);
      require(upload_closed_recordings(store.root,session,credentials,true,false)["execution_samples"]==1,
              "blocked network prevented local work");
      require(fs::exists(directory/"sample-local-blocked.rejected.json"),"local worker did not run during blocked HTTP");
    }
    // Inactive pre-roll cannot be retired ahead of the independent local worker.
    fs::create_directories(session/"rosbag2");
    atomic_json(session/"state.json",{{"status","stopped"},{"windows",Json::array({
      {{"execution_sample",{{"id","sample-not-yet-spooled"}}},{"status","ready"}}})}});
    const auto waiting=upload_closed_recordings(store.root,session,credentials,false,false);
    require(waiting["pre_roll"]["pre_roll_retired"]==false && fs::exists(session/"rosbag2"),"retirement raced local export");
    atomic_json(directory/"sample-not-yet-spooled.rejected.json",{{"reason","synthetic expired sample"}});
    const auto retired=upload_closed_recordings(store.root,session,credentials,false,false);
    require(retired["pre_roll"]["pre_roll_retired"]==true && !fs::exists(session/"rosbag2"),"terminal sample blocks retirement");
    const auto journal=session/"cloud-recordings.json";
    atomic_json(journal,{{"schema_version",2},{"operations",{
      {"incident/done.mcap",{{"status","submitted"},{"object_key","synthetic"},{"attempts",2}}},
      {"incident/pending.mcap",{{"status","retrying"},{"attempts",3},{"next_retry_wall_ns",123}}}}}});
    const auto before=read_json(journal);
    require(compact_recording_journal(journal),"completed journal not compacted");
    const auto compacted=read_json(journal);
    require(compacted["operations"]["incident/done.mcap"]==Json{{"status","submitted"}},"completion acknowledgement lost");
    require(compacted["operations"]["incident/pending.mcap"]==before["operations"]["incident/pending.mcap"],"pending transfer changed");
    require(read_json(fs::path(journal.string()+".before-compaction-"+digest(before)+".json"))==before,"original journal not archived");
    require(!compact_recording_journal(journal),"compaction not idempotent");
    fs::remove_all(root);std::cout<<"Transport fixture passed\n";return 0;
  }catch(const std::exception &e){fs::remove_all(root);std::cerr<<e.what()<<'\n';return 1;}
}
