// Offline MCAP integration only: no ROS node, DDS, policy, network or robot.
#include "recording.hpp"
#include "evidence.hpp"
#include "recording_upload.hpp"
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <iostream>
#include <cstdlib>

using namespace harness::observation;
namespace {
template<class T> std::shared_ptr<const rclcpp::SerializedMessage> encode(const T &message) {
  auto out=std::make_shared<rclcpp::SerializedMessage>();rclcpp::Serialization<T> codec;codec.serialize_message(&message,out.get());return out;
}
}
int main() {
  const auto base=fs::temp_directory_path()/("sampling-mcap-test-"+unique_id());
  const auto root=base/"sessions"/"synthetic";
  ::setenv("REARGUARD_STATE_DIR",(base/"test-events").c_str(),1);
  try {
    fs::create_directories(root/"rosbag2");
    const Json topics=Json::array({{{"name","/joint"},{"type","sensor_msgs/msg/JointState"},{"role","joint_state"}},
      {{"name","/camera"},{"type","sensor_msgs/msg/CompressedImage"},{"role","camera"}},
      {{"name","/wrist"},{"type","sensor_msgs/msg/CompressedImage"},{"role","camera"}},
      {{"name","/commands"},{"type","std_msgs/msg/Float64MultiArray"},{"role","action"}}});
    const Json binding={{"topics",topics},{"joint_order",{"j"}},{"clock","ros_sim"}};
    atomic_json(root/"session.json",{{"config",binding}});
    Recording recording(root,topics);Evidence evidence(binding,"offline-test");
    for (Ns i=0;i<=150;++i) {
      const Ns time=1000000000LL+i*100000000LL;evidence.tick(time,i*100000000LL);
      sensor_msgs::msg::JointState joint;joint.name={"j"};joint.position={static_cast<double>(i)*.001};
      joint.header.stamp.sec=static_cast<int>(time/1000000000LL);joint.header.stamp.nanosec=static_cast<unsigned>(time%1000000000LL);
      recording.write("/joint","sensor_msgs/msg/JointState",encode(joint),time,0);
      evidence.sample("/joint",time,time,i*100000000LL);
      sensor_msgs::msg::CompressedImage camera;camera.format="jpeg";camera.data={1,2,3,4};
      const auto source=time-800000000LL;camera.header.stamp.sec=static_cast<int>(source/1000000000LL);
      camera.header.stamp.nanosec=static_cast<unsigned>(source%1000000000LL);
      for (const auto *name:{"/camera","/wrist"}) {
        recording.write(name,"sensor_msgs/msg/CompressedImage",encode(camera),time,0);
        evidence.sample(name,time,source,i*100000000LL);
      }
    }
    recording.close();
    auto window=evidence.capture(15.,0.,"unlabeled");window["execution_sample"]={{"max_bytes",64*1024*1024}};
    export_window(root,window,root/"export");
    rosbag2_storage::StorageOptions options;options.uri=(root/"export").string();options.storage_id="mcap";
    rosbag2_cpp::Reader reader;reader.open(options,{"cdr","cdr"});unsigned frames=0,joints=0;
    while(reader.has_next()) {
      const auto message=reader.read_next();require(message->recv_timestamp>=1000000000LL && message->recv_timestamp<=16000000000LL,"wrong exported window");
      if (message->topic_name=="/joint") ++joints;
      else {
        rclcpp::SerializedMessage bytes(*message->serialized_data);sensor_msgs::msg::CompressedImage camera;
        rclcpp::Serialization<sensor_msgs::msg::CompressedImage> codec;codec.deserialize_message(&bytes,&camera);
        const Ns source=static_cast<Ns>(camera.header.stamp.sec)*1000000000LL+camera.header.stamp.nanosec;
        require(message->recv_timestamp-source==800000000LL,"acquisition stamp changed");++frames;
      }
    }
    require(frames==302 && joints==151,"not all synchronized messages exported");
    bool rejected=false;window["execution_sample"]["max_bytes"]=10;
    try {export_window(root,window,root/"too-large");}catch(const std::exception &){rejected=true;}
    require(rejected,"byte limit bypassed");
    rejected=false;window["execution_sample"]["max_bytes"]=64*1024*1024;window["coverage"]["/wrist"]["max_receipt_gap_ns"]=2000000000LL;
    try {export_window(root,window,root/"bad-coverage");}catch(const std::exception &){rejected=true;}
    require(rejected,"coverage limit bypassed");
    auto store=Store::create(base/"store",HARNESS_OBSERVATION_CONFIG);
    atomic_json(root/"session.json",{{"config",binding},{"workflow_store",store.root.string()},{"session_id","synthetic"}});
    // Unconfirmed alerts annotate a durable unlabeled sample instead of deleting it.
    window["coverage"]["/wrist"]["max_receipt_gap_ns"]=100000000LL;
    window["execution_sample"]["id"]="sample-alert";
    auto alert=window;alert.erase("execution_sample");alert["id"]="alert";
    alert["detection"]={{"source","experimental_critic"},{"deployment_validated",false}};
    atomic_json(root/"state.json",{{"windows",Json::array({window,alert})}});
    spool_execution_samples(store.root,root);
    const auto directory=store.root/"execution-samples";
    require(fs::is_regular_file(directory/"sample-alert.mcap"),"alert overlap lost durable MCAP");
    const auto manifest=read_json(directory/"sample-alert.json");
    require(manifest["label_status"]=="unlabeled" && manifest["training_eligible"]==false &&
            manifest["unconfirmed_alert_overlaps"].size()==1,"overlap provenance or admission gate lost");
    window["execution_sample"]["id"]="sample-async";
    atomic_json(root/"state.json",{{"windows",Json::array({window,alert})}});
    {
      ExecutionSampleSpooler spooler(store.root,root.parent_path());
      // The caller does no spool work: it may be blocked on an unrelated upload.
      for (int i=0;i<100 && !fs::exists(directory/"sample-async.json");++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      require(fs::is_regular_file(directory/"sample-async.mcap") &&
              fs::is_regular_file(directory/"sample-async.json"),"independent local spooler stalled");
    }
    window["execution_sample"]["id"]="sample-operator";alert.erase("detection");alert["trigger"]="incident_click";
    atomic_json(root/"state.json",{{"windows",Json::array({window,alert})}});
    spool_execution_samples(store.root,root);
    require(fs::exists(directory/"sample-operator.rejected.json") && !fs::exists(directory/"sample-operator.mcap"),
            "operator incident entered ordinary-operation pool");
    fs::remove_all(base);std::cout<<"Offline MCAP: synchronized export, preserved source stamps, sparse commands, coverage and byte limits passed\n";return 0;
  }catch(const std::exception &e){fs::remove_all(base);std::cerr<<e.what()<<'\n';return 1;}
}
