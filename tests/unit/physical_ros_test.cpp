#include "runtime.hpp"
#include "physical.hpp"
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <turbojpeg.h>
#include <chrono>
#include <iostream>
#include <thread>

using namespace harness::observation;
template<class T> std::shared_ptr<const std::vector<std::uint8_t>> serialize_test(const T &message) {
  rclcpp::SerializedMessage bytes;rclcpp::Serialization<T> codec;codec.serialize_message(&message,&bytes);
  const auto &data=bytes.get_rcl_serialized_message();return std::make_shared<const std::vector<std::uint8_t>>(data.buffer,data.buffer+data.buffer_length);
}
int main(int argc,char **argv) {
  if(argc!=3){std::cout<<"Supply a verified bundle and input fixture for the ROS model probe\n";return 77;}
  try {
    const auto input=read_json(argv[2]);auto binding=input.at("binding"),selection=input.at("selection");
    const auto manifest=read_json(fs::path(argv[1])/"manifest.json");const auto cameras=manifest.at("preprocessing").at("camera_topics").get<std::vector<std::string>>();
    binding["robot"]=selection.at("runtime_context").at("embodiment").at("class_id");
    binding["topics"]=Json::array({{{"name","/joint_states"},{"role","joint_state"},{"type","sensor_msgs/msg/JointState"}},
      {{"name","/arm_controller/commands"},{"role","action"},{"type","std_msgs/msg/Float64MultiArray"}}});
    for(const auto &name:cameras)binding["topics"].push_back({{"name",name},{"role","camera"},{"type","sensor_msgs/msg/CompressedImage"}});
    selection["profile_id"]="isolated-native-probe";selection["binding_hash"]=digest(binding);
    auto adapter=make_physical_critic(argv[1],binding,selection);require(adapter->ready(),"physical model did not load");
    require(!adapter->load({},Json::object(),Json::object()),"shadow model unexpectedly accepts recovery authority");
    const int width=320,height=240;std::vector<unsigned char> rgb(static_cast<std::size_t>(width*height*3),127);
    tjhandle compressor=tjInitCompress();require(compressor!=nullptr,"JPEG probe encoder unavailable");
    unsigned char *jpeg=nullptr;unsigned long size=0;
    const int code=tjCompress2(compressor,rgb.data(),width,width*3,height,TJPF_RGB,&jpeg,&size,TJSAMP_420,85,0);
    tjDestroy(compressor);require(code==0 && jpeg,"could not encode probe image");
    std::vector<std::uint8_t> camera_bytes(jpeg,jpeg+size);tjFree(jpeg);
    Observations observations;observations.config=binding;observations.epoch=0;
    Json telemetry;bool scored=false;const auto started=std::chrono::steady_clock::now();
    for (int phase=0;phase<2;++phase) {
      if (phase==1) {
        const auto submitted=telemetry.at("inference_submissions");
        const auto old_time=observations.receipt_ros_ns;
        const auto max_age=static_cast<Ns>(manifest.at("preprocessing").at("max_age_s").get<double>()*1e9);
        for (int idle=0;idle<30;++idle) {
          observations.receipt_ros_ns=old_time+max_age+100000000LL+static_cast<Ns>(idle)*33333333LL;
          require(!adapter->evaluate(observations),"stale inputs produced a detection");
          const auto stopped=adapter->telemetry();
          require(stopped.at("inference_submissions")==submitted && stopped.at("scores").empty() &&
                  stopped.at("status")=="insufficient_evidence","idle input queued work or exposed old scores");
        }
        observations.messages.clear();++observations.epoch;scored=false;
      }
    for(int attempt=0;attempt<120;++attempt) {
      const Ns now=wall_ns(),source=now-5000000LL;
      builtin_interfaces::msg::Time stamp;stamp.sec=static_cast<std::int32_t>(source/1000000000LL);stamp.nanosec=static_cast<std::uint32_t>(source%1000000000LL);
      sensor_msgs::msg::JointState state;state.header.stamp=stamp;state.name=binding.at("joint_order").get<std::vector<std::string>>();
      state.position.assign(state.name.size(),0);state.velocity.assign(state.name.size(),0);
      observations.messages.push_back({"/joint_states","sensor_msgs/msg/JointState",serialize_test(state),now});
      if(attempt==0){std_msgs::msg::Float64MultiArray command;command.data.assign(state.name.size(),0);
        observations.messages.push_back({"/arm_controller/commands","std_msgs/msg/Float64MultiArray",serialize_test(command),now});}
      for(const auto &name:cameras){sensor_msgs::msg::CompressedImage image;image.header.stamp=stamp;image.format="jpeg";image.data=camera_bytes;
        observations.messages.push_back({name,"sensor_msgs/msg/CompressedImage",serialize_test(image),now});}
      observations.receipt_ros_ns=now;
      const auto detection=adapter->evaluate(observations);
      if(detection)require(detection->source=="experimental_critic" && !detection->deployment_validated,"uncalibrated inference claimed validated authority");
      telemetry=adapter->telemetry();
      if(telemetry.value("status","")=="valid") {
        require(telemetry.at("scores").size()==1 && telemetry.at("scores").contains("human.intrusion"),"unexpected proposition scored");
        require(!telemetry.value("enforcement_authorized",true),"probe authorized control");scored=true;break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    require(scored,"no fresh ROS-decoded model score: "+telemetry.dump());
    }
    std::cout<<Json{{"status","passed"},{"model_id",selection.at("model_id")},{"mode","shadow"},
      {"wall_seconds",std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()},
      {"telemetry",telemetry},{"idle_guard","stale inputs queued zero jobs; fresh epoch resumed scoring"},{"scope","in-process ROS serialization test; no ROS publications or robot commands"}}.dump(2)<<'\n';
    return 0;
  } catch(const std::exception &error){std::cerr<<error.what()<<'\n';return 1;}
}
