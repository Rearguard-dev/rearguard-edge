#include "physical.hpp"
#include "runtime.hpp"
#include "worker.hpp"
#include "jpeg.hpp"
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <set>

namespace harness::tiny {
namespace {
using observation::require;
template<class T> T decode_physical(const observation::ObservationMessage &packet) {
  require(packet.cdr && packet.cdr->size()<=64*1024*1024,"invalid physical observation packet");
  rclcpp::SerializedMessage bytes(packet.cdr->size());auto &target=bytes.get_rcl_serialized_message();
  std::memcpy(target.buffer,packet.cdr->data(),packet.cdr->size());target.buffer_length=packet.cdr->size();
  T result;rclcpp::Serialization<T> codec;codec.deserialize_message(&bytes,&result);return result;
}
Ns physical_stamp(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<Ns>(stamp.sec)*1000000000LL+static_cast<Ns>(stamp.nanosec);
}
class PhysicalAdapter final:public observation::CriticAdapter {
  PhysicalModel model_;
  Json binding_,selection_,latest_={{"status","warming_up"}};
  std::map<std::string,Json> topics_;
  std::map<std::string,Ns> processed_;
  Ns epoch_=-1,submitted_=-1,processed_epoch_=-1,last_candidate_=-1,last_result_=-1,origin_=-1;
  bool positive_=false;
  std::uint64_t inference_submissions_=0;
  Worker worker_; // Join before destroying models and packet state.
public:
  PhysicalAdapter(const fs::path &path,Json binding,Json selection):model_(path,binding,selection),binding_(std::move(binding)),selection_(std::move(selection)) {
    require(binding_.value("robot","")==selection_.at("runtime_context").at("embodiment").at("class_id").get<std::string>(),"physical model embodiment differs from live binding");
    for(const auto &topic:binding_.at("topics"))if(topic.value("role","")=="joint_state" || topic.value("role","")=="action")topics_[topic.at("name")]=topic;
    std::size_t states=0,commands=0;
    for(const auto &[name,topic]:topics_){(void)name;if(topic.at("role")=="joint_state"){require(topic.at("type")=="sensor_msgs/msg/JointState","unsupported physical state schema");++states;}
      else{require(topic.at("type")=="std_msgs/msg/Float64MultiArray","unsupported physical command schema");++commands;}}
    require(states==1 && commands<=1,"physical model requires one state and at most one command stream");
    for(const auto &name:model_.cameras()) {
      bool found=false;for(const auto &topic:binding_.at("topics"))if(topic.at("name")==name && topic.at("role")=="camera") {
        require(topic.at("type")=="sensor_msgs/msg/CompressedImage" || topic.at("type")=="sensor_msgs/msg/Image","unsupported physical camera schema");
        require(!name.ends_with("/zstd"),"physical model requires decoded or JPEG camera transport");topics_[name]=topic;found=true;
      }require(found,"trained physical camera missing in current binding");
    }
  }
  bool ready() const override{return true;}
  bool load(const fs::path &,const Json &,const Json &) override{return false;}
  Json telemetry() const override{return latest_;}
  std::optional<observation::Detection> evaluate(const observation::Observations &observations) override {
    const auto now=observations.receipt_ros_ns;
    if(epoch_!=observations.epoch || now<submitted_){worker_.reset();epoch_=observations.epoch;submitted_=-1;origin_=now;last_result_=-1;last_candidate_=-1;positive_=false;}
    const auto hz=model_.manifest().at("spec").at("decision_hz").get<double>();
    const auto tick=std::floor(static_cast<double>(now-origin_)*hz/1e9);
    const auto anchor=origin_+static_cast<Ns>(std::llround(tick*1e9/hz));
    const auto max_age=static_cast<Ns>(model_.manifest().at("preprocessing").at("max_age_s").get<double>()*1e9);
    const bool recent=observation::has_recent_input(observations,max_age);
    if(recent && (submitted_<0 || anchor>submitted_)) {
      std::vector<observation::ObservationMessage> packets;std::map<std::string,observation::ObservationMessage> cameras;
      for(const auto &packet:observations.messages)if(topics_.contains(packet.topic) && packet.receipt_ros_ns<=anchor) {
        if(topics_.at(packet.topic).at("role")=="camera")cameras[packet.topic]=packet;
        else packets.push_back(packet);
      }
      for(auto &[name,packet]:cameras){(void)name;packets.push_back(std::move(packet));}
      std::stable_sort(packets.begin(),packets.end(),[](const auto &a,const auto &b){return a.receipt_ros_ns<b.receipt_ros_ns;});
      const auto epoch=observations.epoch;const auto queued=observation::monotonic_ns();
      worker_.submit([this,packets=std::move(packets),now=anchor,epoch,queued] {
        const auto started=observation::monotonic_ns();
        if(processed_epoch_!=epoch){model_.reset(epoch);processed_.clear();processed_epoch_=epoch;}
        for(const auto &packet:packets) {
          if(processed_.contains(packet.topic) && packet.receipt_ros_ns<=processed_.at(packet.topic))continue;
          processed_[packet.topic]=packet.receipt_ros_ns;const auto &topic=topics_.at(packet.topic);const auto role=topic.at("role").get<std::string>();
          if(role=="joint_state") {
            const auto state=decode_physical<sensor_msgs::msg::JointState>(packet);const auto source=physical_stamp(state.header.stamp);
            if(source<0 || source>packet.receipt_ros_ns)continue;
            require(state.name.size()==model_.joints().size(),"physical joint count differs from binding");
            std::map<std::string,std::size_t> indices;for(std::size_t i=0;i<state.name.size();++i)require(indices.emplace(state.name[i],i).second,"duplicate physical joint name");
            std::vector<float> values(model_.joints().size()*3),known(values.size());const std::vector<double> *channels[]={&state.position,&state.velocity,&state.effort};
            for(std::size_t col=0;col<3;++col){require(channels[col]->empty() || channels[col]->size()==state.name.size(),"physical state channel length differs");
              for(std::size_t i=0;i<model_.joints().size();++i){const auto index=indices.at(model_.joints()[i]);if(!channels[col]->empty() && std::isfinite(channels[col]->at(index))){values[i*3+col]=static_cast<float>(channels[col]->at(index));known[i*3+col]=1;}}}
            model_.state(source,packet.receipt_ros_ns,std::move(values),std::move(known));
          } else if(role=="action") {
            const auto command=decode_physical<std_msgs::msg::Float64MultiArray>(packet);model_.command(packet.receipt_ros_ns,command.data);
          } else {
            const auto index=static_cast<std::size_t>(std::find(model_.cameras().begin(),model_.cameras().end(),packet.topic)-model_.cameras().begin());
            Ns source;RgbImage rgb;
            if(packet.type=="sensor_msgs/msg/CompressedImage") {
              const auto image=decode_physical<sensor_msgs::msg::CompressedImage>(packet);source=physical_stamp(image.header.stamp);
              if(source<0 || source>packet.receipt_ros_ns || now-source>250000000LL)continue;
              rgb=decode_jpeg(image.data);
            } else {
              const auto image=decode_physical<sensor_msgs::msg::Image>(packet);source=physical_stamp(image.header.stamp);
              if(source<0 || source>packet.receipt_ros_ns || now-source>250000000LL)continue;
              const std::size_t w=image.width,h=image.height;const std::size_t channels=image.encoding=="mono8"?1:
                (image.encoding=="rgb8" || image.encoding=="bgr8")?3:(image.encoding=="rgba8" || image.encoding=="bgra8")?4:0;
              require(w>0 && h>0 && w*h<=16777216 && channels && image.step>=w*channels && image.data.size()==h*image.step,"invalid physical RGB transport");
              rgb={w,h,std::vector<std::uint8_t>(w*h*3)};const bool bgr=image.encoding=="bgr8" || image.encoding=="bgra8";
              for(std::size_t y=0;y<h;++y)for(std::size_t x=0;x<w;++x)for(std::size_t c=0;c<3;++c)
                rgb.pixels[(y*w+x)*3+c]=image.data[y*image.step+x*channels+(channels==1?0:bgr?2-c:c)];
            }
            model_.camera(index,source,packet.receipt_ros_ns,rgb.pixels,rgb.width,rgb.height);
          }
        }
        auto result=model_.score(now,epoch);const auto ended=observation::monotonic_ns();
        result.update({{"queue_delay_us",static_cast<double>(started-queued)/1000.},
          {"evaluation_us",static_cast<double>(ended-started)/1000.},{"binding_hash",selection_.at("binding_hash")}});return result;
      });submitted_=anchor;++inference_submissions_;
    }
    latest_=worker_.latest(now,observations.epoch,max_age);
    if (!recent) {
      latest_["status"]="insufficient_evidence";latest_["scores"]=Json::object();
      latest_["reason"]="no recent observation packets";
    }
    latest_.update({{"inference_submissions",inference_submissions_},{"has_recent_input",recent},{"model_id",selection_.at("model_id")},{"profile_id",selection_.at("profile_id")},
      {"binding_hash",selection_.at("binding_hash")},{"source","experimental_critic"},{"mode","shadow"},{"calibrated",false},{"enforcement_authorized",false}});
    if(latest_.value("status","")!="valid"){positive_=false;return std::nullopt;}
    const Ns result_time=latest_.at("t_ns");if(result_time==last_result_)return std::nullopt;last_result_=result_time;
    for(const auto &[id,score]:latest_.at("scores").items()) {
      const bool positive=score.value("valid",false) && !score.at("score").is_null() && score.at("score").get<double>()>=.5;
      const bool rising=positive && !positive_;positive_=positive;
      if(rising && (last_candidate_<0 || result_time-last_candidate_>=10000000000LL)) {
        last_candidate_=result_time;const auto model=selection_.at("model_id").get<std::string>();
        observation::Detection detection{model,id,score.at("score"),10,5,"experimental_critic",model,false};detection.proposition_id=id;return detection;
      }
    }
    return std::nullopt;
  }
};
}
}
namespace harness::observation {
std::unique_ptr<CriticAdapter> make_physical_critic(const fs::path &path,const Json &binding,const Json &selection) {
  return std::make_unique<harness::tiny::PhysicalAdapter>(path,binding,selection);
}
}
