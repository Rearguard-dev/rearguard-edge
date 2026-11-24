#include "physical.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <set>
#include <tuple>

namespace harness::tiny {
using observation::require;
namespace {
Tensor tensor(const Json &description) {
  require(description.at("type")=="tensor(float)","physical graph requires float32 IO");
  return Tensor(description.at("name"),description.at("shape").get<std::vector<std::int64_t>>());
}
void copy(std::vector<float> &to,const std::vector<float> &from) {
  require(to.size()==from.size(),"physical tensor length differs");std::copy(from.begin(),from.end(),to.begin());
}
double cubic(double x) {
  x=std::abs(x);if(x<1)return ((1.5*x-2.5)*x)*x+1;
  if(x<2)return ((-.5*x+2.5)*x-4)*x+2;
  return 0;
}
struct Filter {int first;std::vector<std::int32_t> weights;};
std::vector<Filter> filters(std::size_t original,std::size_t resized) {
  const double scale=static_cast<double>(original)/static_cast<double>(resized),filter=std::max(1.,scale);
  std::vector<Filter> result;
  for(std::size_t n=0;n<resized;++n) {
    const auto center=(static_cast<double>(n)+.5)*scale;
    const int first=std::max(0,static_cast<int>(center-2*filter+.5));
    const int end=std::min(static_cast<int>(original),static_cast<int>(center+2*filter+.5));
    std::vector<double> values;double sum=0;
    for(int k=first;k<end;++k) {const auto weight=cubic((k-center+.5)/filter);values.push_back(weight);sum+=weight;}
    Filter f{first,{}};for(const auto weight:values)f.weights.push_back(static_cast<std::int32_t>(std::lround(weight/sum*(1<<22))));
    result.push_back(std::move(f));
  }return result;
}
float seconds(Ns value) {return static_cast<float>(static_cast<double>(value)/1e9);}
}
PhysicalGraph::PhysicalGraph(const fs::path &root,const Json &description):output_(tensor(description.at("outputs").at(0))) {
  require(description.at("outputs").size()==1,"one physical graph output required");
  const auto file=description.at("file").get<std::string>();
  require(file.find('/')==std::string::npos && file.find('\\')==std::string::npos &&
    observation::file_hash(root/file)==description.at("sha256").get<std::string>(),"physical graph identity mismatch");
  for(const auto &input:description.at("inputs")) inputs_.push_back(tensor(input));
  engine_=make_onnx(root/file,inputs_,output_);
}
Json PhysicalModel::debug_inputs() const {
  Json result=Json::object();for(const auto &[name,graph]:graphs_)for(const auto &input:graph->inputs())result[name][input.name]=input.values;
  return result;
}
std::vector<float> &PhysicalGraph::input(const std::string &name) {
  for(auto &input:inputs_)if(input.name==name)return input.values;
  throw std::runtime_error("unknown physical graph input: "+name);
}
const Tensor &PhysicalGraph::run() {
  engine_->run(inputs_,output_);
  require(std::all_of(output_.values.begin(),output_.values.end(),[](float v){return std::isfinite(v);}),"nonfinite physical graph output");
  return output_;
}
void physical_pixels(std::span<const std::uint8_t> rgb,std::size_t w,std::size_t h,std::span<float> output) {
  require(w>0 && h>0 && w<=16384 && h<=16384 && w*h<=16777216 && rgb.size()==w*h*3 &&
    output.size()==3*224*224 && std::max(w,h)<=16*std::min(w,h),"unsupported physical RGB dimensions");
  const std::size_t rw=w<=h?235:235*w/h,rh=w<=h?235*h/w:235;
  const auto fx=filters(w,rw),fy=filters(h,rh);
  std::vector<std::uint8_t> horizontal(rw*h*3),resized(rw*rh*3);
  for(std::size_t y=0;y<h;++y)for(std::size_t x=0;x<rw;++x)for(std::size_t channel=0;channel<3;++channel) {
    std::int64_t sum=1<<21;const auto &f=fx[x];
    for(std::size_t k=0;k<f.weights.size();++k)sum+=static_cast<std::int64_t>(rgb[(y*w+static_cast<std::size_t>(f.first)+k)*3+channel])*f.weights[k];
    horizontal[(y*rw+x)*3+channel]=static_cast<std::uint8_t>(std::clamp<std::int64_t>(sum>>22,0,255));
  }
  for(std::size_t y=0;y<rh;++y)for(std::size_t x=0;x<rw;++x)for(std::size_t channel=0;channel<3;++channel) {
    std::int64_t sum=1<<21;const auto &f=fy[y];
    for(std::size_t k=0;k<f.weights.size();++k)sum+=static_cast<std::int64_t>(horizontal[((static_cast<std::size_t>(f.first)+k)*rw+x)*3+channel])*f.weights[k];
    resized[(y*rw+x)*3+channel]=static_cast<std::uint8_t>(std::clamp<std::int64_t>(sum>>22,0,255));
  }
  const auto left=static_cast<std::size_t>(std::nearbyint(static_cast<double>(rw-224)/2)),top=static_cast<std::size_t>(std::nearbyint(static_cast<double>(rh-224)/2));
  constexpr float mean[3]={.485F,.456F,.406F},deviation[3]={.229F,.224F,.225F};
  for(std::size_t channel=0;channel<3;++channel)for(std::size_t y=0;y<224;++y)for(std::size_t x=0;x<224;++x)
    output[channel*224*224+y*224+x]=(static_cast<float>(resized[((y+top)*rw+x+left)*3+channel])/255-mean[channel])/deviation[channel];
}

PhysicalModel::PhysicalModel(const fs::path &root,Json binding,Json selection):
  manifest_(observation::read_json(root/"manifest.json")),binding_(std::move(binding)),selection_(std::move(selection)),context_(selection_.at("runtime_context")) {
  require(manifest_.value("architecture","")=="physical_efficientvit_b1_tspulse_r1_evidence128_v2" &&
    manifest_.value("trained",false) && manifest_.value("tensor_contract_version",0)==2,"unsupported physical model");
  require(selection_.value("mode","")=="shadow" && !selection_.value("enforcement_authorized",true),"physical inference requires shadow selection");
  if(selection_.contains("files"))for(const auto &file:selection_.at("files")) {
    const auto name=file.at("name").get<std::string>();require(name.find('/')==std::string::npos && name.find('\\')==std::string::npos,"invalid cached model path");
    require(fs::file_size(root/name)==file.at("size").get<std::uintmax_t>() && observation::file_hash(root/name)==file.at("sha256").get<std::string>(),"cached model file changed before observer load");
  }
  if(selection_.contains("binding_hash"))require(selection_.at("binding_hash")==observation::digest(binding_),"physical model input binding changed");
  joints_=binding_.at("joint_order").get<std::vector<std::string>>();
  require(std::set<std::string>(joints_.begin(),joints_.end()).size()==joints_.size(),"duplicate physical joints");
  require(context_.at("embodiment").at("joint_names")==binding_.at("joint_order"),"physical joint context differs from binding");
  cameras_=manifest_.at("preprocessing").at("camera_topics").get<std::vector<std::string>>();
  j_=manifest_.at("spec").at("joints");c_=manifest_.at("spec").at("cameras");e_=manifest_.at("spec").at("events");t_=manifest_.at("spec").at("steps");
  require(j_==joints_.size() && c_==cameras_.size() && j_<=128 && c_<=4 && e_<=512 && t_<=64,"physical graph capacity differs from binding");
  require(binding_.value("command_interface","")=="position","initial physical adapter supports position commands only");
  require(manifest_.at("backbones").at("preprocess").at("interpolation")=="bicubic" &&
    std::abs(manifest_.at("backbones").at("preprocess").at("crop_pct").get<double>()-.95)<1e-8,"unsupported physical pixel transform");
  require(manifest_.at("backbones").at("preprocess").at("mean")==Json::array({.485,.456,.406}) &&
    manifest_.at("backbones").at("preprocess").at("std")==Json::array({.229,.224,.225}),"physical image normalization differs");
  for(const auto *name:{"vision","sensor","embodiment","fusion","decision"})graphs_[name]=std::make_unique<PhysicalGraph>(root,manifest_.at("graphs").at(name));
  tokens_=c_*16+j_*3+manifest_.at("spec").at("latents").get<std::size_t>();
  require(graphs_.at("decision")->input("fused").size()==t_*tokens_*128,"physical evidence memory shape differs");
  camera_.resize(c_);previous_event_.assign(j_,-1);embodiment();
  auto &ids=graphs_.at("decision")->input("proposition_ids"),&mask=graphs_.at("decision")->input("proposition_mask");
  require(ids.size()==manifest_.at("propositions").size(),"physical proposition capacity differs");
  for(std::size_t i=0;i<ids.size();++i){ids[i]=static_cast<float>(i);mask[i]=0;}
  for(const auto &selected:selection_.at("active_proposition_ids")) {
    bool found=false;for(std::size_t i=0;i<ids.size();++i)if(manifest_.at("propositions")[i].at("id")==selected){mask[i]=1;found=true;}
    require(found,"selected physical proposition is not trained");
  }
  reset(0);
}
void PhysicalModel::embodiment() {
  auto &graph=*graphs_.at("embodiment");auto &numeric=graph.input("joint_numeric"),&known=graph.input("joint_numeric_mask");
  const std::map<std::string,int> roles{{"shoulder_pan",1},{"shoulder_lift",1},{"elbow_flex",2},{"wrist_flex",3},{"wrist_roll",3},{"gripper",4}};
  const std::map<std::string,int> types{{"revolute",1},{"continuous",2},{"prismatic",3}};
  bool topology=true;
  for(const auto &name:joints_){const auto &m=context_.at("joints").at(name);topology=topology && (!m.value("parent_joint",Json(nullptr)).is_null() || m.value("root_known",false));}
  for(std::size_t i=0;i<j_;++i) {
    const auto &name=joints_[i];const auto &m=context_.at("joints").at(name);const std::string type=m.at("type");
    require(types.contains(type) && m.value("position_unit",type=="prismatic"?"m":"rad")==std::string(type=="prismatic"?"m":"rad"),"unsupported physical units or joint type");
    graph.input("joint_type")[i]=static_cast<float>(types.at(type));graph.input("joint_role")[i]=static_cast<float>(roles.contains(name)?roles.at(name):0);
    graph.input("joint_present")[i]=1;graphs_.at("fusion")->input("joint_present")[i]=1;
    auto value=[&](std::size_t column,double number){require(std::isfinite(number),"nonfinite physical metadata");numeric[i*12+column]=static_cast<float>(number);known[i*12+column]=1;};
    const char *fields[]={"position_lower","position_upper","velocity_limit","effort_limit"};
    for(std::size_t k=0;k<4;++k)if(m.contains(fields[k]) && !m.at(fields[k]).is_null())value(k,m.at(fields[k]));
    if(m.contains("axis") && !m.at("axis").is_null())for(std::size_t k=0;k<3;++k)value(k+4,m.at("axis").at(k));
    if(topology){std::size_t depth=0;auto parent=m.value("parent_joint",Json(nullptr));std::set<std::string> seen{name};
      while(!parent.is_null()){const auto p=parent.get<std::string>();require(seen.insert(p).second,"cyclic physical joint topology");++depth;parent=context_.at("joints").at(p).value("parent_joint",Json(nullptr));}
      value(7,static_cast<double>(depth));std::size_t children=0;
      for(const auto &other:joints_)if(context_.at("joints").at(other).value("parent_joint",Json(nullptr))==name)++children;
      value(8,static_cast<double>(children));}
    value(9,1);value(10,0);value(11,0);
  }
  copy(graphs_.at("fusion")->input("embodiment"),graph.run().values);
}
void PhysicalModel::reset(Ns epoch) {states_.clear();events_.clear();history_.clear();for(auto &c:camera_)c=Camera{};std::fill(previous_event_.begin(),previous_event_.end(),-1);epoch_=epoch;last_=-1;event_id_=0;}
void PhysicalModel::state(Ns source,Ns available,std::vector<float> values,std::vector<float> known) {
  require(source>=0 && source<=available && values.size()==j_*3 && known.size()==values.size(),"invalid physical state clock or shape");
  for(std::size_t i=0;i<values.size();++i)if(!std::isfinite(values[i])){values[i]=0;known[i]=0;}
  states_.push_back({source,available,std::move(values),std::move(known)});while(states_.size()>8192)states_.pop_front();
}
void PhysicalModel::command(Ns available,const std::vector<double> &values) {
  require(available>=0 && values.size()==j_,"invalid physical command");
  require(std::all_of(values.begin(),values.end(),[](double v){return std::isfinite(v);}),"nonfinite physical command");
  const auto id=std::to_string(event_id_++)+":"+std::to_string(available);
  for(std::size_t i=0;i<j_;++i){events_.push_back({available,i,static_cast<float>(values[i]),previous_event_[i]<0?0:seconds(available-previous_event_[i]),id});previous_event_[i]=available;}
  // Retain the last issuing event for every joint plus enough recent history.
  while(events_.size()>32768)events_.pop_front();
}
void PhysicalModel::camera(std::size_t index,Ns source,Ns available,std::span<const std::uint8_t> rgb,std::size_t w,std::size_t h) {
  require(index<c_ && source>=0 && source<=available,"invalid physical camera clock");auto &camera=camera_[index];
  if(source<camera.source || (source==camera.source && available==camera.available))return;
  auto &vision=*graphs_.at("vision");physical_pixels(rgb,w,h,vision.input("pixels"));camera.features=vision.run().values;camera.source=source;camera.available=available;
}
Json PhysicalModel::score(Ns anchor,Ns epoch) {
  require(anchor>=0 && epoch==epoch_,"physical epoch must reset before receiving packets");
  require(last_<0 || anchor>last_,"physical anchors must increase");
  auto &fusion=*graphs_.at("fusion");for(const auto &input:fusion.inputs())if(input.name!="embodiment" && input.name!="joint_present")std::fill(fusion.input(input.name).begin(),fusion.input(input.name).end(),0);
  const auto max_age=static_cast<Ns>(manifest_.at("preprocessing").at("max_age_s").get<double>()*1e9);
  auto &vision=fusion.input("vision");auto &cm=fusion.input("camera_mask"),&ages=fusion.input("vision_age_s");
  for(std::size_t i=0;i<c_;++i){const auto &camera=camera_[i];if(camera.source<0 || camera.source>anchor || camera.available>anchor)continue;
    ages[i]=seconds(anchor-camera.source);if(anchor-camera.source<=max_age && anchor-camera.available<=max_age){cm[i]=1;std::copy(camera.features.begin(),camera.features.end(),vision.begin()+static_cast<std::ptrdiff_t>(i*16*128));}}
  auto &sensor=*graphs_.at("sensor");auto &past=sensor.input("past_values"),&observed=sensor.input("observed");std::fill(past.begin(),past.end(),0);std::fill(observed.begin(),observed.end(),0);
  const auto period=static_cast<Ns>(std::llround(1e9/manifest_.at("preprocessing").at("sensor_hz").get<double>()));
  for(std::size_t k=0;k<512;++k){const auto stamp=anchor-static_cast<Ns>(511-k)*period;
    for(auto it=states_.rbegin();it!=states_.rend();++it)if(it->source<=stamp && it->available<=stamp && stamp-it->source<=std::min<Ns>(max_age,50000000)){
      std::copy(it->values.begin(),it->values.end(),past.begin()+static_cast<std::ptrdiff_t>(k*j_*3));std::copy(it->known.begin(),it->known.end(),observed.begin()+static_cast<std::ptrdiff_t>(k*j_*3));break;}}
  bool full=false;for(std::size_t channel=0;channel<j_*3;++channel){bool complete=true;for(std::size_t k=0;k<512;++k)complete=complete && observed[k*j_*3+channel]!=0;full=full||complete;}
  if(full){const auto &temporal=sensor.run().values;for(std::size_t i=0;i<j_;++i){std::copy_n(temporal.begin()+static_cast<std::ptrdiff_t>(i*129),128,fusion.input("temporal").begin()+static_cast<std::ptrdiff_t>(i*128));fusion.input("temporal_mask")[i]=temporal[i*129+128];}}
  std::vector<const Event *> eligible;std::map<std::size_t,const Event *> active;
  for(const auto &event:events_)if(event.stamp<=anchor){eligible.push_back(&event);active[event.joint]=&event;}
  auto before=[&](const Event *a,const Event *b){return std::tie(a->stamp,a->id,joints_[a->joint])<std::tie(b->stamp,b->id,joints_[b->joint]);};
  std::sort(eligible.begin(),eligible.end(),before);std::vector<const Event *> chosen;
  for(const auto *event:eligible)if(active[event->joint]==event)chosen.push_back(event);
  if(chosen.size()>e_)chosen.erase(chosen.begin(),chosen.end()-static_cast<std::ptrdiff_t>(e_));
  std::set<const Event *> picked(chosen.begin(),chosen.end());std::vector<const Event *> rest;
  for(const auto *event:eligible)if(!picked.contains(event))rest.push_back(event);
  const auto spare=e_-chosen.size();const auto count=std::min(spare,rest.size());chosen.insert(chosen.end(),rest.end()-static_cast<std::ptrdiff_t>(count),rest.end());std::sort(chosen.begin(),chosen.end(),before);
  for(std::size_t i=0;i<chosen.size();++i){const auto &event=*chosen[i];auto &numeric=fusion.input("event_numeric");const auto age=seconds(anchor-event.stamp);
    numeric[i*7]=event.value;numeric[i*7+1]=age;numeric[i*7+2]=age;numeric[i*7+3]=event.gap;numeric[i*7+6]=1;
    fusion.input("event_mode")[i]=1;fusion.input("event_mask")[i]=1;fusion.input("event_targets")[i*j_+event.joint]=1;
    fusion.input("event_source_offset_s")[i]=-age;fusion.input("event_available_offset_s")[i]=-age;}
  auto &residual=fusion.input("residual"),&rm=fusion.input("residual_mask");const auto offset=511*j_*3;
  for(std::size_t i=0;i<j_;++i){for(std::size_t col=0;col<3;++col){residual[i*8+col]=past[offset+i*3+col];rm[i*8+col]=observed[offset+i*3+col];}
    const auto &meta=context_.at("joints").at(joints_[i]);if(meta.contains("position_lower") && meta.contains("position_upper") && rm[i*8]!=0.0F){
      const double lo=meta.at("position_lower"),hi=meta.at("position_upper");require(hi>lo,"invalid physical joint limits");const double p=static_cast<double>(residual[i*8]);residual[i*8+6]=static_cast<float>(std::min(p-lo,hi-p)/(hi-lo));rm[i*8+6]=1;}
    if(active.contains(i)){const auto &event=*active[i];residual[i*8+3]=event.value;residual[i*8+7]=seconds(anchor-event.stamp);rm[i*8+3]=rm[i*8+7]=1;
      if(rm[i*8]!=0.0F){double error=static_cast<double>(event.value-residual[i*8]);if(meta.at("type")=="continuous"){error=std::fmod(error+M_PI,2*M_PI);if(error<0)error+=2*M_PI;error-=M_PI;}residual[i*8+4]=static_cast<float>(error);rm[i*8+4]=1;}}}
  fusion.input("step_mask")[0]=1;fusion.input("time_features")[0]=last_<0?0:seconds(anchor-last_);fusion.input("time_features")[1]=eligible.size()>e_?1:0;
  if(last_>=0) {
    const auto skipped=std::max<Ns>(0,static_cast<Ns>(std::llround(static_cast<double>(anchor-last_)*manifest_.at("spec").at("decision_hz").get<double>()/1e9))-1);
    for(Ns k=0;k<std::min<Ns>(skipped,static_cast<Ns>(t_));++k)history_.push_back(std::vector<float>(tokens_*129,0));
  }
  history_.push_back(fusion.run().values);while(history_.size()>t_)history_.pop_front();last_=anchor;
  auto &decision=*graphs_.at("decision");auto &fused=decision.input("fused"),&evidence=decision.input("evidence"),&steps=decision.input("step_mask");std::fill(fused.begin(),fused.end(),0);std::fill(evidence.begin(),evidence.end(),0);std::fill(steps.begin(),steps.end(),0);
  for(std::size_t k=0;k<history_.size();++k){const auto slot=t_-history_.size()+k;steps[slot]=1;
    for(std::size_t token=0;token<tokens_;++token){std::copy_n(history_[k].begin()+static_cast<std::ptrdiff_t>(token*129),128,fused.begin()+static_cast<std::ptrdiff_t>((slot*tokens_+token)*128));evidence[slot*tokens_+token]=history_[k][token*129+128];}}
  const auto &out=decision.run().values;Json scores=Json::object();bool valid=false;
  for(std::size_t i=0;i<manifest_.at("propositions").size();++i){if(decision.input("proposition_mask")[i]==0)continue;const bool known=out[i*2+1]>.5F;valid=valid||known;
    scores[manifest_.at("propositions")[i].at("id").get<std::string>()]={{"logit",out[i*2]},{"valid",known},{"score",known?Json(1./(1.+std::exp(-static_cast<double>(out[i*2])))):Json(nullptr)},{"calibrated",false}};}
  return {{"status",valid?"valid":"insufficient_evidence"},{"scores",scores},{"t_ns",anchor},{"epoch",epoch},
    {"model_id",selection_.at("model_id")},{"source","experimental_critic"},{"mode","shadow"},{"enforcement_authorized",false}};
}
}
