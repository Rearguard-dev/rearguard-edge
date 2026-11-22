#pragma once
#include "critic.hpp"
#include <deque>
#include <map>

namespace harness::tiny {
class PhysicalGraph {
  std::vector<Tensor> inputs_;
  Tensor output_;
  std::unique_ptr<Engine> engine_;
public:
  PhysicalGraph(const fs::path &,const Json &);
  std::vector<float> &input(const std::string &);
  const Tensor &run();
  const std::vector<Tensor> &inputs() const {return inputs_;}
};
// PIL-compatible uint8 bicubic antialias, shortest edge 235 and center crop
// 224, then ImageNet normalization. Separate from the legacy CLIP transform.
void physical_pixels(std::span<const std::uint8_t>,std::size_t,std::size_t,std::span<float>);

class PhysicalModel {
  Json manifest_,binding_,selection_,context_;
  std::map<std::string,std::unique_ptr<PhysicalGraph>> graphs_;
  std::vector<std::string> joints_,cameras_;
  std::size_t j_,c_,e_,t_,tokens_;
  struct State {Ns source,available;std::vector<float> values,known;};
  struct Event {Ns stamp;std::size_t joint;float value,gap;std::string id;};
  struct Camera {Ns source=-1,available=-1;std::vector<float> features;};
  std::deque<State> states_;
  std::deque<Event> events_;
  std::vector<Camera> camera_;
  std::vector<Ns> previous_event_;
  std::deque<std::vector<float>> history_;
  Ns last_=-1,epoch_=-1;
  std::uint64_t event_id_=0;
  void embodiment();
public:
  PhysicalModel(const fs::path &,Json binding,Json selection);
  const Json &manifest() const {return manifest_;}
  const std::vector<std::string> &cameras() const {return cameras_;}
  const std::vector<std::string> &joints() const {return joints_;}
  void reset(Ns epoch);
  void state(Ns source,Ns available,std::vector<float> values,std::vector<float> known);
  void command(Ns available,const std::vector<double> &values);
  void camera(std::size_t,Ns source,Ns available,std::span<const std::uint8_t>,std::size_t,std::size_t);
  Json score(Ns anchor,Ns epoch);
  Json debug_inputs() const;
};
}
