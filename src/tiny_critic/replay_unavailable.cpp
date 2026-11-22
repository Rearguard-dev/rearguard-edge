#include "critic.hpp"
#include "runtime.hpp"
namespace harness::tiny {
Json replay_mcap(Critic &, const fs::path &, const Json &, std::ostream &) {
  throw std::runtime_error("MCAP decoding requires HARNESS_WITH_ROS2=ON (native rosbag2_cpp/CDR)");
}
}
namespace harness::observation {
std::unique_ptr<CriticAdapter> make_physical_critic(const fs::path &,const Json &,const Json &) {
  throw std::runtime_error("physical live inference requires native ROS support");
}
}
