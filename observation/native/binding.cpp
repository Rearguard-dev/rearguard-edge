#include "binding.hpp"

#include <algorithm>
#include <limits>

namespace harness::observation {
namespace {

bool has_type(const Json &topic, const std::string &type) {
  const auto types = topic.value("types", Json::array());
  return types.is_array() &&
         std::find(types.begin(), types.end(), type) != types.end();
}

bool captured(const Json &topic) {
  return topic.value("sample", Json::object()).value("status", "") ==
         "captured";
}

const Json &domain_for(const Json &inventory, Ns domain_id) {
  require(inventory.value("domains", Json::array()).is_array(),
          "setup inventory has no ROS domains");
  for (const auto &domain : inventory.at("domains"))
    if (domain.value("domain_id", std::numeric_limits<Ns>::min()) == domain_id)
      return domain;
  throw std::runtime_error("selected ROS domain is absent from setup inventory");
}

int domain_score(const Json &domain) {
  int joints = 0, cameras = 0, sampled = 0;
  for (const auto &topic : domain.value("topics", Json::array())) {
    if (has_type(topic, "sensor_msgs/msg/JointState") && captured(topic)) ++joints;
    if (has_type(topic, "sensor_msgs/msg/CompressedImage") && captured(topic)) ++cameras;
    if (captured(topic)) ++sampled;
  }
  if (joints != 1 || cameras == 0) return -1;
  return 1000 + cameras * 100 + sampled;
}

} // namespace

std::optional<Ns> select_observation_domain(const Json &inventory) {
  std::optional<Ns> selected;
  int best = -1;
  for (const auto &domain : inventory.value("domains", Json::array())) {
    if (!domain.is_object() || !domain.contains("domain_id") ||
        !domain["domain_id"].is_number_integer())
      continue;
    const auto score = domain_score(domain);
    const auto id = domain["domain_id"].get<Ns>();
    if (score > best || (score == best && selected && id < *selected)) {
      best = score;
      selected = id;
    }
  }
  return best >= 0 ? selected : std::nullopt;
}

Json compile_observation_binding(const Json &inventory, Ns domain_id,
                                 const std::string &robot_label) {
  const auto &domain = domain_for(inventory, domain_id);
  Json joint_topic;
  std::vector<Json> cameras;
  std::vector<Json> commands;
  bool advancing_clock = false;
  for (const auto &topic : domain.value("topics", Json::array())) {
    if (has_type(topic, "sensor_msgs/msg/JointState") && captured(topic)) {
      require(joint_topic.is_null(),
              "setup discovered multiple sampled JointState topics; operator selection is required");
      joint_topic = topic;
    }
    if (has_type(topic, "sensor_msgs/msg/CompressedImage") && captured(topic))
      cameras.push_back(topic);
    if (has_type(topic, "std_msgs/msg/Float64MultiArray") ||
        has_type(topic, "trajectory_msgs/msg/JointTrajectory"))
      commands.push_back(topic);
    if (topic.value("name", "") == "/clock" && captured(topic))
      advancing_clock = true;
  }
  require(!joint_topic.is_null(),
          "setup did not capture one usable JointState topic");
  require(!cameras.empty(), "setup did not capture a compressed camera topic");
  const auto sample = joint_topic.at("sample");
  const auto data = sample.value("data", Json::object());
  const auto joints = data.value("name", Json::array());
  require(joints.is_array() && !joints.empty(),
          "JointState sample did not contain joint names");

  std::sort(cameras.begin(), cameras.end(), [](const Json &left, const Json &right) {
    return left.at("name").get<std::string>() <
           right.at("name").get<std::string>();
  });
  std::sort(commands.begin(), commands.end(), [](const Json &left, const Json &right) {
    const auto left_live = left.value("publisher_count", 0) > 0;
    const auto right_live = right.value("publisher_count", 0) > 0;
    if (left_live != right_live) return left_live > right_live;
    return left.at("name").get<std::string>() <
           right.at("name").get<std::string>();
  });

  Json topics = Json::array({{{"name", joint_topic.at("name")},
                              {"type", "sensor_msgs/msg/JointState"},
                              {"role", "joint_state"}, {"required", true}}});
  // Commands are evidence when present, but passive incident capture must not
  // fail merely because the controller accepts commands without publishing them.
  if (!commands.empty()) {
    const auto type = has_type(commands.front(),
                               "trajectory_msgs/msg/JointTrajectory")
                          ? "trajectory_msgs/msg/JointTrajectory"
                          : "std_msgs/msg/Float64MultiArray";
    topics.push_back({{"name", commands.front().at("name")}, {"type", type},
                      {"role", "action"}, {"required", false}});
  }
  for (const auto &camera : cameras)
    topics.push_back({{"name", camera.at("name")},
                      {"type", "sensor_msgs/msg/CompressedImage"},
                      {"role", "camera"}, {"required", true}});

  return {{"schema_version", 1},
          {"mode", "observation_only"},
          {"robot", robot_label.empty() ? "discovered" : robot_label},
          {"clock", advancing_clock ? "ros_sim" : "ros_system"},
          {"critics", Json::array()}, {"recoveries", Json::array()},
          {"joint_order", joints}, {"command_interface", "position"},
          {"topics", topics},
          {"source", {{"kind", "setup_inventory"},
                      {"domain_id", domain_id}}}};
}

} // namespace harness::observation
