#include "recording.hpp"
#include <iomanip>
#include <rosbag2_cpp/reader.hpp>
#include <set>
#include <sstream>
namespace harness::observation {
namespace {
// Keep enough closed pre-roll segments for a T-10s..T+5s incident window to
// finish before export. The exported MCAP is still clipped to exactly 15s.
constexpr Ns kRetentionNs = 30000000000LL;
constexpr Ns kSegmentDurationNs = 2000000000LL;
constexpr std::size_t kSegmentByteTarget = 32U * 1024U * 1024U;

rosbag2_storage::StorageOptions storage_options(const fs::path &path,
                                                const std::string &storage) {
  rosbag2_storage::StorageOptions options;
  options.uri = path.string();
  options.storage_id = storage;
  return options;
}
} // namespace
Recording::Recording(fs::path session, Json topics, std::string storage)
    : session_(std::move(session)), topics_(std::move(topics)),
      storage_(std::move(storage)) {
  atomic_json(session_ / "recording.json",
              {{"schema_version", 2},
               {"mode", "incident_pre_roll"},
               {"retention_ns", kRetentionNs},
               {"segments", segments_}});
}
void Recording::write(const std::string &topic, const std::string &type,
                      std::shared_ptr<const rclcpp::SerializedMessage> data,
                      Ns now, Ns epoch) {
  if (writer_ && (epoch != current_["epoch"].get<Ns>() ||
                  now - current_["start_ns"].get<Ns>() >= kSegmentDurationNs ||
                  current_bytes_ >= kSegmentByteTarget))
    close();
  if (!writer_) {
    std::ostringstream name;
    name << "rosbag2/segment-" << std::setw(6) << std::setfill('0')
         << next_segment_++;
    current_ = {{"epoch", epoch},
                {"start_ns", now},
                {"end_ns", now},
                {"path", name.str()},
                {"storage_id", storage_},
                {"target_duration_ns", kSegmentDurationNs},
                {"target_size_bytes", kSegmentByteTarget},
                {"message_counts", Json::object()}};
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    current_bytes_ = 0;
    writer_->open(storage_options(session_ / name.str(), storage_),
                  {"cdr", "cdr"});
    for (const auto &item : topics_) {
      rosbag2_storage::TopicMetadata metadata;
      metadata.name = item["name"];
      metadata.type = item["type"];
      metadata.serialization_format = "cdr";
      writer_->create_topic(metadata);
    }
  }
  current_bytes_ += data->size();
  writer_->write(data, topic, type, now, now);
  current_["end_ns"] = now;
  auto &message_counts = current_.at("message_counts");
  const auto count = message_counts.contains(topic)
                         ? message_counts.at(topic).get_ref<const Json::number_unsigned_t &>()
                         : Json::number_unsigned_t{0};
  message_counts[topic] = count + 1U;
}
void Recording::close() {
  if (!writer_)
    return;
  writer_->close();
  writer_.reset();
  current_["serialized_bytes"] = current_bytes_;
  segments_.push_back(current_);
  prune(current_.at("end_ns").get<Ns>());
  atomic_json(session_ / "recording.json",
              {{"schema_version", 2},
               {"mode", "incident_pre_roll"},
               {"retention_ns", kRetentionNs},
               {"segments", segments_}});
}

void Recording::prune(Ns newest_end_ns) {
  const auto cutoff = newest_end_ns - kRetentionNs;
  Json retained = Json::array();
  for (const auto &segment : segments_) {
    if (segment.at("epoch").get<Ns>() == current_.at("epoch").get<Ns>() &&
        segment.at("end_ns").get<Ns>() >= cutoff) {
      retained.push_back(segment);
      continue;
    }
    const fs::path relative(segment.at("path").get<std::string>());
    require(!relative.empty() && !relative.is_absolute(),
            "recording segment path must be relative");
    for (const auto &part : relative)
      require(part != "..", "recording segment path may not escape session");
    std::error_code error;
    fs::remove_all(session_ / relative, error);
    require(!error, "could not retire expired pre-roll segment: " +
                        error.message());
  }
  segments_ = std::move(retained);
}
fs::path export_window(const fs::path &session, const Json &window,
                       const fs::path &destination) {
  require(window.at("status") == "ready", "only ready windows can be exported");
  const auto manifest = read_json(session / "recording.json");
  const auto config = read_json(session / "session.json")["config"];
  std::vector<Json> segments;
  for (const auto &s : manifest["segments"])
    if (s["epoch"] == window["epoch"] &&
        s["end_ns"].get<Ns>() >= window["start_ns"].get<Ns>() &&
        s["start_ns"].get<Ns>() <= window["end_ns"].get<Ns>())
      segments.push_back(s);
  require(!segments.empty(), "no closed evidence segments for window");
  std::map<std::string, unsigned> counts;
  std::set<std::string> registered;
  rosbag2_cpp::Writer writer;
  writer.open(storage_options(destination, "mcap"), {"cdr", "cdr"});
  for (const auto &s : segments) {
    rosbag2_cpp::Reader reader;
    reader.open(storage_options(session / s["path"].get<std::string>(),
                                s["storage_id"]),
                {"cdr", "cdr"});
    for (const auto &topic : reader.get_all_topics_and_types())
      if (registered.insert(topic.name).second)
        writer.create_topic(topic);
    while (reader.has_next()) {
      const auto message = reader.read_next();
      if (message->recv_timestamp >= window["start_ns"].get<Ns>() &&
          message->recv_timestamp <= window["end_ns"].get<Ns>()) {
        writer.write(message);
        ++counts[message->topic_name];
      }
    }
  }
  writer.close();
  for (const auto &group : required_topic_groups(config)) {
    bool covered = false;
    for (const auto &name : group) covered = covered || counts[name] >= 2;
    require(covered, "exported MCAP lacks required topic coverage");
  }
  std::vector<fs::path> files;
  for (const auto &entry : fs::directory_iterator(destination))
    if (entry.path().extension() == ".mcap")
      files.push_back(entry.path());
  require(files.size() == 1, "expected exactly one exported MCAP");
  return files.front();
}
} // namespace harness::observation
