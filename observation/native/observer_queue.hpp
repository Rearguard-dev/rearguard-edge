#pragma once
#include "common.hpp"
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <set>

namespace harness::observation {
// The ROS executor only hands off shared serialized packets. Disk/SQLite work
// runs on the consumer, never under this mutex or on the ROS receive thread.
class ObserverQueue {
public:
  struct Task {
    Ns ros, monotonic;
    std::size_t bytes;
    std::string periodic;
    std::function<void()> run;
  };
private:
  mutable std::mutex mutex_;
  std::condition_variable ready_;
  std::deque<Task> tasks_;
  std::set<std::string> periodic_;
  std::size_t bytes_=0, peak_bytes_=0, peak_tasks_=0;
  const std::size_t max_bytes_, max_tasks_;
  bool overflow_=false;
public:
  explicit ObserverQueue(std::size_t max_bytes=64U*1024U*1024U,
                         std::size_t max_tasks=4096)
      :max_bytes_(max_bytes),max_tasks_(max_tasks) {}
  // Read the clock under the queue lock so concurrent producers retain ordering.
  void push(std::size_t bytes, std::function<Ns()> ros_clock,
            std::function<void()> run, std::string periodic={}) {
    std::lock_guard lock(mutex_);
    if (overflow_ || (!periodic.empty() && periodic_.contains(periodic))) return;
    if (tasks_.size()>=max_tasks_ || bytes>max_bytes_-bytes_) {
      overflow_=true;ready_.notify_one();return;
    }
    tasks_.push_back({ros_clock(),monotonic_ns(),bytes,periodic,std::move(run)});
    if (!periodic.empty()) periodic_.insert(periodic);
    bytes_+=bytes;peak_bytes_=std::max(peak_bytes_,bytes_);
    peak_tasks_=std::max(peak_tasks_,tasks_.size());
    ready_.notify_one();
  }
  std::optional<Task> pop() {
    std::unique_lock lock(mutex_);
    ready_.wait_for(lock,std::chrono::milliseconds(10),[this]{return overflow_ || !tasks_.empty();});
    require(!overflow_,"observer receive queue overflow: recording continuity lost");
    if (tasks_.empty()) return std::nullopt;
    auto task=std::move(tasks_.front());tasks_.pop_front();bytes_-=task.bytes;
    if (!task.periodic.empty()) periodic_.erase(task.periodic);
    return task;
  }
  Json status() const {
    std::lock_guard lock(mutex_);
    return {{"queued_messages",tasks_.size()},{"queued_bytes",bytes_},
            {"peak_messages",peak_tasks_},{"peak_bytes",peak_bytes_},
            {"max_messages",max_tasks_},{"max_bytes",max_bytes_},{"overflow",overflow_}};
  }
};
} // namespace harness::observation
