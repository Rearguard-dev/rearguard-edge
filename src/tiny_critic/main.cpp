#include "critic.hpp"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <chrono>
#include <cmath>
#include <sys/utsname.h>

using namespace harness::tiny;
using harness::observation::require;
int main(int argc, char **argv) {
  try {
    if (argc < 3) {
      std::cerr << "harness_critic <inspect|seal-engines|score|replay|encode|benchmark> BUNDLE [onnx|tensorrt] [INPUT] [BINDING]\n";
      return 2;
    }
    const std::string command = argv[1];
    const fs::path bundle = argv[2];
    if (command == "inspect") {
      std::cout << harness::observation::read_json(bundle / "manifest.json").dump(2) << '\n';
      return 0;
    }
    if (command == "seal-engines") {
      Json engines{{"manifest_hash", harness::observation::file_hash(bundle / "manifest.json")}};
      for (const auto *name : {"vision.engine", "head.engine"}) engines[name] = harness::observation::file_hash(bundle / name);
      harness::observation::atomic_json(bundle / "engines.json", engines);
      std::cout << engines.dump(2) << '\n';
      return 0;
    }
    require(argc >= 5, "backend and input are required");
    Critic critic(bundle, argv[3]);
    if (command == "benchmark") {
      const auto iterations = std::stoull(argv[4]);
      require(iterations >= 10 && iterations <= 100000, "benchmark requires 10–100000 iterations");
      const auto &s = critic.spec();
      const double budget_us = argc > 5 ? std::stod(argv[5]) : 1e6 / s.sample_hz;
      require(std::isfinite(budget_us) && budget_us > 0, "positive latency budget required");
      Frame f(s); f.valid = true;
      std::fill(f.camera_mask.begin(), f.camera_mask.end(), 1.F);
      std::fill(f.joint_mask.begin(), f.joint_mask.end(), 1.F);
      std::vector<std::uint8_t> rgb(224 * 224 * 3, 127);
      std::vector<double> durations;
      durations.reserve(static_cast<std::size_t>(iterations));
      const auto warmup = std::max<std::size_t>(s.steps, 32);
      for (std::size_t i = 0; i < warmup + iterations; ++i) {
        f.t_ns = static_cast<Ns>(static_cast<double>(i) * 1e9 / s.sample_hz);
        const auto begin = std::chrono::steady_clock::now();
        for (std::size_t camera = 0; camera < s.cameras; ++camera)
          critic.encode(rgb, 224, 224, 224 * 3,
              std::span<float>(f.vision.data() + camera * feature_size, feature_size));
        const auto result = critic.push(f);
        const auto us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - begin).count();
        require(result.value("status", "") == "valid" || i < warmup, "benchmark inference unavailable");
        if (i >= warmup) durations.push_back(us);
      }
      std::sort(durations.begin(), durations.end());
      struct utsname platform {};
      require(::uname(&platform) == 0, "cannot identify benchmark platform");
      const auto percentile = [&](double p) { return durations[static_cast<std::size_t>(p * static_cast<double>(durations.size() - 1))]; };
      std::cout << Json{{"iterations", iterations}, {"warmup", warmup},
          {"p50_us", percentile(.5)}, {"p95_us", percentile(.95)}, {"p99_us", percentile(.99)},
          {"max_us", durations.back()}, {"budget_us", budget_us},
          {"within_budget", durations.back() <= budget_us}, {"authoritative", false},
          {"platform", std::string(platform.sysname) + " " + platform.release + " " + platform.machine},
          {"backend", argv[3]}, {"sample_hz", s.sample_hz}, {"window", critic.manifest().at("window")},
          {"workload", "synthetic RGB preprocessing + all cameras + window head; run under real policy load"},
          {"manifest_hash", harness::observation::file_hash(bundle / "manifest.json")}}.dump(2) << '\n';
      return durations.back() <= budget_us ? 0 : 3;
    } else if (command == "encode") {
      require(argc == 8, "encode requires packed RGB file, width, height and output .f32 path");
      const auto width = std::stoull(argv[5]), height = std::stoull(argv[6]);
      require(width > 0 && width <= 16384 && height > 0 && height <= 16384 && width * height <= 16777216,
              "invalid image dimensions");
      require(fs::file_size(argv[4]) == width * height * 3, "packed RGB file size mismatch");
      std::vector<std::uint8_t> rgb(static_cast<std::size_t>(width * height * 3));
      std::ifstream input(argv[4], std::ios::binary);
      input.read(reinterpret_cast<char *>(rgb.data()), static_cast<std::streamsize>(rgb.size()));
      require(input.good(), "cannot read packed RGB file");
      std::vector<float> features(feature_size);
      critic.encode(rgb,static_cast<std::size_t>(width),static_cast<std::size_t>(height),static_cast<std::size_t>(width * 3),features);
      require(!fs::exists(argv[7]), "output file already exists");
      std::ofstream output(argv[7], std::ios::binary);
      output.write(reinterpret_cast<const char *>(features.data()), static_cast<std::streamsize>(features.size() * sizeof(float)));
      require(output.good(), "cannot write features");
    } else if (command == "replay") {
      require(argc == 6, "replay requires an MCAP and binding JSON");
      replay_mcap(critic, argv[4], harness::observation::read_json(argv[5]), std::cout);
    } else if (command == "score") {
      // Diagnostic canonical tensor input, float32 little endian, in the order
      // vision/camera_mask/sensors/joint_mask/step_mask. Text comes from bundle.
      const auto &s = critic.spec();
      const std::array<std::size_t, 5> widths{s.cameras * feature_size, s.cameras,
                                            s.joints * sensor_size, s.joints, 1};
      std::array<std::vector<float>, 5> inputs;
      std::size_t total = 0;
      for (const auto width : widths) total += width * s.steps * sizeof(float);
      require(fs::file_size(argv[4]) == total, "canonical tensor file size mismatch");
      std::ifstream file(argv[4], std::ios::binary);
      for (std::size_t k = 0; k < inputs.size(); ++k) {
        inputs[k].resize(widths[k] * s.steps);
        file.read(reinterpret_cast<char *>(inputs[k].data()), static_cast<std::streamsize>(inputs[k].size() * sizeof(float)));
      }
      require(file.good(), "cannot read canonical tensors");
      Frame f(s);
      Json result;
      for (std::size_t t = 0; t < s.steps; ++t) {
        std::array<std::vector<float> *,4> destinations{&f.vision, &f.camera_mask, &f.sensors, &f.joint_mask};
        for (std::size_t k = 0; k < destinations.size(); ++k)
          std::copy_n(inputs[k].begin() + static_cast<std::ptrdiff_t>(t * widths[k]), widths[k], destinations[k]->begin());
        require(inputs[4][t] == 0 || inputs[4][t] == 1, "invalid step mask");
        f.valid = inputs[4][t] == 1; f.t_ns = static_cast<Ns>(static_cast<double>(t) * 1e9 / s.sample_hz);
        result = critic.push(f);
      }
      std::cout << result.dump() << '\n';
    } else throw std::runtime_error("unknown critic command");
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "critic: " << error.what() << '\n';
    return 1;
  }
}
