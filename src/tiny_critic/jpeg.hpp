#pragma once
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>
namespace harness::tiny {
struct RgbImage {
  std::size_t width, height;
  std::vector<std::uint8_t> pixels;
};
// Source JPEG only; zstd transports are deliberately not treated as JPEG.
RgbImage decode_jpeg(std::span<const std::uint8_t> bytes);
}
