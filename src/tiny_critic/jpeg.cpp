#include "jpeg.hpp"
#include <memory>
#include <stdexcept>
#include <turbojpeg.h>
namespace harness::tiny {
RgbImage decode_jpeg(std::span<const std::uint8_t> bytes) {
  if (bytes.size() < 3 || bytes.size() > 64 * 1024 * 1024 || bytes[0] != 0xff || bytes[1] != 0xd8)
    throw std::runtime_error("critic requires a valid JPEG camera packet");
  std::unique_ptr<void, decltype(&tjDestroy)> decoder(tjInitDecompress(), tjDestroy);
  if (!decoder) throw std::runtime_error("could not initialize JPEG decoder");
  int width = 0, height = 0, subsampling = 0, colorspace = 0;
  if (tjDecompressHeader3(decoder.get(), bytes.data(), static_cast<unsigned long>(bytes.size()),
                          &width, &height, &subsampling, &colorspace) != 0 ||
      width <= 0 || height <= 0 || static_cast<std::size_t>(width) * static_cast<std::size_t>(height) > 16777216)
    throw std::runtime_error("invalid JPEG dimensions/header");
  RgbImage image{static_cast<std::size_t>(width), static_cast<std::size_t>(height),
                 std::vector<std::uint8_t>(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3)};
  if (tjDecompress2(decoder.get(), bytes.data(), static_cast<unsigned long>(bytes.size()),
                    image.pixels.data(), width, width * 3, height, TJPF_RGB, TJFLAG_ACCURATEDCT) != 0)
    throw std::runtime_error("JPEG camera decode failed");
  return image;
}
}
