#include "jpeg.hpp"
#include <turbojpeg.h>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <iostream>
int main(int argc, char **argv) {
  try {
    if (argc == 3) {
      std::ifstream input(argv[1], std::ios::binary);
      std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(input)), {});
      const auto image = harness::tiny::decode_jpeg(bytes);
      std::ofstream output(argv[2], std::ios::binary);
      output.write(reinterpret_cast<const char *>(image.pixels.data()), static_cast<std::streamsize>(image.pixels.size()));
      return output ? 0 : 1;
    }
    auto encoder = tjInitCompress();
    std::vector<std::uint8_t> rgb(8 * 8 * 3, 120);
    unsigned char *jpeg = nullptr;
    unsigned long size = 0;
    const auto encoded = tjCompress2(encoder, rgb.data(), 8, 24, 8, TJPF_RGB, &jpeg, &size, TJSAMP_444, 95, 0);
    tjDestroy(encoder);
    if (encoded != 0) throw std::runtime_error("test JPEG encoding failed");
    std::vector<std::uint8_t> bytes(jpeg, jpeg + size);
    tjFree(jpeg);
    const auto decoded = harness::tiny::decode_jpeg(bytes);
    if (decoded.width != 8 || decoded.height != 8 || decoded.pixels != rgb)
      throw std::runtime_error("JPEG RGB layout mismatch");
    bool rejected = false;
    try { harness::tiny::decode_jpeg(std::vector<std::uint8_t>{1, 2, 3}); }
    catch (const std::exception &) { rejected = true; }
    if (!rejected) throw std::runtime_error("non-JPEG accepted");
    bytes.resize(bytes.size() / 2);
    rejected = false;
    try { harness::tiny::decode_jpeg(bytes); }
    catch (const std::exception &) { rejected = true; }
    if (!rejected) throw std::runtime_error("truncated JPEG accepted");
    std::cout << "JPEG decode and invalid-packet checks passed\n";
    return 0;
  } catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
