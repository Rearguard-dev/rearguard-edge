#include "update.hpp"
#include "service.hpp"
#include <cerrno>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <vector>
#include <sys/wait.h>
#include <unistd.h>

#ifndef REARGUARD_VERSION
#define REARGUARD_VERSION "0.0.0"
#endif

namespace harness::cli {
int run_update(const std::string &version, bool check_only) {
  namespace fs = std::filesystem;
  try {
    if (version != "latest" && !std::regex_match(version, std::regex(
        R"(v?(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)(-[0-9A-Za-z-]+(\.[0-9A-Za-z-]+)*)?(\+[0-9A-Za-z-]+(\.[0-9A-Za-z-]+)*)?)")))
      throw std::runtime_error("--version must be latest or a release version such as v0.4.28");
    const auto executable = fs::canonical(observation::service_executable());
    const auto prefix = executable.parent_path().parent_path();
    if (executable.parent_path().filename() != "bin" || executable.filename() != "rearguard")
      throw std::runtime_error("update requires an installed bin/rearguard; use the original installer for a source build");
    const auto installer = prefix / "share/harness/install/install.sh";
    if (!fs::is_regular_file(installer))
      throw std::runtime_error("this installation lacks the bundled updater; upgrade once using the original installer");
    // Copy before starting: the installer replaces share/ while this shell runs.
    auto temporary = (fs::temp_directory_path() / "rearguard-update-XXXXXX").string();
    std::vector<char> path(temporary.begin(), temporary.end()); path.push_back('\0');
    const int fd = ::mkstemp(path.data());
    if (fd < 0) throw std::runtime_error("could not create temporary installer");
    ::close(fd);
    struct Cleanup { fs::path path; ~Cleanup() { std::error_code error; fs::remove(path, error); } } cleanup{path.data()};
    fs::copy_file(installer, cleanup.path, fs::copy_options::overwrite_existing);
    std::cout << (check_only ? "Checking" : "Updating") << " Rearguard " << REARGUARD_VERSION
              << " at " << prefix << " using the original release installer\n" << std::flush;
    const auto tag = version == "latest" || version.starts_with('v') ? version : "v" + version;
    const auto child = ::fork();
    if (child < 0) throw std::runtime_error("could not start release installer");
    if (child == 0) {
      if (::setenv("HARNESS_INSTALL_DIR", prefix.c_str(), 1) != 0 ||
          ::setenv("REARGUARD_UPDATE_FROM", REARGUARD_VERSION, 1) != 0 ||
          ::setenv("REARGUARD_UPDATE_CHECK_ONLY", check_only ? "1" : "0", 1) != 0)
        ::_exit(126);
      ::execl("/bin/sh", "sh", path.data(), tag.c_str(), static_cast<char *>(nullptr));
      ::_exit(127);
    }
    int status = 0;
    while (::waitpid(child, &status, 0) < 0) {
      if (errno != EINTR) throw std::runtime_error("could not wait for release installer");
    }
    return WIFEXITED(status) ? WEXITSTATUS(status) : 1;
  } catch (const std::exception &error) {
    std::cerr << "rearguard update: " << error.what() << '\n';
    return 1;
  }
}
}
