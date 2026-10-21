#include "purge.hpp"

#include "events.hpp"
#include "service.hpp"

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <system_error>

namespace harness::cli {
namespace fs = std::filesystem;
namespace {

std::string escape_json(const std::string& input) {
    std::string output;
    for (const char character : input) {
        if (character == '"' || character == '\\') output += '\\';
        output += character;
    }
    return output;
}

fs::path normalized(const fs::path& path, std::error_code& error) {
    const auto absolute = fs::absolute(path, error).lexically_normal();
    if (error) return {};
    const auto resolved = fs::weakly_canonical(absolute, error);
    return error ? fs::path{} : resolved;
}

bool safe_state_root(const fs::path& requested, fs::path& target, std::string& error) {
    std::error_code ec;
    target = normalized(requested, ec);
    if (ec || target.empty() || target == target.root_path()) {
        error = "refusing unsafe state path " + requested.string();
        return false;
    }

    if (const char* home = std::getenv("HOME"); home && *home) {
        std::error_code home_error;
        const auto resolved_home = normalized(home, home_error);
        if (!home_error && target == resolved_home) {
            error = "refusing to purge the home directory " + target.string();
            return false;
        }
    }

    std::size_t depth = 0;
    for (const auto& component : target.relative_path()) {
        (void)component;
        ++depth;
    }
    if (depth < 3) {
        error = "refusing broad state path " + target.string();
        return false;
    }

    if (!fs::exists(target, ec)) return true;
    if (ec || !fs::is_directory(target, ec)) {
        error = "state path is not a directory: " + target.string();
        return false;
    }

    const bool recognized =
        fs::exists(target / "events.jsonl", ec) ||
        fs::exists(target / "service" / "status.json", ec) ||
        fs::exists(target / "service" / "config.json", ec) ||
        fs::is_directory(target / "service" / "sessions", ec);
    if (!recognized) {
        error = "refusing unrecognized state directory " + target.string();
        return false;
    }
    return true;
}

std::uintmax_t count_files(const fs::path& root) {
    std::error_code ec;
    std::uintmax_t count = 0;
    for (fs::recursive_directory_iterator iterator(root, ec), end;
         !ec && iterator != end; iterator.increment(ec)) {
        if (!iterator->is_directory(ec)) ++count;
    }
    return count;
}

}  // namespace

int run_purge(const PurgeOptions& options) {
    fs::path target;
    std::string safety_error;
    if (!safe_state_root(harness::observation::state_root(), target, safety_error)) {
        std::fprintf(stderr, "error: %s\n", safety_error.c_str());
        return 1;
    }

    std::error_code ec;
    const bool exists = fs::exists(target, ec);
    if (ec) {
        std::fprintf(stderr, "error: cannot inspect %s: %s\n", target.string().c_str(),
                     ec.message().c_str());
        return 1;
    }
    const auto files = exists ? count_files(target) : 0;

    if (options.dry_run) {
        if (options.json) {
            std::printf("{\"path\":\"%s\",\"dry_run\":true,\"files\":%llu,"
                        "\"removed\":false}\n",
                        escape_json(target.string()).c_str(),
                        static_cast<unsigned long long>(files));
        } else {
            std::printf("would stop Rearguard and permanently remove %llu file(s) from:\n"
                        "  %s\n\npairing credentials will be preserved\n"
                        "dry run: nothing was removed\n",
                        static_cast<unsigned long long>(files), target.string().c_str());
        }
        return 0;
    }

    if (!exists) {
        if (options.json) {
            std::printf("{\"path\":\"%s\",\"dry_run\":false,\"files\":0,"
                        "\"removed\":true,\"status\":\"already_empty\"}\n",
                        escape_json(target.string()).c_str());
        } else {
            std::printf("nothing to purge: %s does not exist\n", target.string().c_str());
        }
        return 0;
    }

    if (!options.assume_yes) {
        std::printf("This will stop Rearguard and permanently remove %llu file(s) from:\n"
                    "  %s\n\nPairing credentials will be preserved. Purge state? [y/N] ",
                    static_cast<unsigned long long>(files), target.string().c_str());
        std::fflush(stdout);
        std::string answer;
        if (!std::getline(std::cin >> std::ws, answer) ||
            (answer != "y" && answer != "Y")) {
            std::printf("aborted; nothing was removed\n");
            return 1;
        }
    }

    try {
        harness::observation::stop_service_if_running();
    } catch (const std::exception& exception) {
        std::fprintf(stderr, "error: cannot stop Rearguard before purge: %s\n",
                     exception.what());
        return 1;
    }

    fs::remove_all(target, ec);
    if (ec) {
        std::fprintf(stderr, "error: failed to purge %s: %s\n", target.string().c_str(),
                     ec.message().c_str());
        return 1;
    }

    if (options.json) {
        std::printf("{\"path\":\"%s\",\"dry_run\":false,\"files\":%llu,"
                    "\"removed\":true}\n",
                    escape_json(target.string()).c_str(),
                    static_cast<unsigned long long>(files));
    } else {
        std::printf("purged Rearguard state from %s\n", target.string().c_str());
        std::printf("pairing credentials were preserved; run `rearguard start` when ready\n");
    }
    return 0;
}

}  // namespace harness::cli
