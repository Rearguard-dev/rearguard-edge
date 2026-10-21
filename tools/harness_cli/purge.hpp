#ifndef HARNESS_CLI_PURGE_HPP
#define HARNESS_CLI_PURGE_HPP

namespace harness::cli {

struct PurgeOptions {
    bool dry_run = false;
    bool assume_yes = false;
    bool json = false;
};

// Stops the service and removes the complete Rearguard state root while
// preserving the separately stored paired-device credential.
int run_purge(const PurgeOptions& options);

}  // namespace harness::cli

#endif  // HARNESS_CLI_PURGE_HPP
