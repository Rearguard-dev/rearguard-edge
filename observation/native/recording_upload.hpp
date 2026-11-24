#pragma once

#include "sync.hpp"
#include <thread>

namespace harness::observation {

// Publishes exact incident evidence and selected execution samples. Rolling
// pre-roll segments are never a monitoring archive. Submitted means accepted
// for asynchronous verification/indexing, not already verified.
Json upload_closed_recordings(const fs::path &store, const fs::path &session,
                              const DeviceCredentials &credentials, bool include_execution_samples = true,
                              bool spool_local = true);
Ns recording_retry_delay_ns(unsigned attempts);
// Local-only export; never waits for HTTP. One spooler owns a profile at a time.
void spool_execution_samples(const fs::path &store, const fs::path &session);
class ExecutionSampleSpooler {
  std::jthread worker_;
public:
  ExecutionSampleSpooler(fs::path store, fs::path sessions_root);
};
// Preserve full acknowledgement history, reduce completed entries to tombstones.
bool compact_recording_journal(const fs::path &path);

// Long-running, independently supervised upload queue. Recording and the
// safety service continue even while this process is blocked on cloud I/O.
Json run_recording_uploader(const Options &options);

} // namespace harness::observation
