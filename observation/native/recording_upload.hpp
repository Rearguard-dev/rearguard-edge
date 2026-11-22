#pragma once

#include "sync.hpp"

namespace harness::observation {

// Publishes exact incident evidence and selected execution samples. Rolling
// pre-roll segments are never a monitoring archive. Submitted means accepted
// for asynchronous verification/indexing, not already verified.
Json upload_closed_recordings(const fs::path &store, const fs::path &session,
                              const DeviceCredentials &credentials, bool include_execution_samples = true);
Ns recording_retry_delay_ns(unsigned attempts);

// Long-running, independently supervised upload queue. Recording and the
// safety service continue even while this process is blocked on cloud I/O.
Json run_recording_uploader(const Options &options);

} // namespace harness::observation
