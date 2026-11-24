# Automatic execution sampling

The native observer selects ordinary-operation windows; the existing supervised
uploader exports and uploads them. There is no button or additional daemon.
Samples are **unlabeled**, `training_eligible: false`, and never workflow failure
candidates. Incident recording and enforcement behavior are unchanged.

## Initial policy

Enabled when a bound native observer starts. An optional profile-store file
`execution-sampling-policy.json` overrides the defaults below, read on observer
startup. This file is a local configuration interface, not a cloud/UI setting.
Disabling or invalid configuration disables sampling without stopping observation.

```json
{
  "schema_version": 1,
  "enabled": true,
  "mean_interval_s": 30,
  "joint_delta": 0.01,
  "daily_limit": 12,
  "bootstrap_limit": 40,
  "failure_ratio": 3,
  "pool_limit": 200,
  "max_window_bytes": 67108864,
  "pool_bytes": 4294967296,
  "pending_bytes": 268435456
}
```

Measured displacement over 100–500 ms must exceed `joint_delta` in at least one
bound joint. The threshold uses joint position units (radians for SO101); other
embodiments need an appropriate setting. Valid, recently source-stamped joint
states must continue arriving. Changed command targets also count as work,
including possible stalls; repeated identical targets and a first target alone
do not. Camera traffic and policy connectivity do not count as activity. The
initial command threshold uses the same units as position; non-position
interfaces should use a separately reviewed policy before collecting there.

Activity stays eligible for twenty seconds after the latest movement/change,
including stationary pauses long enough to yield a fully stationary 15-second window. No execution/action-server state is inferred
from mere ROS topic presence. The initial implementation does not allocate
separate pause/motion quotas or pose-diversity buckets.

The first selection with an empty reservation ledger is due after 15 seconds
of eligible activity. Subsequent intervals are uniform between 15 seconds and
`2 * mean_interval_s - 15`: 15–45 seconds with the default mean of 30 seconds.
This removes the several-minute random tail. Setting `mean_interval_s` to 15
selects every 15 eligible seconds. Existing explicit policy overrides remain
in effect; a configured mean of 300 should be lowered for short operations.
Selections are independent of critic scores and retain retrospective 15-second
windows without overlap. Inactivity pauses the eligible-time counter; daily,
pool and pending-byte budgets still apply. A short operation can qualify through
the existing twenty-second activity tail, provided joint states keep arriving.

This collection cadence does not schedule critic predictions. The native critic
uses its model's `decision_hz` (currently 30 Hz) to evaluate rolling context;
it does not wait for a complete saved MCAP. Scheduling frequency is not a
measured end-to-end detection latency guarantee.

Required streams must cover the interval, with no receipt gap or
endpoint gap above one second, and no known invalid messages. The exported MCAP
is checked again, so expired pre-roll cannot be accepted using old metadata.
Image payload decoding remains the existing cloud verifier/renderer's job.
Camera acquisition/delivery delay is not relabeled or rewritten. Command events
inside the window are retained; zero command events is allowed. An earlier
latched command is not fabricated; downstream annotation/training must account
for missing initial command context.

## Budgets and durability

Pool capacity is `min(pool_limit, max(bootstrap_limit, failure_ratio * F))`, where
F counts distinct real evidence hashes attached to accepted failures in the
local profile snapshot, never synthetic examples or raw critic alerts.

The profile-wide exclusive lock and atomic `execution-sampling-ledger.json`
reserve capacity before capture. UTC-day counts and byte reservations survive
observer restarts and ROS clock resets. A selection reserves the maximum window
size conservatively; rejected/crashed selections still consume a daily/pool
reservation. Submitted or rejected entries release pending capacity. This bounds
both offline queues and total retention without depending on the cloud.
Sampling pauses if free disk space cannot preserve 250 MB plus two window
reservations. This is additional to the recorder's existing free-space policy.

At capacity collection pauses. There is **no reservoir replacement, automatic
deletion, or eviction of old/pinned recordings** in this version. Do not delete
the ledger to reset quotas while retained recordings exist. Lifecycle/rotation
needs explicit pinning and a catalog-aware cleanup operation in a later update.

## Export and catalog

Selected windows carry `execution_sample` metadata in session `state.json`.
They bypass workflow candidate creation. A local worker in the uploader process
spools ready samples independently of network calls, to `execution-samples/<sample-id>.mcap` under the profile
store. A paired JSON manifest records session/window, selection reason, policy,
binding/context, SHA256 and size. Operator-reported or accepted-failure overlap known before export rejects
the sample. Unvalidated experimental critic alerts do not block selection or
export: `unconfirmed_alert_overlaps` retains their detection metadata for review.
These samples remain unlabeled and ineligible for training until reviewed. A later unreported/reported incident does not automatically rewrite
an already uploaded sample's catalog metadata: all samples remain unlabeled.

Upload uses the same authenticated immutable object grants, completion endpoint
and asynchronous verification/catalog as incident evidence. Retries reuse the
same recording identity and content hash. Completion markers say submitted for
verification, not verified. Network work and MCAP export happen on independent workers in the existing
uploader process, outside the observer/safety loop. The local worker polls at
500 ms, skips unchanged session state, and takes a profile-wide export lock.
A blocked upload does not prevent local export. Disk/CPU failures can still delay
export; unavailable pre-roll is rejected rather than replaced by another interval.
The worker reads the full workflow snapshot only when a sample actually needs
export, rather than on every observation heartbeat.

The existing generic recording catalog accepts `manifest.kind=execution_sample`;
no database migration is required. Incident-specific automatic preview jobs and
investigation/seed joins are not reused. Training admission/review, fixed-eval
reservation, automatic labeling, cloud configuration and dedicated sampling UI
are **not** added by this edge change. Review source samples before using them.

## Visibility and verification

The current session's `state.json` includes `execution_sampling` with policy,
reservation count, draw progress, and sampling/budget status. The event log
reports execution_sampling/spooled and execution_sampling/rejected; immutable
completion/rejection JSON files live alongside sample MCAPs. Stream headers and
joint/command values are unchanged.

Build with ROS2 enabled and run the isolated targets:

```sh
ctest --test-dir <build> --output-on-failure \
  -R 'harness_execution_sampling(_ros)?_tests|harness_observation_tests'
```

Five policy check groups cover activity gating, coverage/clocks, durable budgets,
incident separation/unlabeled status, and policy/one-owner behavior. The offline
MCAP test checks both cameras and joint states, unchanged acquisition stamps,
sparse commands, incomplete coverage and byte limits. It does not start a ROS
node, policy, cloud upload or robot motion. Production upload must still be
observed after the operator deploys; passing offline checks is not live admission.

Verified on 2026-11-21 in the isolated Linux laptop cache build: ROS-enabled CLI
compiled; all three CTest targets passed (42.44 seconds total), including five
sampler groups and the offline MCAP checks. No binaries were installed, no service
was restarted and no real recording was uploaded during verification.


## Upload efficiency follow-up

The uploader reuses separate per-thread API/object HTTP sessions, retaining
connection/DNS/TLS caches while resetting request options between calls. It
requests CPU niceness 10 so hashing/export/TLS receive lower scheduling priority
than the observer under contention; this is best effort, not a resource guarantee.
Joint and command activity checks reuse the values already decoded for input
validation rather than deserializing the message a second time.

Fresh transfers use grant → direct PUT → complete, without a speculative complete
request first. Retried transfers probe complete to reconcile bytes whose prior
acknowledgement was lost. Persistent per-recording retry deadlines back off from
5 to 300 seconds; submitted samples are skipped before manifest parsing and
hashing. The ordinary sample queue is scanned once per distinct binding per
uploader polling pass, rather than once for every historical session. All file
hash and cloud verification checks remain in place.

There is no live ROS stream sent to Supabase or Modal. Selected exact MCAP files
go directly from edge to private R2 over presigned HTTPS PUT URLs. The API handles
authentication, grants and completion metadata; the existing cloud worker verifies
and indexes the object, and Supabase holds the catalog. JPEG/PNG messages are
stored as received; no additional camera re-encoding was added. Compression
options for raw image topics and resumable multipart upload need measurement
before changing their defaults. Current ordinary samples are capped at 64 MiB.

MCAP export and HTTP share one supervised uploader process but now run on
independent workers. This addresses network-induced pre-roll expiry; real
throughput and observer latency have not been benchmarked.

The localhost HTTP test uses synthetic bytes/credentials and checks fresh request
order, durable retry suppression, completion-only recovery after an interrupted
acknowledgement, submitted-file skipping and connection reuse. Run it alongside
the sampler checks using CTest target `harness_recording_upload_http_tests`.

## Upload efficiency follow-up

The uploader retains separate reusable HTTP sessions for API and R2 requests,
resetting per-request options while keeping connection/DNS/TLS session caches.
Fresh uploads request a grant directly, PUT once, then complete; only retries
probe completion first to reconcile a possibly successful earlier PUT.

Incident and sample retries persist their attempt counters and wall-clock retry
schedule (5, 10, 20, 40, 80, 160, then at most 300 seconds). A failed sample does
not prevent other samples from being attempted. Completed manifests are skipped
before parsing/hashing, sample queues are scanned at most once per successful
binding per polling pass, and already-retired pre-roll avoids repeated rewrites.
The uploader requests lower CPU scheduling priority (nice 10); this is not a
network bandwidth cap. Joint/command activity reuses validation's decoded values
instead of decoding each ROS message a second time.

The localhost HTTP fixture uses synthetic bytes and dummy credentials only. It
checks the first-attempt request sequence, lost-completion reconciliation without
a second PUT, persisted backoff, completed skips, and two reused HTTP connections
(one API, one object transfer). It does not benchmark the real uplink or cloud.
Zenoh is not introduced: ROS transport and immutable MCAP transfer remain separate
layers. Local export scheduling is now independent of PUT progress; resumable
multipart upload remains future work.

Efficiency validation on 2026-11-21: the ROS-enabled CLI compiled; all four
observation/sampler/MCAP/HTTP CTest targets passed (36.30 seconds). After the final
queue-scan guard adjustment, the CLI and HTTP fixture rebuilt and the HTTP test
passed again (1.95 seconds). Tests ran in the isolated Linux cache; no installed
service, real recording, cloud account or robot execution was changed.


## Local preservation and completed queue cleanup (2026-11-24)

The supervised uploader starts a local-only sample worker before draining HTTP
transfers. That worker does not call the backend or wait for incident uploads.
`ready` in observer state means the interval ended, not that an MCAP is durable.
A sample manifest plus MCAP means locally saved; `.submitted.json` means accepted
for asynchronous cloud verification, not already verified. `.rejected.json`
contains a terminal local-export reason. Gap checks and byte limits are unchanged.

On the first visit to each session after uploader startup, completed transfer
journal entries are compacted to `{"status":"submitted"}` tombstones. The full
original journal is archived under a content-hashed `.before-compaction-*.json`
name before replacement. Pending/retrying entries remain unchanged, acknowledgements
are never deleted, and subsequent runs cannot reupload completed recordings.
No recording artifacts, workflow candidates or cloud records are deleted. This
cleanup runs with the updated uploader, not as a competing editor of live journals.

Validation on 2026-11-24: the ROS-enabled CLI rebuilt in the isolated Linux cache;
all four observation/sampler/offline-MCAP/loopback-HTTP targets passed (42.63 s).
The HTTP mock withheld a grant response until the independent local worker
processed a newly appearing window. Offline MCAP checks confirmed durable export
with unvalidated-alert overlap metadata, continued exclusion of operator incidents,
and unchanged coverage/byte guards. Queue tests checked archived originals,
preserved completion tombstones and retries, idempotent compaction, and retirement
waiting for local export. No installed service or cloud data was changed.
