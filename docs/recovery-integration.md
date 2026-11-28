# Native recovery integration

The ROS observer constructs `make_ros_recovery_executor`. An approved artifact
with `kind: recovery_action_library` selects the library; an existing artifact
with `ros2_policy` and no `kind` selects the legacy trajectory executor. Unknown
formats and refused library recipes fail preparation. The existing approval,
artifact integrity, controller switching and fallback boundaries remain in force.

## Binding the workcell

The approved observation binding includes `recovery_adapters`, described in
[recovery-adapters.md](recovery-adapters.md), plus `recovery_environment`:

| Field | Meaning |
| --- | --- |
| `schema_version` | `1` |
| `provider_id` | Exact configured workcell facts/client provider identity |
| `state_topic`, `request_topic`, `ack_topic` | Three distinct `std_msgs/msg/String` JSON channels under `/rearguard/` |
| `predicate_ids` | Explicitly supplied stage, physical readiness and incident rearm predicates |
| `resources` | Map from resource ID to `controller_id` and nonempty `claimed_interfaces` |
| `command_lower`, `command_upper` | Finite position command limits in the full binding's joint order and driver units |
| `max_start_delta` | Positive allowed deviation from measured nominal positions, at most `0.25` in those units |

The native channel subscribes to the binding's single `JointState` source. It
requires measured position and velocity for the controlled joints, maps the arm
subset explicitly, and supplies arm/nominal positions and velocities locally.
Messages cannot override those measurements. Missing/stale joints or moving arm
state prevent a quiescent handback. Resource ownership is queried from the actual
controller-manager list service, including claimed interfaces and competing active
controllers; a provider's ownership assertion is insufficient.

The workcell state envelope contains `provider_id`, `binding_hash`, `epoch`,
`t_ns`, `state_revision`, `scene_revision`, `quiescent`, `disengaged`,
`payload_modelled`, and `predicates`. Each predicate has boolean `observable` and
`satisfied` fields. It must describe actual measured facts and the planner's
synchronized scene, with its source ROS time and current observation epoch.
Unobservable facts stay unobservable. The channel announces `observation_context`
on the request channel once per second
(and on epoch changes), so the provider can bind facts to the current observation
epoch and binding rather than guessing them. It records receipt monotonic time
locally. Both clocks must be fresh; future samples wait for a later tick. One
causal state/ack snapshot is used for each complete recovery tick.

The native integration supplies dispatch and protocol enforcement. The workcell
provider must connect its scene, contact, payload and device-specific facts to
these channels. It must not publish unconditional readiness values. Installing
MoveIt/Nav2 message packages does not start a planning server or qualify a driver.

## Policy command gate

[recovery_client.hpp](../include/harness/recovery_client.hpp) provides the policy
side gate, included in the release's public headers. Embed it at the sole nominal
command output, after any policy action queue. Every in-flight policy result
carries the `command_epoch()` captured when that computation began.

1. On a new `prepare` request, `handle()` drops its pending command, blocks output,
   changes the command epoch and returns true. Cancel pending policy work, reset
   its execution context and reobserve the reported post-recovery state.
2. Supply the first new candidate with `stage(epoch, command, observation_ns)`.
   It rejects results from old epochs or observations before preparation. Publish
   `acknowledgement(now_ns, physical_epoch)` on `ack_topic`; keep acknowledgements
   fresh while waiting. The native side checks the exact transaction, current
   physical revision, joint count, finite values, limits and start compatibility.
   Before staging, the workcell integration must also clear/prepare the inactive
   nominal controller's internal command cache. Its activation must retain a
   measured hold or the prepared candidate until the gate opens. The policy gate
   cannot clear a downstream controller's private queue; a controller that replays
   an old setpoint on activation is not qualified for this handback protocol.
3. `commit` retains the candidate behind the gate. It does not allow output.
4. Only `authority_transferred`, sent after the coordinator confirms nominal
   ownership, releases that candidate through `take()`. Old policy generations
   remain invalid; duplicate transfers cannot replay the first command.
5. `cancel` drops pending work and leaves the gate blocked. Neither a timeout nor
   a late transfer acknowledgement reopens it. Operator intervention is required.

Protocol messages carry `schema_version: 1`, `binding_hash`, `kind`, and the exact
`transaction` received during preparation. The transaction contains episode,
deployment and approval identities, physical state revision, observation epoch,
preparation time, transaction ID and new command epoch. Missing protocol consumers
or acknowledgements prevent automatic handback. An unmodified policy publishing
around this gate is not a qualified integration.

## Qualified contact exit

`unload_disengage` uses `qualified_contact_exit_v1`, with parameters `procedure`,
`max_force_n`, and `max_distance_m`. `recovery_adapters.contact` names its arm
resource and maps procedure names to an exact `implementation_hash`, qualified
force/travel limits, and `timeout_s`. Native recipe limits cannot exceed them.

The provider advertises the exact procedure hashes in `contact_capabilities` and
fresh `contact_feedback: {t_ns, force_n}` before dispatch. Native code sends
`execute_contact_exit` with a unique operation ID, epoch, hash, bounds and timeout.
The provider executes its qualified force/contact-controlled procedure. This
message is not a generic MoveIt trajectory or a task retry.

While running, it publishes fresh force feedback and `contact_exit` containing
`request_id`, `t_ns`, `phase`, `force_n`, and `distance_m`. Completion additionally
requires `disengaged`, `settled`, `no_late_work`, and measured quiescence. Excess
force/travel, stale feedback, lost ownership or timeout prevents progression and
requests cancellation. A cancel request alone does not establish terminal state;
unacknowledged or late work keeps the hold unconfirmed. The provider must enforce
its own local force and timeout watchdog independently of observer communication.

## Binding an approved physical critic

Client learning publishes a shadow model. To use its judgments for enforcement,
the existing approved recovery bundle must include a `critic_model` JSON artifact:

```json
{
  "schema_version": 1,
  "kind": "physical_critic_reference",
  "model_id": "<selected immutable model>",
  "profile_id": "<this profile>",
  "binding_hash": "<approved binding digest>",
  "manifest_sha256": "<cached manifest file digest>",
  "proposition_id": "manipulation.grasp_not_established",
  "proposition_hash": "<digest of the exact model proposition object>",
  "exit_gate": {
    "kind": "proposition_clear",
    "proposition_id": "manipulation.grasp_not_established",
    "query_id": "<approved clear query>",
    "recovery_id": "<approved recovery>",
    "threshold": 0.9,
    "max_age_ms": 250,
    "persistence_ms": 500
  }
}
```

The threshold/timings above illustrate the schema, not validated settings. The
reference's exit gate must equal the approved runtime contract's exit gate.
Entry `evidence_class` must equal the exact proposition ID, with an explicitly
validated `confidence_min`. Runtime injects installed deployment/approval identity
only after verifying the bundle; profile selection cannot inject that authority.
The adapter rechecks model files, manifest, binding, profile and proposition.
Failed activation or deactivation clears this authorization.

Background model delivery is deferred while an approved recovery is installed,
enforcement is armed, or recovery/fallback authority is retained. The supervisor
requires a current profile-revision runtime receipt before restarting the observer;
an old pre-deactivation status cannot authorize a model switch. Status reports the
desired generation separately from the running generation and exposes
`update_deferred` while waiting. This does not delay evidence uploads or training.

Enforcement reads the latest causal judgment independently of incident rising
edges. The clear judgment is `1 - failure_score`; it is an uncalibrated model
score, and remains separate from measured physical/client readiness. It does not
judge task success. Motion completion and a cleared critic cannot bypass the
physical readiness, command gate and verified ownership transfer.

Use `rearguard workflow recovery-capabilities --store <profile-store>` to inspect
implementation identities derived from the installed executable. Releasing a new
binary changes these identities and requires matching recipe validation. Never
copy the synthetic limits/checkpoints from unit tests into a hardware binding.

## Validation and deployment

The software tests use mock controller services, workcell facts and policy output.
They verify the observer factory, causal snapshots, ownership queries, stage
execution and gated handback without a physical robot. The actual driver,
provider, policy gate, planner scene and contact procedure still require
configuration and commissioning for the chosen workcell. Model training alone
does not provide that evidence or silently activate a recovery.

All code changes ship through the normal edge release. The operator publishes
and installs that release; no direct laptop deployment is required.
