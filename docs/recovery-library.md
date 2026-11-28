# Recovery library: native executor and handback contract

Updated: 22 November 2026.

This is the first implementation layer of
[the recovery library strategy](../../strategy/recovery-action-library_v2.md).
It adds the portable recipe compiler/executor and integrates asynchronous handback
preparation into the existing runtime and authority coordinator. Concrete
MoveIt/Nav2/gripper dispatch and an opt-in ROS action transport are now implemented;
see [adapter development and tests](recovery-adapters.md). The live observer
selects the library for explicit library artifacts and the trajectory executor
for legacy policies. The native ROS environment and packaged policy gate are
described in [recovery-integration.md](recovery-integration.md). Exact workcell
bindings, validation and commissioning remain required before enforcement.

## Existing components

Critic inference, proposition compilation, model cache, exit scoring, recording,
upload and execution sampling keep their existing contracts. The only critic
source edits make existing numeric conversions explicit for the strict native
build; float subtraction still happens before conversion to double.

`Runtime` continues to validate the installed deployment, artifact hashes,
approved proposition attachment and binding. It still requires both action
completion and the existing critic exit gate before preparing handback. The new
executor does not replace the critic exit gate with a successful motion result.

Existing `ControllerAdapter` implementations inherit no-op observation and
immediate handback preparation hooks and retain their existing `resume()` checks.
The library executor overrides these hooks and refuses `resume()` until physical
and client readiness have passed. Thus existing deployments retain their current
behavior; the stronger protocol applies when using the library executor.

The current store still uses the `recovery_model` artifact slot and legacy bundle
submission lifecycle. A JSON recipe can occupy that content-addressed slot when
used with the library executor. A separate recipe-authoring/submission lifecycle
that avoids the legacy generation job is still required. This implementation
does not migrate stored profiles or change server/frontend APIs.

## Recipe compilation

`compile_recovery_recipe()` accepts schema version 1 and kind
`recovery_action_library`. It validates exact binding and controller identities,
unique stage IDs, allowlisted action names, implementation SHA-256 syntax, finite
positive deadlines, maximum stage count, bounded observation age and stability,
and a total duration budget within the approved runtime contract. Unknown fields
at recipe, stage or handback level are rejected. The runtime contract must specify
`retry_budget: 0`; the first library version has no implicit recipe retries.

Each stage has a typed action plus a capability-specific parameter object:

| Action name | Required integration responsibility |
| --- | --- |
| `pause_reobserve` | Preserve the approved hold/tool state and refresh observations. |
| `retreat_checkpoint` | Use the qualified MoveIt/Nav2 adapter and approved checkpoint; establish disengagement before free-space motion. |
| `reset_end_effector` | Execute an explicit supported operation with payload/support requirements and device acknowledgement. |
| `unload_disengage` | Execute a qualified contact exit using the required feedback and bounded robot/tool controls. |

Every stage also supplies `id`, `capability`, `implementation_hash`,
`entry_predicate`, `completion_predicate` and `timeout_ms`.
The top-level manifest supplies `id`, `version`, `binding_hash`, `controller_id`,
`rearm_predicate`, `stages` and `handback`. `handback` supplies
`readiness_predicate`, `max_age_ms`, `stability_ms` and `timeout_ms`.

The bound `RecoveryLibraryPort::configure()` must additionally validate every
capability, implementation digest, resource, parameter, frame/checkpoint,
predicate and client protocol. The generic compiler cannot infer device limits
or whether a connector can be withdrawn. An unsupported configuration must fail
before activation; it must never map to a generic trajectory or tool reset.
Physical commissioning and exact approved validation evidence remain deployment
requirements, not conclusions of schema validation.

## Execution under one authority acquisition

Construct `LibraryRecoveryController(port)`, wrap it in
`SwitchingRecoveryController(authority, executor)`, then pass that controller to
`Runtime`. `Runtime` supplies observation ticks and creates a unique recovery
episode on each approved trigger. Port methods execute on the observer worker;
asynchronous robot/client work belongs on the integration's own bounded worker.
They must enqueue/poll rather than block the realtime controller loop.

The coordinator acquires authority once. Each stage must pass fresh entry
evidence before starting. A lower-level success also needs fresh measured
completion evidence from the same clock epoch and no earlier than stage start.
Only then can the next stage start. A failed stage, unknown result, timeout or
clock reset cannot skip a stage or start a task retry.

The executor returns action success after every stage completes. This result
does not release authority. Runtime keeps polling the coordinator, including
after action success, so ownership loss during the exit/readiness wait is still
detected.

One execution is allowed until the configured rearm predicate has been observed
after confirmed handback for the configured stability interval. The integration
must define that predicate as an incident-cleared condition; it cannot simply
reuse a transient disappearance of the failure score. New recipe execution also
requires a new approved runtime trigger. This is not a persistent incident ledger
across process restart or explicit deployment reactivation; that lifecycle remains
to be implemented before claiming restart-resistant incident budgets.

## Readiness while the VLA remains locked

`prepare_handback()` returns `pending`, `ready` or `failed`. Pending leaves Runtime
recovering and the VLA excluded, under both recipe and runtime deadlines. There
is no blocking wait for the client.

Physical readiness requires fresh, observable, satisfied evidence from the
current epoch, no earlier than recipe completion, and a nonempty state revision.
It must remain stable for the configured interval. `quiescent()` additionally
asserts terminal/acknowledged-cancelled recovery goals and the required tool state
with no late recovery work. The named physical predicate must cover the approved
payload/support, engagement, device, clearance, velocity and reentry constraints.
Unknown evidence fails readiness. State revision changes restart stability.

After physical readiness, the executor generates a transaction containing:

- Episode, deployment and deployment content hash, plus runtime failure context.
- Unique transaction ID and command generation/epoch.
- Current state revision and observation clock epoch.
- `prepared_after_ns` in receipt ROS time.

`prepare_client(transaction, report)` must invalidate old nominal action chunks
and request policy preparation from fresh post-recovery state while actuation
remains excluded. A client acknowledgement has this envelope:

```json
{
  "transaction": {"...": "the exact complete supplied transaction"},
  "ready": true,
  "command": [0.1, 0.2]
}
```

The shown transaction is illustrative; the actual object must be echoed exactly.
The command is a finite, nonempty numeric vector in the adapter's declared
command contract. `validate_candidate()` must additionally verify units,
dimensions, bounds, current-state compatibility and generation identity. The
adapter must establish that the client generated this candidate from the
requested post-recovery state; receiving a command later is insufficient.
The acknowledgement is a trusted client-adapter result, not an unauthenticated
public ROS topic accepted by the generic executor.

Wrong-episode, stale-generation and mismatching state acknowledgements cannot
prepare handback. Missing client responses time out. Lost physical readiness
invalidates the transaction. A changed state requires stability and a new
transaction/command; the prior acknowledgement is insufficient.

`resume()` rechecks physical evidence, stability, quiescence and state revision,
then asks `commit_candidate()` to stage the accepted command behind the gate.
This method must neither release VLA exclusion nor publish around the gate.
The coordinator rechecks recovery ownership, performs the existing authority
release and verifies nominal ownership. Only then does `handback_confirmed()`
mark the executor's report as confirmed. The integration must ensure the staged
candidate is the command available at transfer and late recovery commands cannot
actuate. Multi-resource tool ownership and watchdog response require ROS testing.

On fallback, the coordinator attempts executor cancellation and authority hold,
including when cancellation throws or returns false. It reports confirmed hold
only if both acknowledge. Failed or partial release must enter this path; it
cannot be recorded as successful handback. The existing legacy trajectory
adapter's cancellation behavior is unchanged and is not upgraded to an
acknowledged multi-resource cancellation protocol by this change.

## Reporting and remaining integration

Executor telemetry records recipe identity/hash, stage outcomes, physical/client
readiness, handback transaction, optional measured tool/contact/payload context,
and confirmed authority handback. Runtime includes it in snapshots and the
existing enforcement outbox. Task outcome remains `unknown`; the client VLA owns
the next insertion, grasp or fastening attempt.

Before enabling this library in the live ROS observer:

1. Configure and qualify the native environment with the selected workcell's
   scene, payload and physical predicates, plus supported MoveIt/Nav2/tool/contact
   capabilities.
2. Connect the packaged policy gate at the sole nominal output, including policy
   queue invalidation/reset and ownership of independently active tools. Bind
   implementation versions and resource claims to the approved configuration.
3. Submit and review the exact recipe/model reference through the existing store
   approval flow. Executor selection is explicit; a refused library recipe never
   falls back to a legacy generated policy.
4. Run ROS fault integration, simulation and physical commissioning. Validate
   cancellation, late callbacks, partial switches, process death, watchdogs and
   the actual command available at authority transfer.

The native tests cover the portable executor/coordinator and Runtime readiness
wait, alongside the existing critic, sampling, recording/upload and workflow
regressions. They do not demonstrate live MoveIt/Nav2 execution, physical contact
recovery or a commissioned client VLA handshake.

Validation on 22 November 2026: a fresh macOS build with ROS 2, ONNX and TensorRT
disabled passed all 17 native CTest tests, including 189 recovery-library checks.
The recording/upload HTTP test required localhost socket permission outside the
sandbox. The build used the repository's normal warning-as-error settings.

## Explicit operator halt (28 November 2026)

The fifth action, `halt_until_operator`, is allowed only as the final stage. Its
entry and measured completion checks remain bounded. Once those pass, the
executor returns `operator_halt`, never ordinary success. Runtime invokes the
coordinator's robot-specific hold and records either `held` or
`fault_unconfirmed_hold`; it does not evaluate automatic handback for this result.
Neither elapsed time nor a later cleared critic signal releases this latch.
The concrete adapter capability is `operator_halt_v1` with empty parameters and
requires the bound environment's quiescent hold. This does not implement a remote
operator-release protocol or replace the independent robot watchdog.

Profile authoring exposes all five action identities as drafts. Selecting one
is not recipe compilation or activation. Existing exact binding, validation,
approval and commissioning requirements still apply, including to operator halt.
