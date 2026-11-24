# Recovery adapters: development targets and test gates

Date: 22 November 2026.

The first targets are **SO-101 and AgileX PiPER**. WidowX is outside this iteration
until an exact model is selected. See the machine-readable
[development target catalog](../observation/recovery/targets.json).
These are development targets; neither is declared commissioned by native tests.
No deployment is performed by the adapter tests or test runner.

## Implementation

`AdapterRecoveryLibraryPort` connects the existing library executor to concrete
dispatch implementations. It accepts a `RecoveryActionTransport`, an explicit
workcell/client `RecoveryEnvironment`, and a trusted installed-implementation hash
registry. A recipe's declared implementation hash must match that registry.
The host must derive registry hashes from verified installed adapter artifacts;
copying hashes from an untrusted recipe does not establish implementation identity.

The port validates the `recovery_adapters` block inside the approved robot binding.
Robot names only validate target identity and arm axis count. They never select
topics, controller IDs, limits, calibration or checkpoints. The observation
binding remains discovered from the ROS graph; no observation/critic preset is
automatically installed.

| Target | Arm vector | End effector | Integration requirements |
| --- | --- | --- | --- |
| SO-101 | Five arm axes, explicitly ordered | Separate tool resource | Calibrated radians for the arm; a gap adapter for servo-driven grippers; exclusive recovery controller and VLA command path. |
| PiPER | Six arm axes, explicitly ordered | Separate tool resource | Pin firmware, description and ROS driver together; validate direct CAN/ROS command exclusion; normalize the gripper interface. |

Observed JointState messages may also contain gripper or unrelated joints.
`recovery_adapters.arm.joint_order` selects the exact controlled arm subset; the
gripper must not be appended to a generic six-element arm vector. Names and limits
must come from the selected robot description/calibration. Test values in
`recovery_adapters_test.cpp` are synthetic and must never be installed as robot
limits or physical checkpoints.

For PiPER, AgileX's ROS 2 MoveIt configuration documents a separate interpolated
command path to the hardware driver. Integrating MoveIt does not by itself prove
that every hardware command source is gated. The workcell integration must prove
exclusive ownership through the actual driver topology.
[AgileX MoveIt integration](https://github.com/agilexrobotics/agx_arm_ros/blob/ros2/src/agx_arm_moveit/README_EN.md).

## Bound configuration

`recovery_adapters` has `schema_version: 1`, `robot_family`, `robot_model` and
`arm`, plus optional `moveit`, `tool` and `nav2` blocks. Supported family/model pairs
are `so101/so101` and `agilex_piper/piper`. Unknown fields are rejected.

- `arm`: `joint_order`, `resource`, `units: rad`, finite `lower`, `upper`,
  `velocity`, `acceleration` vectors; positive `start_tolerance`, `goal_tolerance`,
  `stopped_velocity`; named joint-vector `checkpoints`; and `trajectory_action`
  namespaced by the approved recovery controller.
- `moveit`: `action`, `group`, `pipeline`, `planner`, positive `planning_time_s`,
  and `velocity_scale`/`acceleration_scale` in `(0,1]`. A zero scale is rejected
  because MoveIt can replace an invalid scale with its default maximum.
- `tool`: `action`, separate `resource`, `units: m`, `min_gap`, `max_gap`,
  `max_effort` in newtons and gap `tolerance` in metres. A servo angle, finger joint
  displacement or vendor integer is not automatically a full gripper gap.
- `nav2`: `action`, separate `resource`, `frame`, explicit qualified
  `behavior_tree` and named `checkpoints` as `[x,y,z,qx,qy,qz,qw]` with a unit
  quaternion. The workcell must validate the server-side behavior-tree artifact
  and its bounded behavior; an arbitrary tree with hidden task/recovery retries
  cannot be substituted for the approved one.

`RecoveryEnvironment::configure()` must additionally resolve every physical
predicate and checkpoint, validate resource ownership/gating, calibration, scene
and payload synchronization with MoveIt/Nav2, client protocol and any remote
artifacts. It must reject unsupported vendor drivers or direct-command bypasses.
No concrete environment for a physical workcell is provided in this change.

The environment supplies fresh measured arm positions/velocities, epoch/receipt
time, scene revision, disengagement and payload-model state. It must map measured
values into the declared arm order. A scene revision must change whenever a
collision-relevant object, payload, frame or other planning assumption changes;
it must correspond to the planner's actual synchronized scene, not an arbitrary
local counter. The generic port cannot establish that synchronization itself.

## Action behavior

### MoveIt joint checkpoint

`retreat_checkpoint` with capability `moveit_joint_checkpoint_v1` takes only an
approved `checkpoint` name. It checks fresh, stopped, disengaged arm state with
the payload represented in the planning scene. It requests MoveGroup **plan only**
with one planning attempt, explicit start/goal/scales and no look-around or
automatic replanning. MoveIt has no authority to execute this request.

On a successful plan result, the port checks exact joint order, finite point
dimensions, increasing time, start/goal tolerances, joint bounds, configured
velocity/acceleration values, segment average speed, stopped start/end velocities,
and remaining stage duration. It rechecks measured arm state and scene revision.
Only then does it send `FollowJointTrajectory` to the bound recovery controller.

These checks are an additional envelope check, not an independent geometric or
continuous-time collision proof. The qualified MoveIt pipeline and controller
remain responsible for collision validation, time parameterization, interpolation
and tracking. Changed or insufficient scene/payload evidence prevents execution.

### Tool gap

`reset_end_effector` with `gripper_gap_v1` accepts only
`operation: set_gap`, `gap_m` and `max_effort_n` within the approved bounds.
It uses `control_msgs/action/GripperCommand`. The environment's entry predicate
must establish whether releasing or retaining the payload is permitted.
Success requires a reached goal, no stall, measured gap within tolerance and
finite measured effort within the requested limit. Stage-level measured completion
predicates and handback readiness remain mandatory afterward.

This implements a normalized gripper action client. It does not invent a gap
conversion for either stock driver or implement arbitrary spindle/vacuum resets.
Those need their own qualified device adapter. `unload_disengage` is rejected;
there is no generic force-controlled or fastening reversal procedure.

### Nav2 checkpoint

`retreat_checkpoint` with `nav2_pose_checkpoint_v1` takes only an approved pose
checkpoint. It sends `NavigateToPose` with the configured frame and behavior tree
inside the independently owned base command path. Action completion still needs
the environment's measured pose/clearance predicate. Fixed-arm bindings have no
implicit base capability. This is tested as an optional synthetic base capability,
not a mobile-manipulation qualification for either first-target arm.

### Pause, cancellation and handback

`pause_reobserve` with `pause_condition_v1` accepts an empty parameter object.
It preserves the already established quiescent hold; it issues no implicit motion
or tool reset. The executor's completion predicate supplies the awaited condition.

All resources used anywhere in the recipe must be exclusively owned before any
stage starts and throughout polling/handback. Losing the tool or base resource is
not covered by retaining arm ownership. Action success never releases the lock.
The existing physical/client readiness transaction still gates handback.

Only one transport operation can be in flight. Cancellation is not terminal until
goal rejection or a terminal action result arrives. A goal accepted after a cancel
request is immediately cancelled, and a late success cannot start the next stage.
Unknown transport state remains nonquiescent. The current coordinator has a
single boolean fallback acknowledgement; pending cancellation therefore produces
an unconfirmed fallback rather than a false confirmed hold. A future asynchronous
fallback monitor can reconcile that state after cancellation actually completes.

## Tests before deployment

Native tests run without ROS and exercise concrete dispatch through a controlled
test transport for both arm configurations. They cover valid composition and
handback, limits, wrong joints/units, excessive duration, changing scene, unknown
payload/engagement, missing authority, invalid tool results, unsupported capability,
Nav2 configuration and late cancellation results.

```sh
cmake -S edge -B /tmp/rearguard-adapters-build \
  -DHARNESS_WITH_ROS2=OFF -DHARNESS_BUILD_TESTS=ON
cmake --build /tmp/rearguard-adapters-build --parallel 4
ctest --test-dir /tmp/rearguard-adapters-build --output-on-failure
```

The optional `RosRecoveryActionTransport` maps these requests to typed ROS 2
actions on a private executor thread. It is enabled only by
`HARNESS_WITH_RECOVERY_ROS=ON` together with `HARNESS_WITH_ROS2=ON`, adding
`moveit_msgs` and `nav2_msgs` build/runtime dependencies. It does not automatically
start MoveIt/Nav2 servers, install packages or select the library in the observer.

On a Jazzy development host, install the normal edge development dependencies and
`ros-jazzy-moveit-msgs`/`ros-jazzy-nav2-msgs`, then run:

```sh
bash edge/scripts/test_recovery_adapters_ros.sh /tmp/rearguard-recovery-ros
```

The runner builds the library, adapter and ROS mock-server tests. It uses localhost
on ROS domain 103 (override `REARGUARD_RECOVERY_TEST_DOMAIN_ID` if needed) and
test-only action endpoints. The ROS test exercises real DDS goal payloads, results,
domain errors, goal rejection and cancellation before delayed acceptance. It uses
mock action servers, not real MoveIt planners, Nav2 controllers or physical arms.
Its result must be recorded separately from the native dispatch tests.

Deployment remains gated on passing these tests, implementing the physical
environment and gated client protocol, then validating the exact driver and
controller topology in simulation. Physical commissioning must precede autonomous
operation, especially payload retention, contact and independently active tools.
The legacy observer default, critic inference, cache, sampling and recording
pipelines remain in place.

Validation recorded on 22 November 2026:

- Strict native macOS build passed all 18 CTest suites, including 296 concrete
  adapter checks across SO-101 and PiPER and 189 library/coordinator checks.
- The recording/upload regression passed with localhost socket permission.
- The ROS test runner passes shell syntax validation and refuses to run without
  Jazzy. The optional ROS transport and mock-server test have **not been compiled
  or executed here**: Jazzy is absent and the installed Docker application cannot
  start. The ROS build/protocol gate remains open; native results do not close it.
- No simulation or physical commissioning result is claimed. Nothing was deployed.

## Protocol references

- [MoveGroup action](https://github.com/moveit/moveit_msgs/blob/ros2/action/MoveGroup.action)
- [MoveIt planning options](https://github.com/moveit/moveit_msgs/blob/ros2/msg/PlanningOptions.msg)
- [MoveIt motion request](https://github.com/moveit/moveit_msgs/blob/ros2/msg/MotionPlanRequest.msg)
- [FollowJointTrajectory](https://github.com/ros-controls/control_msgs/blob/jazzy/control_msgs/action/FollowJointTrajectory.action)
- [GripperCommand](https://github.com/ros-controls/control_msgs/blob/jazzy/control_msgs/action/GripperCommand.action)
- [NavigateToPose](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_msgs/action/NavigateToPose.action)
