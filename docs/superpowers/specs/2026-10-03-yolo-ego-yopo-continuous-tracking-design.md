# YOLO–EGO–YOPO Continuous Target Tracking Design

## Goal

Make real-flight target tracking continuous and safe: improve long-range YOLO tracking and identity retention, feed smooth moving targets to EGO and YOPO at a common 10 Hz rate, keep EGO 1.2 m from a tracked target, align yaw only once per flight task, and reduce EGO map over-inflation without shrinking the collision radius below the 0.25 m physical aircraft radius.

## Scope

This change covers:

- `yolo_trt_detector` tracking lifecycle, reacquisition, diagnostics, and dynamic-target publication.
- EGO dynamic-target handling, non-blocking replanning, terminal velocity, trajectory-end hold, and task-level yaw alignment.
- YOPO dynamic-target updates and OFFBOARD lifecycle integration so the shared target stream does not reset its reference on every update.
- The `orin-lidar-01` real-flight launch and YAML profiles.
- EGO grid-map inflation geometry.

Static navigation goals published as `geometry_msgs/PoseStamped` on `/planning/goal` remain supported and retain zero terminal velocity. PX4/offboard control output rates remain unchanged.

## Design Choices

### Dynamic and static goals use separate semantics

- `/planning/goal` remains the static three-dimensional waypoint interface used by scripts and RViz.
- `/yolo_trt/tracked_target` remains the dynamic `nav_msgs/Odometry` interface. It carries filtered position, velocity, covariance, timestamp, and frame.
- `/yolo_trt/target_status` remains the dynamic target lifecycle interface with `LOST`, `TENTATIVE`, `CONFIRMED`, and `COASTING`.
- EGO and YOPO subscribe to both interfaces. Static goals start a conventional point-to-point task. Dynamic targets update the current tracking task without resetting its trajectory reference.
- A dynamic target is executable only in `CONFIRMED`, or in `COASTING` for the configured coast interval. `LOST` longer than 0.8 s causes one controlled brake-to-hover action and invalidates the dynamic task.

This avoids overloading `PoseStamped` with velocity or validity and prevents the EGO-specific 1.2 m standoff from changing script or YOPO waypoint semantics.

### Common planning update rate

- YOLO inference remains 10 Hz.
- YOLO dynamic target and planner-reference publication is limited to 10 Hz.
- EGO consumes at most one latest dynamic-target update every 0.1 s and replans at 10 Hz while valid updates are available.
- YOPO depth inference and dynamic-goal update processing run at 10 Hz.
- EGO trajectory command publication, YOPO position-command publication, and offboard FSM setpoint publication remain at their existing higher rates. Only target/replanning rates are synchronized to 10 Hz.

Callbacks use latest-value-wins storage. They must not call `ros::spinOnce()` or wait for an FSM transition inside a subscriber callback.

## YOLO Tracking

The real-flight profile uses:

```yaml
conf_threshold: 0.25
track_init_confidence: 0.45
track_update_confidence: 0.25
track_confirm_hits: 3
track_confirm_window: 5
track_init_gate_distance: 1.2
track_coast_publish_time: 0.50
track_lost_time: 0.80
track_reacquire_memory_time: 2.0
track_reacquire_max_distance: 1.5
track_max_measurement_jump: 1.0
planner_goal_publish_rate: 10.0
```

When a confirmed track is lost, the detector retains the last filtered state and timestamp for 2.0 s. During that interval, a candidate may reacquire the old identity only if its three-dimensional world position is within 1.5 m of the predicted position and it passes the normal depth-quality checks. A single measurement jump greater than 1.0 m is rejected unless a new-track confirmation sequence is completed after the memory expires.

Lowering the initialization confidence therefore does not permit immediate switching to another planter, bench, or visually similar object. A new identity still needs three consistent hits in five frames.

The node records throttled diagnostic counters for rejection by detector confidence, insufficient depth samples, excessive depth MAD, missing synchronized odometry, association gating, and jump gating.

## EGO Dynamic Tracking

EGO stores the latest valid dynamic target under a mutex. Its 100 Hz FSM timer checks a 10 Hz rate limiter and processes at most the newest update. It does not block the ROS callback queue.

For a target position `p_t`, aircraft position `p_d`, and horizontal direction `u_xy`, the EGO tracking reference is:

```text
p_goal.xy = p_t.xy - 1.2 * u_xy
```

The default real-flight dynamic-target altitude mode is `hold_current`: the altitude captured when dynamic tracking starts is retained. Static `/planning/goal` waypoints remain fully three-dimensional. A configurable `target_3d` mode remains available for later aerial-target tests, with vertical slew and acceleration limits.

The target velocity from `nav_msgs/Odometry` becomes the global trajectory terminal velocity after clamping to EGO's configured velocity limits. In `hold_current` mode its vertical component is zero. The standoff reference is also velocity- and acceleration-limited before being passed to the planner.

Each valid dynamic update refreshes the global endpoint and requests a replan from the current trajectory state. It must not reset to raw odometry, clear a valid command stream, or wait inside the subscriber callback. If several updates arrive before the next 0.1 s planning slot, only the newest is used.

When a trajectory reaches its end, `traj_server` continuously publishes the terminal position with zero velocity and acceleration instead of returning before publication. This prevents `/position_cmd` timeout while hovering.

## YOPO Dynamic Tracking Compatibility

YOPO keeps `/planning/goal` for static waypoint tests and subscribes to `/yolo_trt/tracked_target` plus `/yolo_trt/target_status` for target tracking.

A dynamic update changes the goal and target velocity but does not clear `has_trajectory_`, reset `ctrl_time_`, reset the reference to odometry, or restart goal yaw alignment. First acquisition or a confirmed new identity initializes the task. Loss beyond 0.8 s generates a smooth hold from current odometry.

The real-flight profile enables:

```yaml
require_explicit_goal: true
pause_with_fsm: true
fsm_status_topic: /offboard_fsm/status
goal_yaw_alignment_enabled: false
depth_replan_rate: 10.0
```

This prevents the default `(0, 0, 1)` goal from becoming active, prevents reference advancement while OFFBOARD is unavailable, and removes repeated brake-turn cycles caused by moving YOLO goals.

Only one planner may publish the active `/position_cmd` during a flight. The launch procedure continues to require starting either EGO or YOPO, not both; this change does not add a planner command multiplexer.

## Task-Level Yaw Alignment

`traj_server` maintains a task-level `yaw_aligned_for_task` flag.

- `/traj_start_trigger` starts a new flight task and resets the flag.
- The first valid EGO trajectory after the trigger holds position and aligns yaw to the initial horizontal trajectory direction.
- Successful stable alignment sets the flag.
- Replanning, dynamic-target updates, trajectory completion, and terminal holding do not reset it.
- A landing lifecycle message resets the task state for the next flight.
- If the node starts without seeing a trigger, the first received trajectory is treated as the first task and is aligned once.

Normal in-flight yaw remains aligned with horizontal trajectory velocity and continues to obey the existing yaw-rate and yaw-acceleration limits.

## Obstacle Inflation

The aircraft's propeller-inclusive diameter is 0.50 m, so its physical horizontal radius is 0.25 m. The profile uses:

```yaml
grid_map:
  obstacles_inflation: 0.30
```

The current code expands every occupied point through a complete cube whose half-width is `ceil(inflation / resolution)`. At 0.20 m map resolution this turns 0.35 m into a much larger two-cell cube and makes diagonal inflation especially conservative.

Both point-cloud and depth-fusion inflation paths will instead generate offsets whose voxel-center Euclidean distance is no greater than the configured radius, apart from a numerical epsilon. With a 0.30 m configured radius and 0.20 m voxels, axial and horizontal-diagonal neighbor cells are included, two-cell axial offsets and three-dimensional cube corners are excluded. This preserves the 0.25 m physical radius plus 0.05 m static margin without recreating the old two-cell cube.

`optimization/dist0` remains 0.50 m for the first flight tests. It continues to provide trajectory optimization clearance and is not silently reduced together with occupancy inflation. Collision emergency handling remains enabled.

## Concurrency and Failure Handling

- Subscriber callbacks only validate and store data; planning work stays in planner timers.
- Dynamic target timestamps must be monotonic and no older than 0.8 s.
- Non-finite position, velocity, covariance, or frame-inconsistent targets are rejected.
- A target loss produces exactly one brake/hover request per loss event, not one request per timer tick.
- A reacquired target must complete confirmation before motion resumes.
- EGO and YOPO preserve the last safe hover reference whenever dynamic input is invalid.

## Testing

Tests are added before production changes and must demonstrate the prior failure.

YOLO tests cover:

- a 0.45-confidence candidate requiring three-in-five confirmation;
- successful near-prediction reacquisition;
- rejection of a 1.0 m-plus jump during identity memory;
- acceptance of a new identity only after memory expiry and confirmation;
- rate limiting to 10 Hz.

EGO and trajectory-server tests cover:

- latest-value-wins dynamic updates without callback blocking;
- 1.2 m horizontal standoff;
- terminal velocity clamping and altitude hold;
- one yaw alignment per `/traj_start_trigger` task;
- no repeated alignment after a short trajectory finishes;
- continuous terminal hover publication.

YOPO tests cover:

- dynamic target updates preserving the active reference and trajectory;
- LOST causing one smooth hold;
- FSM interruption pausing and resynchronizing control;
- explicit-goal enforcement.

Grid-map tests cover:

- inclusion of offsets inside 0.30 m;
- exclusion of cube corners outside 0.30 m;
- identical Euclidean behavior for cloud and depth inflation helpers.

The affected ROS packages must build successfully, unit tests must pass, and launch files must resolve all parameters. Ground testing verifies topic types, 10 Hz target/replanning rates, single yaw alignment, and single active `/position_cmd` publisher. The first flight remains limited to 1 m/s or less until logs show no repeated BRAKE/HOVER transitions or target identity jumps.

## Non-Goals

- No disabling of EGO emergency collision handling.
- No increase of offboard timeouts to hide missing planner commands.
- No change to PX4 attitude, rate, or position-controller gains.
- No increase of real-flight maximum speed in this change.
- No new planner command multiplexer; planner exclusivity remains an operational requirement.
