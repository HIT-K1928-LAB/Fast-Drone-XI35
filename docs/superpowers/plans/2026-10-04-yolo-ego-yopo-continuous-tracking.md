# YOLO–EGO–YOPO Continuous Tracking Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deliver identity-safe 10 Hz YOLO target tracking, continuous 10 Hz EGO/YOPO replanning, a 1.2 m EGO standoff, one yaw alignment per flight task, and a true 0.30 m Euclidean EGO obstacle inflation radius.

**Architecture:** Keep static `PoseStamped` waypoints and dynamic `Odometry` targets as separate inputs. YOLO owns target identity and validity, EGO and YOPO own planner-specific continuous-reference behavior, and `traj_server` owns task-level yaw and high-rate terminal hold publication. Extract small pure policy helpers where needed so safety behavior can be tested without TensorRT, a camera, or a flying vehicle.

**Tech Stack:** ROS Noetic, catkin, C++14/C++17, Eigen, OpenCV Kalman filter, GoogleTest, YAML/roslaunch, TensorRT runtime.

**Spec:** `docs/superpowers/specs/2026-10-03-yolo-ego-yopo-continuous-tracking-design.md`

## Global Constraints

- YOLO target publication, EGO dynamic-target replanning, and YOPO depth replanning are all 10 Hz.
- High-rate `PositionCommand` and MAVROS setpoint publication rates do not change.
- EGO dynamic-target horizontal standoff is exactly 1.2 m; static `/planning/goal` waypoints are not offset.
- Real-flight dynamic-target altitude mode defaults to `hold_current`; static goals remain fully three-dimensional.
- Target loss after 0.8 s requests one controlled brake/hover action; collision emergency handling remains enabled.
- Yaw aligns once after `/traj_start_trigger` and does not realign for replans or short trajectory completion.
- Grid inflation uses a 0.30 m Euclidean voxel-center radius; `optimization/dist0` remains 0.50 m.
- Only one planner publishes the active `/position_cmd` during a flight.
- Preserve all pre-existing dirty-worktree changes and do not commit unrelated user modifications.

## Review Focus

- A distant low-confidence target must not switch identity to a nearby planter or bench after a short loss.
- A 10 Hz moving goal must not reset trajectory time, reference state, or yaw alignment on each message.
- A target becoming LOST while the aircraft is moving must generate one smooth stop, not stale pursuit or repeated emergency events.
- A completed short EGO trajectory must continue publishing a finite hover command without restarting yaw alignment.
- The 0.30 m inflation kernel must include required neighbor cells but exclude old cube corners and two-cell axial offsets.

---

## Preflight: Create an Isolated Implementation Workspace

The deployed repository contains many intentional uncommitted changes, including files touched by this feature. A clean worktree from `HEAD` alone would omit those changes.

- [ ] Create a remote `codex/continuous-tracking` worktree from the current `board` commit.
- [ ] Overlay only the current deployed versions of the affected YOLO, YOPO, EGO, profile, and test files into that worktree so the implementation starts from the actual flying configuration.
- [ ] Record `git diff --stat`, `git status --short`, and SHA-256 hashes of every overlaid file before editing.
- [ ] Run the existing affected unit tests and a package build in the isolated worktree.
- [ ] If a baseline test fails, stop and report it before changing production code.

## Task 1: Add Identity-Safe YOLO Reacquisition

**Files:**

- Create: `src/perception/yolo_trt_detector/include/yolo_trt_detector/track_reacquisition.h`
- Create: `src/perception/yolo_trt_detector/test/test_track_reacquisition.cpp`
- Modify: `src/perception/yolo_trt_detector/src/yolo_trt_node.cpp`
- Modify: `src/perception/yolo_trt_detector/CMakeLists.txt`

- [ ] Write tests that characterize the unsafe current behavior and assert the required replacement behavior: three hits in five confirm a 0.45-confidence new track; a candidate within 1.5 m reacquires retained identity; a jump greater than 1.0 m is rejected during the 2.0 s identity memory; a new identity can initialize only after memory expiry and normal confirmation.
- [ ] Run the new test target and confirm the old behavior fails the assertions.
- [ ] Implement a small pure reacquisition policy containing retained position, velocity, timestamp, memory expiry, distance gate, jump gate, and confirmation window.
- [ ] Integrate the policy into `resetTracker()`/`updateTracker()` without publishing tentative or rejected candidates.
- [ ] Preserve the last confirmed filter state for 2.0 s instead of erasing all identity state immediately on LOST.
- [ ] Run the YOLO unit tests and confirm the new tests and existing orientation/standoff tests pass.

## Task 2: Add YOLO Rejection Diagnostics and 10 Hz Profile

**Files:**

- Modify: `src/perception/yolo_trt_detector/src/yolo_trt_node.cpp`
- Modify: `src/perception/yolo_trt_detector/launch/yolo_trt_detector.launch`
- Modify: `bringup/profiles/orin-lidar-01/launch/lidar/yolo_detector.launch`
- Modify: `bringup/profiles/orin-lidar-01/tmux.sh`

- [ ] Write a failing test for a 10 Hz publication limiter using deterministic timestamps.
- [ ] Add counters for confidence rejection, insufficient depth samples, excessive MAD, odometry-time mismatch, association rejection, and jump rejection; emit throttled summaries without per-frame log spam.
- [ ] Apply the confirmed profile values: init confidence 0.45, gate 1.2 m, coast 0.50 s, lost 0.80 s, memory 2.0 s, reacquire distance 1.5 m, maximum jump 1.0 m, and publication 10 Hz.
- [ ] Keep `/yolo_trt/tracked_target` as the dynamic `Odometry` stream and `/yolo_trt/target_status` as lifecycle state. Do not apply EGO's standoff to this raw dynamic target.
- [ ] Extend the target rosbag pane to include `/planning/goal`, `/position_cmd`, and the planner status needed to correlate target and control behavior.
- [ ] Run YOLO tests and use `roslaunch --dump-params` to verify every profile value resolves as specified.

## Task 3: Implement EGO Dynamic-Target Policy and Non-Blocking 10 Hz Replanning

**Files:**

- Create: `src/planning/ego_planner/plan_manage/include/plan_manage/dynamic_target_policy.h`
- Create: `src/planning/ego_planner/plan_manage/test/dynamic_target_policy_test.cpp`
- Modify: `src/planning/ego_planner/plan_manage/include/plan_manage/ego_replan_fsm.h`
- Modify: `src/planning/ego_planner/plan_manage/src/ego_replan_fsm.cpp`
- Modify: `src/planning/ego_planner/plan_manage/CMakeLists.txt`
- Modify: `src/planning/ego_planner/plan_manage/launch/advanced_param.xml`
- Modify: `bringup/profiles/orin-lidar-01/config/lidar/ego_planner_lidar.yaml`
- Modify: `bringup/profiles/orin-lidar-01/launch/lidar/ego_planner_lidar.launch`

- [ ] Write failing pure-policy tests for 1.2 m horizontal standoff, captured-altitude hold, finite/clamped terminal velocity, monotonic timestamps, latest-value-wins replacement, 0.1 s replanning interval, and one loss event after 0.8 s.
- [ ] Run the policy test and verify failures correspond to missing dynamic-target behavior.
- [ ] Implement `DynamicTargetPolicy` with validation, standoff computation, reference velocity/acceleration limiting, target timeout, and a one-shot loss-event latch.
- [ ] Subscribe to `/yolo_trt/tracked_target` and `/yolo_trt/target_status` while retaining `/planning/goal` for static waypoints.
- [ ] Make callbacks validate/store only. Move target processing to the existing 100 Hz FSM timer with a 10 Hz rate limiter and latest-value-wins semantics.
- [ ] Replace the blocking `while (exec_state_ != EXEC_TRAJ) { ros::spinOnce(); ... }` path with an FSM event/request that never re-enters the callback queue.
- [ ] For dynamic targets, update the global endpoint and terminal velocity and replan from the current trajectory state. For static goals, preserve zero terminal velocity and existing 3D behavior.
- [ ] On one-shot dynamic-target loss, publish one controlled hover request and clear the dynamic target without disabling collision emergency behavior.
- [ ] Run EGO policy and existing planner tests.

## Task 4: Make `traj_server` Hold Continuously and Align Yaw Once Per Task

**Files:**

- Modify: `src/planning/ego_planner/plan_manage/src/traj_server.cpp`
- Modify: `src/planning/ego_planner/plan_manage/test/traj_server_yaw_test.cpp`
- Modify: `src/planning/ego_planner/plan_manage/launch/advanced_param.xml`
- Modify: `bringup/profiles/orin-lidar-01/launch/lidar/ego_planner_lidar.launch`

- [ ] Add failing tests showing that a second trajectory after completion currently re-enters yaw alignment and that terminal sampling currently exits before producing a publishable hold command.
- [ ] Add task lifecycle state reset by `/traj_start_trigger` and landing, with node-start fallback that treats the first trajectory as a task start.
- [ ] Mark yaw alignment complete only after the configured tolerance/stable-time condition succeeds. Do not clear the flag on replan or trajectory completion.
- [ ] Remove the terminal-path early return and publish the final position with zero velocity, acceleration, jerk, and yaw rate at the normal command rate.
- [ ] Preserve finite-yaw, yaw-rate, yaw-acceleration, and odometry-freshness protections.
- [ ] Run `traj_server_yaw_test` and confirm existing and new cases pass.

## Task 5: Make YOPO Dynamic Updates Continuous at 10 Hz

**Files:**

- Modify: `src/planning/yopo_planner/include/yopo_planner/yopo_planner.h`
- Modify: `src/planning/yopo_planner/src/yopo_planner.cpp`
- Modify: `src/planning/yopo_planner/src/yopo_node.cpp`
- Create: `src/planning/yopo_planner/test/test_dynamic_target_tracking.cpp`
- Modify: `src/planning/yopo_planner/CMakeLists.txt`
- Modify: `bringup/profiles/orin-lidar-01/config/lidar/yopo.yaml`

- [ ] Write failing tests proving a dynamic target update currently clears the active trajectory/reference and that repeated updates can restart goal yaw alignment.
- [ ] Add a dynamic-target update method that changes target position/velocity without resetting `has_trajectory_`, `ctrl_time_`, the continuous reference, or task-level yaw state.
- [ ] Subscribe to `/yolo_trt/tracked_target` and `/yolo_trt/target_status`; treat static `/planning/goal` as a new waypoint task and dynamic updates as rolling target state.
- [ ] Implement one smooth hold on dynamic target loss and resume only after confirmed reacquisition.
- [ ] Enable `require_explicit_goal`, `pause_with_fsm`, and 10 Hz depth replanning; disable per-goal yaw alignment in the real-flight profile.
- [ ] Verify control output remains at `ctrl_dt=0.02` (50 Hz) and only replanning is limited to 10 Hz.
- [ ] Run all YOPO tests, including terminal approach and legacy yaw helpers.

## Task 6: Replace Cubic Map Inflation with a 0.30 m Euclidean Kernel

**Files:**

- Create: `src/planning/ego_planner/plan_env/include/plan_env/inflation_kernel.h`
- Create: `src/planning/ego_planner/plan_env/test/inflation_kernel_test.cpp`
- Modify: `src/planning/ego_planner/plan_env/src/grid_map.cpp`
- Modify: `src/planning/ego_planner/plan_env/include/plan_env/grid_map.h`
- Modify: `src/planning/ego_planner/plan_env/CMakeLists.txt`
- Modify: `bringup/profiles/orin-lidar-01/config/lidar/ego_planner_lidar.yaml`

- [ ] Write a failing test against the current cube behavior: at 0.20 m resolution and 0.30 m radius, include axial and XY-diagonal one-cell offsets; exclude two-cell axial offsets and XYZ cube corners outside 0.30 m.
- [ ] Implement one reusable Euclidean offset generator using `offset.norm() * resolution <= radius + epsilon`.
- [ ] Use the same kernel in both occupancy-update and direct point-cloud inflation paths.
- [ ] Set `obstacles_inflation: 0.30` and keep `optimization/dist0: 0.50` unchanged.
- [ ] Run plan-environment tests and verify generated offsets are finite, unique, and symmetric.

## Task 7: Integration Configuration and Regression Verification

**Files:**

- Create: `bringup/profiles/orin-lidar-01/test/test_continuous_tracking_config.py`
- Review: `bringup/profiles/orin-lidar-01/tmux.sh`
- Review: all files listed in Tasks 1–6

- [ ] Add/extend configuration tests that assert YOLO, EGO, and YOPO target/replanning rates are all 10 Hz; EGO standoff is 1.2 m; inflation is 0.30 m; YOPO explicit-goal/FSM pause are enabled; and YOPO per-goal yaw alignment is disabled.
- [ ] Run affected package tests with testing enabled and inspect `catkin_test_results` for zero failures.
- [ ] Run `catkin build yolo_trt_detector yopo_planner plan_env ego_planner --no-status --summarize` in the isolated workspace.
- [ ] Run the repository's `./tools/make.sh all` as the full regression build and record any unrelated pre-existing failure separately.
- [ ] Inspect the complete diff against the preflight file hashes and confirm no unrelated dirty files changed.
- [ ] Transfer only the verified affected files back to the deployed worktree, preserving permissions and the user's other modifications.
- [ ] Rebuild the same four packages in the deployed workspace and rerun affected tests.
- [ ] With propellers removed or the vehicle disarmed, start FAST-LIVO, MAVROS, YOLO, exactly one planner, and offboard FSM. Verify topic types, exactly one `/position_cmd` publisher, 10 Hz target/replan diagnostics, 50 Hz command output, one yaw alignment log per trigger, and a single hold transition on target loss.
- [ ] Do not conduct an autonomous flight. Hand off a first-flight checklist limited to 1 m/s or less and the rosbag topics needed to verify identity, replanning, yaw, collision, and FSM behavior.

## Implementation Commit Strategy

The isolated worktree receives one focused commit after each task passes its tests. Deployment back to the dirty real-flight worktree is file-scoped and is not committed automatically, because committing an already-dirty overlapping file would also capture earlier user changes. The final handoff lists isolated commits, deployed file hashes, test results, and remaining pre-existing dirty files.
