# PX4 native OFFBOARD FSM

This package replaces the legacy PX4Ctrl position/thrust controller. PX4 now
owns state estimation, position/velocity control, attitude control and motor
control. The ROS node only manages OFFBOARD lifecycle and sends local
position/velocity/acceleration references.

The normal state path is:

```text
WAIT_FCU -> PRESTREAM -> GROUND -> ARMING -> TAKEOFF -> HOVER
                                                        |
                                      planner ready -> EXTERNAL
                                                        |
                              timeout/hover request -> BRAKE -> HOVER

TAKEOFF / HOVER / EXTERNAL / BRAKE -> LANDING -> DISARMING -> GROUND
```

## Safety model

- The node continuously pre-streams the current PX4 local pose before OFFBOARD.
- Entering OFFBOARD never arms the vehicle.
- Arming only occurs after an explicit TAKEOFF event in the GROUND state.
- Planner commands are ignored in GROUND, ARMING, TAKEOFF and LANDING.
- HOVER requires new planner messages received after HOVER entry.
- Planner timeout enters BRAKE and then captures a new hover point.
- Planner position, velocity, acceleration and yaw jumps are bounded; a
  configurable ground-relative geofence rejects out-of-range references.
- Critical MAVROS or odometry loss stops setpoint publication so PX4's
  configured OFFBOARD-loss action can take over.
- Manual exit from OFFBOARD always has priority and is never overridden.
- Simultaneous events have deterministic priority: LAND, then ABORT/HOVER,
  then TAKEOFF.
- RC commands are disabled by default. When enabled, one `takeoff_land`
  switch uses hysteresis, debounce and release-before-trigger logic. Its
  rising edge means TAKEOFF only in GROUND, means LAND in ARMING or a flying
  state, and is ignored in PRESTREAM, LANDING, DISARMING and FAILSAFE.

`node/request_offboard_automatically` defaults to `false`. With this default,
the pilot must select OFFBOARD before TAKEOFF is accepted. If explicitly
enabled, a TAKEOFF event requests OFFBOARD repeatedly, but still does not arm
until PX4 reports OFFBOARD and the vehicle reports ON_GROUND.

## Commands

```bash
# Take off to the configured relative height
rostopic pub -1 /offboard_fsm/command px4ctrl/FsmCommand \
  "{command: 1, takeoff_height: 0.0, request_id: 1}"

# Take off to 1.0 m relative to the current ground pose
rostopic pub -1 /offboard_fsm/command px4ctrl/FsmCommand \
  "{command: 1, takeoff_height: 1.0, request_id: 2}"

# Land
rostopic pub -1 /offboard_fsm/command px4ctrl/FsmCommand \
  "{command: 2, takeoff_height: 0.0, request_id: 3}"

# Leave EXTERNAL control and brake to a hover
rostopic pub -1 /offboard_fsm/command px4ctrl/FsmCommand \
  "{command: 3, takeoff_height: 0.0, request_id: 4}"
```

Status is published on `/offboard_fsm/status`. The current reference is
published on `/offboard_fsm/reference`, and health is included in
`/diagnostics`.

The planner input is accepted only while hovering, only after the configured
number of consecutive fresh commands, and only if the command satisfies the
configured jump, velocity, acceleration and geofence limits. EGO-Planner's
`/planning/Emergency_hover` also forces the node out of EXTERNAL control.

## MAVLink message rates

`message_intervals/messages` is a variable-length YAML list. Every enabled
entry is applied using `mavros_msgs/MessageInterval` after MAVROS connects.
The list may be edited without recompiling the package. `enabled: false`
skips an entry; `required: true` raises an ERROR on `/diagnostics` if all
retries fail. The setting changes only the PX4-to-MAVROS stream request and
does not change the serial link baud rate.

Before a propeller-on test, configure PX4's data-link-loss and OFFBOARD-loss
actions, verify `/mavros/extended_state` and `/mavros/local_position/odom` are
fresh, confirm the configured RC channels on `/mavros/rc/in`, and first run a
props-off state-transition test. This node intentionally does not overwrite
PX4 failsafe parameters.

Legacy configuration files intentionally fail at startup because every new
configuration must contain `schema_version: 1`.
