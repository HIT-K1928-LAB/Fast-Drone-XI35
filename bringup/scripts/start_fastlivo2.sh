#!/usr/bin/env bash

set -eu

show_usage() {
    cat <<'EOF'
Usage: start_fastlivo2.sh [auto|sim|real]

Modes:
  auto  Use sim for FASTDRONE_PROFILE=pc_sim, otherwise use real.
  sim   Start FAST-LIVO2 with Gazebo sensor topics.
  real  Start the Mid-360 driver and RK3588 FAST-LIVO2 configuration.
EOF
}

MODE="${1:-auto}"
case "$MODE" in
    auto)
        if [ "${FASTDRONE_PROFILE:-}" = "pc_sim" ]; then
            MODE="sim"
        else
            MODE="real"
        fi
        ;;
    sim|simulation)
        MODE="sim"
        ;;
    real|hardware)
        MODE="real"
        ;;
    -h|--help)
        show_usage
        exit 0
        ;;
    *)
        show_usage >&2
        exit 64
        ;;
esac

RVIZ="${FASTLIVO2_RVIZ:-false}"
ROSLAUNCH="${FASTLIVO2_ROSLAUNCH:-roslaunch}"

if [ "$MODE" = "sim" ]; then
    sim_launch_args=(fast_livo mapping_mid360_sim.launch rviz:="$RVIZ")
    if [ -n "${FASTLIVO2_CONFIG_FILE:-}" ]; then
        sim_launch_args+=(config_file:="$FASTLIVO2_CONFIG_FILE")
    fi
    if [ -n "${FASTLIVO2_CAMERA_CONFIG_FILE:-}" ]; then
        sim_launch_args+=(camera_config_file:="$FASTLIVO2_CAMERA_CONFIG_FILE")
    fi
    if [ -n "${FASTLIVO2_LIDAR_TOPIC:-}" ]; then
        sim_launch_args+=(lidar_topic:="$FASTLIVO2_LIDAR_TOPIC")
    fi
    exec "$ROSLAUNCH" "${sim_launch_args[@]}"
fi

DRIVER_STARTUP_DELAY="${FASTLIVO2_DRIVER_STARTUP_DELAY:-2}"
driver_pid=""

cleanup_driver() {
    if [ -n "$driver_pid" ] && kill -0 "$driver_pid" 2>/dev/null; then
        kill "$driver_pid" 2>/dev/null || true
        wait "$driver_pid" 2>/dev/null || true
    fi
}

trap cleanup_driver EXIT
trap 'exit 130' INT TERM

"$ROSLAUNCH" fast_livo mid360_rk3588.launch &
driver_pid=$!
sleep "$DRIVER_STARTUP_DELAY"

if ! kill -0 "$driver_pid" 2>/dev/null; then
    wait "$driver_pid"
    echo "FAST-LIVO2 Mid-360 driver exited during startup." >&2
    exit 1
fi

"$ROSLAUNCH" fast_livo mapping_mid360_hikrobot_rk3588.launch rviz:="$RVIZ"
