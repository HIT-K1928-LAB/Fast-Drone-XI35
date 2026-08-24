#!/usr/bin/env bash

set -eu

PROFILE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRINGUP_ROOT="$(cd "$PROFILE_DIR/../.." && pwd)"
WORKSPACE="$(cd "$BRINGUP_ROOT/.." && pwd)"
PROFILE_TMUX_SCRIPT="$PROFILE_DIR/tmux.sh"

# shellcheck source=/dev/null
source "$PROFILE_DIR/profile.env"
PROFILE_MODE="${ODOMETRY_SOURCE:?ODOMETRY_SOURCE is required}"
case "$PROFILE_MODE" in
    ground_truth|vins|lidar) ;;
    *)
        echo "Unsupported ODOMETRY_SOURCE: $PROFILE_MODE" >&2
        exit 64
        ;;
esac

# shellcheck source=/dev/null
source "$BRINGUP_ROOT/lib/tmux_runtime.sh"

prepend_env_path() {
    local variable_name="$1"
    local new_path="$2"
    local current_value="${!variable_name:-}"
    case ":$current_value:" in
        *":$new_path:"*) return ;;
    esac
    printf -v "$variable_name" '%s%s' \
        "$new_path" "${current_value:+:$current_value}"
    export "$variable_name"
}

setup_profile_environment() {
    export FAST_DRONE_ROOT PX4_DIR PX4_BUILD_DIR SITL_GAZEBO
    prepend_env_path GAZEBO_PLUGIN_PATH "$PX4_BUILD_DIR/build_gazebo"
    prepend_env_path LD_LIBRARY_PATH "$PX4_BUILD_DIR/build_gazebo"
    prepend_env_path GAZEBO_MODEL_PATH "$FAST_DRONE_ROOT/src/simulation/gazebo_models"
    prepend_env_path GAZEBO_MODEL_PATH "$SITL_GAZEBO/models"
    prepend_env_path GAZEBO_MODEL_PATH \
        "$FAST_DRONE_ROOT/src/simulation/uav_simulator/simulation_bridges/models"
    prepend_env_path ROS_PACKAGE_PATH "$SITL_GAZEBO"
    prepend_env_path ROS_PACKAGE_PATH "$PX4_DIR"
}

build_profile_layout() {
    new_window "simulator" \
        "roslaunch '$PROFILE_DIR/launch/simulator.launch'" manual

    case "$PROFILE_MODE" in
        ground_truth)
            new_window "ground truth" \
                "roslaunch '$PROFILE_DIR/launch/ground_truth/localization.launch'" manual
            ;;
        vins)
            new_window "vins location" \
                "roslaunch '$PROFILE_DIR/launch/vins/frontend.launch'" manual
            add_pane "vins location" "vins" \
                "roslaunch '$PROFILE_DIR/launch/vins/estimator.launch'" manual
            ;;
        lidar)
            add_pane "simulator" "lidar format" \
                "roslaunch '$PROFILE_DIR/launch/lidar_bridge.launch'" manual
            new_window "fastlivo2 location" \
                "roslaunch '$PROFILE_DIR/launch/fastlivo2.launch'" manual
            ;;
    esac

    new_window "px4ctrl" \
        "roslaunch '$PROFILE_DIR/launch/$PROFILE_MODE/px4ctrl.launch'" manual
    add_pane "px4ctrl" "land" \
        "bash '$BRINGUP_ROOT/commands/land.sh'" manual
    add_pane "px4ctrl" "takeoff" \
        "bash '$BRINGUP_ROOT/commands/takeoff.sh'" manual

    new_window "planner" \
        "roslaunch '$PROFILE_DIR/launch/$PROFILE_MODE/ego_planner.launch'" manual
    add_pane "planner" "search_plan" \
        "roslaunch '$PROFILE_DIR/launch/$PROFILE_MODE/search_plan.launch'" manual
    add_pane "planner" "yopo" \
        "roslaunch '$PROFILE_DIR/launch/$PROFILE_MODE/yopo.launch'" manual

    new_window "services" \
        "roslaunch '$PROFILE_DIR/launch/services.launch'" auto
}

run_tmux_profile "$@"
