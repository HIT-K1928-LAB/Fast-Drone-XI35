#!/usr/bin/env bash

set -eu

PROFILE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRINGUP_ROOT="$(cd "$PROFILE_DIR/../.." && pwd)"
WORKSPACE="$(cd "$BRINGUP_ROOT/.." && pwd)"
PROFILE_TMUX_SCRIPT="$PROFILE_DIR/tmux.sh"

# shellcheck source=/dev/null
source "$PROFILE_DIR/profile.env"
PROFILE_MODE="${LOCALIZATION_DEFAULT:?LOCALIZATION_DEFAULT is required}"
case "$PROFILE_MODE" in
    vins_stereo|mocap|motion_capture) ;;
    *)
        echo "Unsupported localization mode: $PROFILE_MODE" >&2
        exit 64
        ;;
esac

# shellcheck source=/dev/null
source "$BRINGUP_ROOT/lib/tmux_runtime.sh"

build_profile_layout() {
    case "$PROFILE_MODE" in
        vins_stereo)
            new_window "vins location" \
                "roslaunch '$PROFILE_DIR/launch/vins.launch'" manual
            add_pane "vins location" "kf_fusion" \
                "bash '$BRINGUP_ROOT/scripts/start_kf_fusion.sh'" manual
            ;;
        mocap|motion_capture)
            new_window "mocap location" \
                "bash '$BRINGUP_ROOT/scripts/start_mocap.sh'" manual
            ;;
    esac

    new_window "px4ctrl" \
        "roslaunch '$PROFILE_DIR/launch/px4ctrl.launch'" manual
    add_pane "px4ctrl" "land" \
        "bash '$BRINGUP_ROOT/commands/land.sh'" manual
    add_pane "px4ctrl" "takeoff" \
        "bash '$BRINGUP_ROOT/commands/takeoff.sh'" manual

    new_window "planner" \
        "roslaunch '$PROFILE_DIR/launch/ego_planner.launch'" manual
    add_pane "planner" "search_plan" \
        "roslaunch '$PROFILE_DIR/launch/search_plan.launch'" manual
    add_pane "planner" "yopo" \
        "roslaunch '$PROFILE_DIR/launch/yopo.launch'" manual

    new_window "services" \
        "roslaunch '$PROFILE_DIR/launch/services.launch'" auto
}

run_tmux_profile "$@"
