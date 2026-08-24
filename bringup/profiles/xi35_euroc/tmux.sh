#!/usr/bin/env bash

set -eu

PROFILE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRINGUP_ROOT="$(cd "$PROFILE_DIR/../.." && pwd)"
WORKSPACE="$(cd "$BRINGUP_ROOT/.." && pwd)"
PROFILE_TMUX_SCRIPT="$PROFILE_DIR/tmux.sh"

# shellcheck source=/dev/null
source "$PROFILE_DIR/profile.env"
PROFILE_MODE="euroc"

# shellcheck source=/dev/null
source "$BRINGUP_ROOT/lib/tmux_runtime.sh"

build_profile_layout() {
    new_window "vins location" \
        "roslaunch '$PROFILE_DIR/launch/vins.launch'" manual
    add_pane "vins location" "kf_fusion" \
        "bash '$BRINGUP_ROOT/scripts/start_kf_fusion.sh'" manual

    new_window "rosbag" "rosbag play" manual

    new_window "services" \
        "roslaunch '$PROFILE_DIR/launch/rosout.launch'" auto
}

run_tmux_profile "$@"
