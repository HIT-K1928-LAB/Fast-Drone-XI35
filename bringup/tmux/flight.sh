#!/usr/bin/env bash

set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "$SCRIPT_DIR/../.." && pwd)"
PANE_SHELL="$SCRIPT_DIR/flight.sh"

list_available_profiles() {
    local profile_env
    local -a profiles=()

    for profile_env in "$WORKSPACE"/bringup/profiles/*/profile.env; do
        if [ -f "$profile_env" ]; then
            profiles+=("$(basename "$(dirname "$profile_env")")")
        fi
    done

    if [ "${#profiles[@]}" -eq 0 ]; then
        echo "(none)"
    else
        printf '%s\n' "${profiles[@]}" | sort
    fi
}

print_completion() {
    cat <<'EOF'
_fast_drone_flight_completion() {
    local cur script profiles

    COMPREPLY=()
    cur="${COMP_WORDS[COMP_CWORD]}"
    script="${COMP_WORDS[0]}"

    case "$COMP_CWORD" in
        1)
            profiles="$("$script" --list-profiles 2>/dev/null)"
            COMPREPLY=( $(compgen -W "$profiles --help --list-profiles completion" -- "$cur") )
            ;;
        2)
            COMPREPLY=( $(compgen -W "vins_stereo mocap motion_capture" -- "$cur") )
            ;;
        3)
            COMPREPLY=( $(compgen -W "xfeat" -- "$cur") )
            ;;
    esac
}

complete -F _fast_drone_flight_completion flight.sh
complete -F _fast_drone_flight_completion ./bringup/tmux/flight.sh
complete -F _fast_drone_flight_completion bringup/tmux/flight.sh
EOF
    printf 'complete -F _fast_drone_flight_completion %q\n' "$PANE_SHELL"
}

show_help() {
    cat <<EOF
Usage:
  $(basename "$0") [profile] [localization] [frontend]
  $(basename "$0") --help
  $(basename "$0") --list-profiles
  $(basename "$0") completion

Arguments:
  profile       Vehicle profile name (default: xi35_default)
  localization  vins_stereo or mocap
  frontend      Frontend name (default: xfeat)

Commands:
  completion    Print Bash completion definitions

Examples:
  $(basename "$0") xi35_10
  $(basename "$0") xi35_10 vins_stereo xfeat
  $(basename "$0") xi35_10 mocap

Available vehicle profiles:
$(list_available_profiles | sed 's/^/  /')

Window run modes used inside this script:
  manual  Prefill the command and wait for Enter
  auto    Run the command immediately
EOF
}

case "${1:-}" in
    -h|--help)
        show_help
        exit 0
        ;;
    --list-profiles)
        list_available_profiles
        exit 0
        ;;
    completion)
        print_completion
        exit 0
        ;;
esac

if ! command -v tmux >/dev/null 2>&1; then
    echo "Error: tmux is not installed or is not available in PATH." >&2
    echo "Install it on Ubuntu/Debian with: sudo apt update && sudo apt install tmux" >&2
    exit 127
fi

# =============================================================================
# 用户维护区域
# =============================================================================
# 通常，仅在添加程序或调整程序排列时编辑此部分。
#
# Public helpers:
#   new_window "window_name" "command" run_mode ["command" run_mode ...]
#   add_pane   "window_name" "pane_title" "command" run_mode \
#              ["command" run_mode ...]
#
# run_mode:
#   manual -> prefill the command and wait for Enter (recommended for flight)
#   auto   -> run the command immediately
#
# 同一窗格中的多条命令会按照从左到右的顺序处理。示例：
#   add_pane "flight" "land" \
#       "echo preparing" auto \
#       "bash '$WORKSPACE/bringup/commands/land.sh'" manual
# =============================================================================

setup_common_environment() {
    export ROS_LOG_DIR="$WORKSPACE/log"
    mkdir -p "$ROS_LOG_DIR"
}

start_localization() {
    case "$LOCALIZATION" in
        vins_stereo)
            new_window "vins location" \
                "bash '$WORKSPACE/bringup/scripts/start_vins.sh' '$VINS_CONFIG_FILE'" \
                manual

            add_pane "vins location" "kf_fusion" \
                "bash '$WORKSPACE/bringup/scripts/start_kf_fusion.sh'" \
                manual
            ;;

        mocap|motion_capture)
            new_window "mocap location" \
                "bash '$WORKSPACE/bringup/scripts/start_mocap.sh'" \
                manual
            ;;

        *)
            echo "Unsupported localization mode: $LOCALIZATION" >&2
            exit 1
            ;;
    esac
}

start_control() {
    new_window "px4ctrl" \
        "roslaunch px4ctrl run_ctrl.launch config_file:='$PX4CTRL_CONFIG_FILE'" \
        manual

    add_pane "px4ctrl" "land" \
        "bash '$WORKSPACE/bringup/commands/land.sh'" \
        manual

    add_pane "px4ctrl" "takeoff"  \
        "bash '$WORKSPACE/bringup/commands/takeoff.sh'" \
        manual

    new_window "planner" \
        "roslaunch ego_planner single_run_in_exp.launch" \
        manual

    add_pane "planner" "search_plan" \
        "roslaunch search_plan search_plan.launch" \
        manual

    add_pane "planner" "yopo" \
        "roslaunch yopo_planner yopo_planner.launch" \
        manual
}

start_services() {
    new_window "services" \
        "roslaunch uav_utils rosout_file_logger.launch" \
        auto
}

build_default_user_layout() {
    start_localization
    start_control
    start_services
}

# =============================================================================
# 用户维护区域结束
# Everything below is runtime plumbing and normally does not need editing.
# =============================================================================

# Internal mode used by tmux to create a clean, profile-aware pane shell.
if [ "${1:-}" = "--pane-shell" ]; then
    PANE_WORKSPACE="${2:?Missing workspace path}"
    PANE_PROFILE="${3:?Missing profile name}"
    READY_SIGNAL="${4:?Missing tmux ready signal}"
    LAYOUT_GATE="${5:?Missing layout-ready gate}"
    PANE_PROFILE_DIR="$PANE_WORKSPACE/bringup/profiles/$PANE_PROFILE"

    shift 5
    if [ "$#" -eq 0 ] || [ $(( $# % 2 )) -ne 0 ]; then
        echo "Invalid pane command plan: expected command/mode pairs" >&2
        exit 64
    fi

    source "$PANE_PROFILE_DIR/profile.env"
    source "$PANE_WORKSPACE/devel/setup.bash" >/dev/null
    setup_common_environment

    export ROS_MASTER_URI="$ROS_MASTER_URI_DEFAULT"
    export ROS_IP="$ROS_IP_DEFAULT"
    export FASTDRONE_PROFILE="$PANE_PROFILE"
    export FASTDRONE_PROFILE_DIR="$PANE_PROFILE_DIR"
    VINS_CONFIG_FILE=""
    PX4CTRL_CONFIG_FILE=""
    if [ -n "${VINS_CONFIG:-}" ]; then
        VINS_CONFIG_FILE="$PANE_PROFILE_DIR/$VINS_CONFIG"
    fi
    if [ -n "${PX4CTRL_CONFIG:-}" ]; then
        PX4CTRL_CONFIG_FILE="$PANE_PROFILE_DIR/$PX4CTRL_CONFIG"
    fi
    export VINS_CONFIG_FILE
    export PX4CTRL_CONFIG_FILE

    cd "$PANE_WORKSPACE"
    tmux wait-for -S "$READY_SIGNAL"

    while [ ! -e "$LAYOUT_GATE" ]; do
        sleep 0.02
    done
    export PS1='\u@\h:\w\$ '

    set +e
    if [ "$(id -u)" -eq 0 ]; then
        PROMPT_MARK='#'
    else
        PROMPT_MARK='$'
    fi
    PANE_PROMPT="$(id -un)@${HOSTNAME%%.*}:$PWD${PROMPT_MARK} "

    while [ "$#" -gt 0 ]; do
        ENCODED_COMMAND="$1"
        RUN_MODE="$2"
        shift 2
        PANE_COMMAND="$(printf '%s' "$ENCODED_COMMAND" | base64 --decode)"

        case "$RUN_MODE" in
            auto)
                trap 'printf "\n"' INT
                eval "$PANE_COMMAND"
                trap - INT
                ;;
            manual)
                trap 'printf "\n"; stty sane; exec bash --noprofile --norc -i' INT
                if IFS= read -e -r -i "$PANE_COMMAND" -p "$PANE_PROMPT" EDITED_COMMAND; then
                    eval "$EDITED_COMMAND"
                fi
                trap - INT
                ;;
            *)
                echo "Invalid pane run mode: $RUN_MODE (expected manual or auto)" >&2
                exit 64
                ;;
        esac
    done

    exec bash --noprofile --norc -i
fi

PROFILE="${1:-xi35_default}"

PROFILE_DIR="$WORKSPACE/bringup/profiles/$PROFILE"
PROFILE_ENV="$PROFILE_DIR/profile.env"

if [ ! -f "$PROFILE_ENV" ]; then
    echo "Profile not found: $PROFILE" >&2
    echo "Available profiles:" >&2
    list_available_profiles | sed 's/^/  /' >&2
    exit 1
fi

source "$PROFILE_ENV"

: "${ROS_MASTER_URI_DEFAULT:?ROS_MASTER_URI_DEFAULT is not set in $PROFILE_ENV}"
: "${ROS_IP_DEFAULT:?ROS_IP_DEFAULT is not set in $PROFILE_ENV}"

LOCALIZATION="${2:-${LOCALIZATION_DEFAULT:-vins_stereo}}"
FRONTEND="${3:-xfeat}"

VINS_CONFIG_FILE=""
PX4CTRL_CONFIG_FILE=""
if [ -n "${VINS_CONFIG:-}" ]; then
    VINS_CONFIG_FILE="$PROFILE_DIR/$VINS_CONFIG"
fi
if [ -n "${PX4CTRL_CONFIG:-}" ]; then
    PX4CTRL_CONFIG_FILE="$PROFILE_DIR/$PX4CTRL_CONFIG"
fi

if [ -z "${FLIGHT_LAYOUT_SCRIPT:-}" ]; then
    : "${VINS_CONFIG:?VINS_CONFIG is required by the built-in layout in $PROFILE_ENV}"
    : "${PX4CTRL_CONFIG:?PX4CTRL_CONFIG is required by the built-in layout in $PROFILE_ENV}"
    BUILTIN_LAYOUT_REQUIRED_FILES=("$VINS_CONFIG_FILE" "$PX4CTRL_CONFIG_FILE")
else
    BUILTIN_LAYOUT_REQUIRED_FILES=()
fi

for required_file in "${BUILTIN_LAYOUT_REQUIRED_FILES[@]}" "$WORKSPACE/devel/setup.bash"; do
    if [ ! -f "$required_file" ]; then
        echo "Required file not found: $required_file" >&2
        exit 1
    fi
done

# The profile has the highest priority, regardless of the caller's shell.
ROS_MASTER_URI="$ROS_MASTER_URI_DEFAULT"
ROS_IP="$ROS_IP_DEFAULT"
CAMERA_TYPE_STATUS="${CAMERA_TYPE:-unknown}"
VINS_CONFIG_STATUS="${VINS_CONFIG_FILE:-not configured}"
PX4CTRL_CONFIG_STATUS="${PX4CTRL_CONFIG_FILE:-not configured}"

if git -C "$WORKSPACE" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    GIT_BRANCH="$(git -C "$WORKSPACE" branch --show-current)"
    if [ -z "$GIT_BRANCH" ]; then
        GIT_BRANCH="detached-head"
    fi
    GIT_COMMIT="$(git -C "$WORKSPACE" rev-parse --short HEAD)"
    if [ -n "$(git -C "$WORKSPACE" status --porcelain)" ]; then
        GIT_WORKTREE="dirty"
    else
        GIT_WORKTREE="clean"
    fi
else
    GIT_BRANCH="unavailable"
    GIT_COMMIT="unavailable"
    GIT_WORKTREE="unavailable"
fi

export WORKSPACE
export PROFILE
export PROFILE_DIR
export VINS_CONFIG_FILE
export PX4CTRL_CONFIG_FILE
export ROS_MASTER_URI
export ROS_IP

SESSION="fd_${PROFILE}_${LOCALIZATION}"
LAYOUT_GATE="/tmp/${SESSION}_layout_ready"
rm -f "$LAYOUT_GATE"

wait_for_pane() {
    local signal="$1"
    tmux wait-for "$signal"
}

validate_run_mode() {
    case "$1" in
        manual|auto)
            ;;
        *)
            echo "Invalid run mode: $1 (expected manual or auto)" >&2
            exit 64
            ;;
    esac
}

build_command_plan() {
    local command
    local run_mode
    local encoded_command
    local plan=""

    if [ "$#" -eq 0 ] || [ $(( $# % 2 )) -ne 0 ]; then
        echo "Expected one or more command/run-mode pairs" >&2
        return 64
    fi

    while [ "$#" -gt 0 ]; do
        command="$1"
        run_mode="$2"
        shift 2

        validate_run_mode "$run_mode"
        encoded_command="$(printf '%s' "$command" | base64 -w 0)"
        plan+=" '$encoded_command' '$run_mode'"
    done

    printf '%s' "$plan"
}

new_window() {
    local name="$1"
    shift
    local signal="${SESSION}_${name}_ready"
    local pane
    local command_plan

    command_plan="$(build_command_plan "$@")"

    tmux new-window -d -t "$SESSION" -n "$name" -c "$WORKSPACE" \
        "'$PANE_SHELL' --pane-shell '$WORKSPACE' '$PROFILE' '$signal' '$LAYOUT_GATE'$command_plan"
    wait_for_pane "$signal"
    pane="$(tmux display-message -p -t "$SESSION:$name" '#{pane_id}')"
    tmux select-pane -t "$pane" -T "$name"
}

add_pane() {
    local window="$1"
    local title="$2"
    shift 2
    local pane
    local pane_count
    local split_direction
    local signal="${SESSION}_${window}_${title}_ready"
    local command_plan

    command_plan="$(build_command_plan "$@")"

    pane_count="$(tmux list-panes -t "$SESSION:$window" | wc -l)"
    if [ "$pane_count" -eq 1 ]; then
        split_direction="-h"
    else
        split_direction="-v"
    fi

    pane="$(tmux split-window \
        "$split_direction" -d -t "$SESSION:$window" -c "$WORKSPACE" \
        -P -F '#{pane_id}' \
        "'$PANE_SHELL' --pane-shell '$WORKSPACE' '$PROFILE' '$signal' '$LAYOUT_GATE'$command_plan")"

    wait_for_pane "$signal"
    tmux select-pane -t "$pane" -T "$title"
    tmux select-layout -t "$SESSION:$window" tiled >/dev/null
}

# A profile may replace the built-in tmux layout with a shell file of its own.
# Relative paths are resolved from the workspace root. The file is sourced only
# after every public layout helper has been defined.
if [ -n "${FLIGHT_LAYOUT_SCRIPT:-}" ]; then
    case "$FLIGHT_LAYOUT_SCRIPT" in
        /*)
            FLIGHT_LAYOUT_SCRIPT_FILE="$FLIGHT_LAYOUT_SCRIPT"
            ;;
        *)
            FLIGHT_LAYOUT_SCRIPT_FILE="$WORKSPACE/$FLIGHT_LAYOUT_SCRIPT"
            ;;
    esac

    if [ ! -f "$FLIGHT_LAYOUT_SCRIPT_FILE" ]; then
        echo "Flight layout script not found: $FLIGHT_LAYOUT_SCRIPT_FILE" >&2
        echo "Configured by FLIGHT_LAYOUT_SCRIPT in $PROFILE_ENV" >&2
        exit 1
    fi
    if [ ! -r "$FLIGHT_LAYOUT_SCRIPT_FILE" ]; then
        echo "Flight layout script is not readable: $FLIGHT_LAYOUT_SCRIPT_FILE" >&2
        exit 1
    fi

    unset -f build_user_layout 2>/dev/null || true
    # shellcheck source=/dev/null
    source "$FLIGHT_LAYOUT_SCRIPT_FILE"
    if ! declare -F build_user_layout >/dev/null; then
        echo "Flight layout script must define build_user_layout():" >&2
        echo "  $FLIGHT_LAYOUT_SCRIPT_FILE" >&2
        exit 64
    fi
else
    FLIGHT_LAYOUT_SCRIPT_FILE="built-in default"
    build_user_layout() {
        build_default_user_layout
    }
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Attaching existing session: $SESSION"
    exec tmux attach-session -t "$SESSION"
fi

STATUS_SIGNAL="${SESSION}_status_ready"
STATUS_COMMAND="printf '\n  Fast-Drone flight session\n\n  %-14s : %s\n  %-14s : %s\n  %-14s : %s\n  %-14s : %s\n  %-14s : %s\n  %-14s : %s\n\n  %-14s : %s\n  %-14s : %s\n  %-14s : %s\n  %-14s : %s\n\n  %-14s : %s\n  %-14s : %s\n  %-14s : %s\n\n' \
'Profile' '$PROFILE' \
'Localization' '$LOCALIZATION' \
'Frontend' '$FRONTEND' \
'Camera type' '$CAMERA_TYPE_STATUS' \
'ROS master' '$ROS_MASTER_URI' \
'ROS IP' '$ROS_IP' \
'Profile dir' '$PROFILE_DIR' \
'Layout script' '$FLIGHT_LAYOUT_SCRIPT_FILE' \
'VINS config' '$VINS_CONFIG_STATUS' \
'PX4Ctrl config' '$PX4CTRL_CONFIG_STATUS' \
'Git branch' '$GIT_BRANCH' \
'Git commit' '$GIT_COMMIT' \
'Git worktree' '$GIT_WORKTREE'"
STATUS_COMMAND_ENCODED="$(printf '%s' "$STATUS_COMMAND" | base64 -w 0)"
tmux new-session -d -s "$SESSION" -n "status" -c "$WORKSPACE" \
    "'$PANE_SHELL' --pane-shell '$WORKSPACE' '$PROFILE' '$STATUS_SIGNAL' '$LAYOUT_GATE' '$STATUS_COMMAND_ENCODED' 'auto'"
wait_for_pane "$STATUS_SIGNAL"

# Session-local mouse support: click panes/windows, resize panes, and scroll.
tmux set-option -t "$SESSION" mouse on
tmux set-option -t "$SESSION" status on
tmux set-option -t "$SESSION" history-limit 50000
tmux set-option -t "$SESSION" status-left-length 48
tmux set-option -t "$SESSION" status-left \
    '#[fg=colour39,bold] #S #[default]'
tmux set-option -t "$SESSION" status-right-length 24
tmux set-option -t "$SESSION" status-right \
    '#[fg=white,bg=colour160,bold] EXIT SESSION #[default]'
tmux set-option -t "$SESSION" @fastdrone_exit_button 1
tmux bind-key -T root MouseDown1StatusRight \
    if-shell -F '#{==:#{@fastdrone_exit_button},1}' \
    'kill-session' 'display-message "No action"'

build_user_layout

# All window sizes are final. Release every pane to display or run its command.
touch "$LAYOUT_GATE"

tmux select-window -t "$SESSION:status"
exec tmux attach-session -t "$SESSION"
