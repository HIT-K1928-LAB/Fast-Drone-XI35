#!/usr/bin/env bash

# Shared tmux mechanics for bringup/profiles/*/tmux.sh. This file is sourced;
# profile scripts own ROS environment hooks and window/pane layout.

tmux_cmd() {
    if [ -n "${FASTDRONE_TMUX_SOCKET:-}" ]; then
        command tmux -L "$FASTDRONE_TMUX_SOCKET" "$@"
    else
        command tmux "$@"
    fi
}

configure_tmux_session() {
    : "${SESSION:?SESSION is required}"

    tmux_cmd set-option -t "$SESSION" mouse on

    # Ptyxis/VTE ignores OSC52 clipboard writes. Keep mouse selection and
    # paste entirely inside tmux so copied text is never confused with the
    # outer terminal's stale clipboard or PRIMARY selection.
    tmux_cmd set-option -s set-clipboard off
    tmux_cmd bind-key -T copy-mode \
        MouseDragEnd1Pane send-keys -X copy-selection-and-cancel
    tmux_cmd bind-key -T copy-mode-vi \
        MouseDragEnd1Pane send-keys -X copy-selection-and-cancel
    tmux_cmd bind-key -T root MouseDown2Pane \
        paste-buffer -p -t =
    tmux_cmd bind-key -T root MouseDown3Pane \
        paste-buffer -p -t =

    tmux_cmd set-option -t "$SESSION" status on
    tmux_cmd set-option -t "$SESSION" history-limit 50000
    tmux_cmd set-option -t "$SESSION" status-left-length 48
    tmux_cmd set-option -t "$SESSION" status-left \
        '#[fg=colour39,bold] #S #[default]'
    tmux_cmd set-option -t "$SESSION" status-right-length 24
    tmux_cmd set-option -t "$SESSION" status-right \
        '#[fg=white,bg=colour160,bold] EXIT SESSION #[default]'
    tmux_cmd set-option -t "$SESSION" @fastdrone_exit_button 1
    tmux_cmd bind-key -T root MouseDown1StatusRight \
        if-shell -F '#{==:#{@fastdrone_exit_button},1}' \
        'kill-session' \
        'display-message "No action"'
}

restore_terminal_after_tmux() {
    if [ ! -r /dev/tty ] || [ ! -w /dev/tty ]; then
        return
    fi

    # tmux 3.0a can leave mouse/focus reporting enabled when a session is
    # killed from a mouse binding. The next shell would then print click
    # packets such as "Y5#Y5" as ordinary input.
    command stty sane </dev/tty >/dev/null 2>&1 || true
    printf '%b' \
        '\033[?9l\033[?1000l\033[?1001l\033[?1002l\033[?1003l' \
        '\033[?1004l\033[?1005l\033[?1006l\033[?1015l' >/dev/tty
}

attach_tmux_session() {
    : "${SESSION:?SESSION is required}"

    local attach_status=0
    trap 'restore_terminal_after_tmux' EXIT
    tmux_cmd attach-session -t "$SESSION" || attach_status=$?
    trap - EXIT
    restore_terminal_after_tmux
    return "$attach_status"
}

FASTDRONE_PANE_PROCESS_GROUP=""
FASTDRONE_PANE_PROCESS_GROUPS=""

collect_descendant_process_groups() {
    local root_pid="$1"

    python3 - "$root_pid" <<'PY'
import os
import sys

root_pid = int(sys.argv[1])
processes = {}
children = {}
for entry in os.scandir("/proc"):
    if not entry.name.isdigit():
        continue
    try:
        stat = open(f"/proc/{entry.name}/stat", encoding="utf-8").read()
        fields = stat[stat.rfind(")") + 2:].split()
        pid = int(entry.name)
        parent_pid = int(fields[1])
        process_group = int(fields[2])
    except (FileNotFoundError, IndexError, PermissionError, ValueError):
        continue
    processes[pid] = process_group
    children.setdefault(parent_pid, []).append(pid)

descendants = []
pending = [root_pid]
while pending:
    parent_pid = pending.pop()
    for child_pid in children.get(parent_pid, ()):
        descendants.append(child_pid)
        pending.append(child_pid)

seen = set()
for pid in (*descendants, root_pid):
    process_group = processes.get(pid)
    if process_group and process_group not in seen:
        seen.add(process_group)
        print(process_group)
PY
}

refresh_pane_process_groups() {
    local root_pid="${FASTDRONE_PANE_PROCESS_GROUP:-}"
    local own_group process_group discovered_groups
    if [ -z "$root_pid" ]; then
        return
    fi

    own_group="$(ps -o pgid= -p $$ 2>/dev/null | tr -d ' ')"
    discovered_groups="$(collect_descendant_process_groups "$root_pid")"
    for process_group in $discovered_groups; do
        if [ "$process_group" = "$own_group" ]; then
            continue
        fi
        case " $FASTDRONE_PANE_PROCESS_GROUPS " in
            *" $process_group "*) ;;
            *)
                FASTDRONE_PANE_PROCESS_GROUPS="${FASTDRONE_PANE_PROCESS_GROUPS:+$FASTDRONE_PANE_PROCESS_GROUPS }$process_group"
                ;;
        esac
    done
}

signal_pane_process_groups() {
    local signal_name="$1"
    local process_group

    for process_group in $FASTDRONE_PANE_PROCESS_GROUPS; do
        command kill "-$signal_name" -- "-$process_group" 2>/dev/null || true
    done
}

wait_for_pane_process_groups() {
    local max_attempts="$1"
    local attempt process_group any_alive

    for attempt in $(seq 1 "$max_attempts"); do
        any_alive=0
        for process_group in $FASTDRONE_PANE_PROCESS_GROUPS; do
            if command kill -0 -- "-$process_group" 2>/dev/null; then
                any_alive=1
                break
            fi
        done
        if [ "$any_alive" -eq 0 ]; then
            return 0
        fi
        sleep 0.05
    done
    return 1
}

stop_pane_process_group() {
    local process_group="${FASTDRONE_PANE_PROCESS_GROUP:-}"
    if [ -z "$process_group" ]; then
        return
    fi

    refresh_pane_process_groups
    signal_pane_process_groups TERM
    if ! wait_for_pane_process_groups 20; then
        signal_pane_process_groups KILL
    fi
    wait "$process_group" 2>/dev/null || true
    FASTDRONE_PANE_PROCESS_GROUP=""
    FASTDRONE_PANE_PROCESS_GROUPS=""
}

exit_pane_on_signal() {
    local exit_status="$1"
    trap - EXIT HUP INT TERM
    interrupt_pane_process_group
    exit "$exit_status"
}

interrupt_pane_process_group() {
    local process_group="${FASTDRONE_PANE_PROCESS_GROUP:-}"
    if [ -n "$process_group" ]; then
        refresh_pane_process_groups
        signal_pane_process_groups INT
        if ! wait_for_pane_process_groups 40; then
            refresh_pane_process_groups
            signal_pane_process_groups TERM
            if ! wait_for_pane_process_groups 20; then
                signal_pane_process_groups KILL
            fi
        fi
        wait "$process_group" 2>/dev/null || true
        FASTDRONE_PANE_PROCESS_GROUP=""
        FASTDRONE_PANE_PROCESS_GROUPS=""
    fi
}

run_managed_pane_command() {
    local pane_command="$1"
    local command_status=0

    python3 -c '
import os
import signal
import sys

for signal_number in (signal.SIGHUP, signal.SIGINT, signal.SIGTERM):
    signal.signal(signal_number, signal.SIG_DFL)
os.execvp("setsid", ("setsid", "bash", "-c", sys.argv[1]))
' "$pane_command" &
    FASTDRONE_PANE_PROCESS_GROUP=$!
    FASTDRONE_PANE_PROCESS_GROUPS=""
    trap 'stop_pane_process_group' EXIT
    trap 'exit_pane_on_signal 129' HUP
    trap 'interrupt_pane_process_group' INT
    trap 'exit_pane_on_signal 143' TERM

    wait "$FASTDRONE_PANE_PROCESS_GROUP" || command_status=$?
    trap - EXIT HUP INT TERM
    stop_pane_process_group
    return "$command_status"
}

setup_profile_runtime_environment() {
    if [ ! -f "$WORKSPACE/devel/setup.bash" ]; then
        echo "Required file not found: $WORKSPACE/devel/setup.bash" >&2
        return 1
    fi

    # ROS/catkin setup scripts read optional variables before defining them,
    # so they are not safe to source while the caller has nounset enabled.
    # Restore the caller's shell option immediately after the setup completes.
    local fastdrone_had_nounset=0
    local fastdrone_setup_status=0
    case $- in
        *u*)
            fastdrone_had_nounset=1
            set +u
            ;;
    esac
    # shellcheck source=/dev/null
    source "$WORKSPACE/devel/setup.bash" >/dev/null || \
        fastdrone_setup_status=$?
    if [ "$fastdrone_had_nounset" -eq 1 ]; then
        set -u
    fi
    if [ "$fastdrone_setup_status" -ne 0 ]; then
        return "$fastdrone_setup_status"
    fi

    export ROS_LOG_DIR="$WORKSPACE/log"
    mkdir -p "$ROS_LOG_DIR"

    if declare -F setup_profile_environment >/dev/null; then
        setup_profile_environment
    fi

    export ROS_MASTER_URI="$ROS_MASTER_URI_DEFAULT"
    export ROS_IP="$ROS_IP_DEFAULT"
    export WORKSPACE PROFILE_DIR PROFILE_NAME PROFILE_MODE
    export FASTDRONE_PROFILE="$PROFILE_NAME"
    export FASTDRONE_PROFILE_DIR="$PROFILE_DIR"
}

validate_command_pairs() {
    if [ "$#" -eq 0 ] || [ $(( $# % 2 )) -ne 0 ]; then
        echo "Expected one or more command/run-mode pairs" >&2
        return 64
    fi

    while [ "$#" -gt 0 ]; do
        case "$2" in
            manual|auto) ;;
            *)
                echo "Invalid run mode: $2 (expected manual or auto)" >&2
                return 64
                ;;
        esac
        shift 2
    done
}

build_command_plan() {
    local command run_mode encoded_command
    local plan=""

    validate_command_pairs "$@"
    while [ "$#" -gt 0 ]; do
        command="$1"
        run_mode="$2"
        shift 2
        encoded_command="$(printf '%s' "$command" | base64 -w 0)"
        plan+=" '$encoded_command' '$run_mode'"
    done
    printf '%s' "$plan"
}

new_window() {
    local name="$1"
    shift

    validate_command_pairs "$@"
    if [ "${FASTDRONE_PRINT_LAYOUT:-0}" = "1" ]; then
        while [ "$#" -gt 0 ]; do
            printf 'WINDOW|%s|%s|%s\n' "$name" "$2" "$1"
            shift 2
        done
        return
    fi

    local signal="${SESSION}_${name}_ready"
    local pane command_plan
    command_plan="$(build_command_plan "$@")"
    tmux_cmd new-window -d -t "$SESSION" -n "$name" -c "$WORKSPACE" \
        "bash '$PROFILE_TMUX_SCRIPT' --pane-shell '$signal' '$LAYOUT_GATE'$command_plan"
    tmux_cmd wait-for "$signal"
    pane="$(tmux_cmd display-message -p -t "$SESSION:$name" '#{pane_id}')"
    tmux_cmd select-pane -t "$pane" -T "$name"
}

add_pane() {
    local window="$1"
    local title="$2"
    shift 2

    validate_command_pairs "$@"
    if [ "${FASTDRONE_PRINT_LAYOUT:-0}" = "1" ]; then
        while [ "$#" -gt 0 ]; do
            printf 'PANE|%s|%s|%s|%s\n' "$window" "$title" "$2" "$1"
            shift 2
        done
        return
    fi

    local pane pane_count split_direction command_plan
    local signal="${SESSION}_${window}_${title}_ready"
    command_plan="$(build_command_plan "$@")"
    pane_count="$(tmux_cmd list-panes -t "$SESSION:$window" | wc -l)"
    if [ "$pane_count" -eq 1 ]; then
        split_direction="-h"
    else
        split_direction="-v"
    fi
    pane="$(tmux_cmd split-window \
        "$split_direction" -d -t "$SESSION:$window" -c "$WORKSPACE" \
        -P -F '#{pane_id}' \
        "bash '$PROFILE_TMUX_SCRIPT' --pane-shell '$signal' '$LAYOUT_GATE'$command_plan")"
    tmux_cmd wait-for "$signal"
    tmux_cmd select-pane -t "$pane" -T "$title"
    tmux_cmd select-layout -t "$SESSION:$window" tiled >/dev/null
}

run_profile_pane_shell() {
    local ready_signal="${1:?Missing tmux ready signal}"
    local layout_gate="${2:?Missing layout gate}"
    shift 2

    validate_command_pairs "$@"
    setup_profile_runtime_environment
    cd "$WORKSPACE"
    tmux_cmd wait-for -S "$ready_signal"
    while [ ! -e "$layout_gate" ]; do
        sleep 0.02
    done

    export PS1='\u@\h:\w\$ '
    set +e
    local prompt_mark pane_prompt encoded_command run_mode pane_command edited
    if [ "$(id -u)" -eq 0 ]; then prompt_mark='#'; else prompt_mark='$'; fi
    pane_prompt="$(id -un)@${HOSTNAME%%.*}:$PWD${prompt_mark} "

    while [ "$#" -gt 0 ]; do
        encoded_command="$1"
        run_mode="$2"
        shift 2
        pane_command="$(printf '%s' "$encoded_command" | base64 --decode)"
        case "$run_mode" in
            auto)
                run_managed_pane_command "$pane_command"
                ;;
            manual)
                trap 'printf "\n"; stty sane; exec bash --noprofile --norc -i' INT
                if IFS= read -e -r -i "$pane_command" -p "$pane_prompt" edited; then
                    trap - INT
                    run_managed_pane_command "$edited"
                fi
                trap - INT
                ;;
        esac
    done
    exec bash --noprofile --norc -i
}

create_status_window() {
    local git_branch git_commit git_worktree camera_type
    if git -C "$WORKSPACE" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
        git_branch="$(git -C "$WORKSPACE" branch --show-current)"
        git_branch="${git_branch:-detached-head}"
        git_commit="$(git -C "$WORKSPACE" rev-parse --short HEAD)"
        if [ -n "$(git -C "$WORKSPACE" status --porcelain)" ]; then
            git_worktree="dirty"
        else
            git_worktree="clean"
        fi
    else
        git_branch="unavailable"
        git_commit="unavailable"
        git_worktree="unavailable"
    fi
    camera_type="${CAMERA_TYPE:-unknown}"

    local status_command status_encoded status_signal
    status_signal="${SESSION}_status_ready"
    printf -v status_command \
        "printf '\\n  Fast-Drone flight session\\n\\n  %-14s : %%s\\n  %-14s : %%s\\n  %-14s : %%s\\n  %-14s : %%s\\n  %-14s : %%s\\n  %-14s : %%s\\n  %-14s : %%s\\n  %-14s : %%s\\n  %-14s : %%s\\n  %-14s : %%s\\n\\n' %q %q %q %q %q %q %q %q %q %q" \
        Profile Mode "Camera type" "ROS master" "ROS IP" "Profile dir" \
        "Profile script" "Git branch" "Git commit" "Git worktree" \
        "$PROFILE_NAME" "$PROFILE_MODE" "$camera_type" "$ROS_MASTER_URI" \
        "$ROS_IP" "$PROFILE_DIR" "$PROFILE_TMUX_SCRIPT" "$git_branch" \
        "$git_commit" "$git_worktree"
    status_encoded="$(printf '%s' "$status_command" | base64 -w 0)"
    tmux_cmd new-session -d -s "$SESSION" -n status -c "$WORKSPACE" \
        "bash '$PROFILE_TMUX_SCRIPT' --pane-shell '$status_signal' '$LAYOUT_GATE' '$status_encoded' auto"
    tmux_cmd wait-for "$status_signal"
}

run_tmux_profile() {
    : "${WORKSPACE:?WORKSPACE is required}"
    : "${PROFILE_DIR:?PROFILE_DIR is required}"
    : "${PROFILE_NAME:?PROFILE_NAME is required}"
    : "${PROFILE_MODE:?PROFILE_MODE is required}"
    : "${PROFILE_TMUX_SCRIPT:?PROFILE_TMUX_SCRIPT is required}"
    : "${ROS_MASTER_URI_DEFAULT:?ROS_MASTER_URI_DEFAULT is required}"
    : "${ROS_IP_DEFAULT:?ROS_IP_DEFAULT is required}"

    case "${1:-}" in
        --print-layout)
            if [ "$#" -ne 1 ]; then return 64; fi
            FASTDRONE_PRINT_LAYOUT=1
            printf 'PROFILE|%s\nMODE|%s\n' "$PROFILE_NAME" "$PROFILE_MODE"
            build_profile_layout
            return
            ;;
        --pane-shell)
            shift
            run_profile_pane_shell "$@"
            return
            ;;
        "") ;;
        *)
            echo "Profile tmux scripts do not accept public arguments." >&2
            return 64
            ;;
    esac
    if [ "$#" -ne 0 ]; then return 64; fi
    if ! command -v tmux >/dev/null 2>&1; then
        echo "Error: tmux is not installed or unavailable." >&2
        return 127
    fi

    setup_profile_runtime_environment
    SESSION="fd_${PROFILE_NAME}_${PROFILE_MODE}"
    LAYOUT_GATE="/tmp/${SESSION}_layout_ready"
    rm -f "$LAYOUT_GATE"

    if tmux_cmd has-session -t "$SESSION" 2>/dev/null; then
        attach_tmux_session
        return
    fi

    create_status_window
    configure_tmux_session
    build_profile_layout
    touch "$LAYOUT_GATE"
    tmux_cmd select-window -t "$SESSION:status"
    attach_tmux_session
}
