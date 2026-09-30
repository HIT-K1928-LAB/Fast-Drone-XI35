#!/usr/bin/env bash
set -euo pipefail

# Run inside the fd_runtime container. Ground localization check only.
SESSION="mocap_imu_fusion_test"
WORKSPACE="/root/Fast-Drone-XI35"
ROS_MASTER_URI="http://192.168.31.28:11311"
ROS_IP="192.168.31.28"

if [[ ! -f "$WORKSPACE/devel/setup.bash" ]]; then
    echo "Missing ROS workspace: $WORKSPACE/devel/setup.bash" >&2
    exit 1
fi
if ! command -v tmux >/dev/null 2>&1; then
    echo "tmux is not installed" >&2
    exit 1
fi

# The active KF config must select motion capture before this test.
KF_CONFIG="$WORKSPACE/src/realflight_modules/kf_fusion/config/config.yaml"
if ! grep -Eq '^[[:space:]]*use_motion_capture:[[:space:]]*true([[:space:]]*(#.*)?)?$' "$KF_CONFIG"; then
    echo "KF config does not have use_motion_capture: true: $KF_CONFIG" >&2
    exit 1
fi

base="export ROS_MASTER_URI=$ROS_MASTER_URI ROS_IP=$ROS_IP; source $WORKSPACE/devel/setup.bash; cd $WORKSPACE"

run_window() {
    local name="$1"
    local command="$2"
    tmux new-window -t "$SESSION" -n "$name"
    tmux send-keys -t "$SESSION:$name" "$base; $command"
}

if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux new-session -d -s "$SESSION" -n "01-mocap"
    tmux set-option -t "$SESSION" remain-on-exit on
    tmux send-keys -t "$SESSION:01-mocap" "$base; bash shfiles/motion_capture_receive_publish.sh"

    run_window "02-mavros" "roslaunch mavros px4.launch"
    run_window "03-imu-filter" "roslaunch imu_filter imu_filter.launch"
    run_window "04-kf-fusion" "roslaunch kf_fusion kf_fusion.launch"

    tmux new-window -t "$SESSION" -n "05-topic-hz"
    tmux send-keys -t "$SESSION:05-topic-hz" "$base; rostopic hz /motion_capture/motion_capture_odom"
    pane="$(tmux split-window -v -t "$SESSION:05-topic-hz" -P -F '#{pane_id}')"
    tmux send-keys -t "$pane" "$base; rostopic hz /kf_fusion/kf_imu_odom"
    tmux select-layout -t "$SESSION:05-topic-hz" even-vertical >/dev/null
    tmux select-window -t "$SESSION:05-topic-hz"
fi

echo "Session: $SESSION"
echo "Check both topic rates and the four node windows for errors."
echo "Detach: Ctrl-b d; stop test: tmux kill-session -t $SESSION"
if [[ -t 0 && -t 1 ]]; then
    exec tmux attach-session -t "$SESSION"
fi
