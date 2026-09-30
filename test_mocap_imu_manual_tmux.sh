#!/usr/bin/env bash
set -euo pipefail

# Run inside fd_runtime. Commands are typed into tmux but NOT executed.
SESSION="mocap_imu_manual_test"
WORKSPACE="/root/Fast-Drone-XI35"
ROS_MASTER_URI="http://192.168.31.28:11311"
ROS_IP="192.168.31.28"
KF_CONFIG="$WORKSPACE/src/realflight_modules/kf_fusion/config/config.yaml"

if [[ ! -f "$WORKSPACE/devel/setup.bash" ]]; then
    echo "Missing $WORKSPACE/devel/setup.bash" >&2
    exit 1
fi
if [[ ! -f "$KF_CONFIG" ]] || ! grep -Eq '^[[:space:]]*use_motion_capture:[[:space:]]*true([[:space:]]*(#.*)?)?$' "$KF_CONFIG"; then
    echo "Check use_motion_capture: true in $KF_CONFIG" >&2
    exit 1
fi
command -v tmux >/dev/null || { echo "tmux is required" >&2; exit 1; }

base="export ROS_MASTER_URI=$ROS_MASTER_URI ROS_IP=$ROS_IP; source $WORKSPACE/devel/setup.bash; cd $WORKSPACE"

prefill() {
    local target="$1"
    local command="$2"
    tmux send-keys -t "$target" "$base; $command"
}

if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux new-session -d -s "$SESSION" -n "01-mocap"
    tmux set-option -t "$SESSION" remain-on-exit on
    prefill "$SESSION:01-mocap" "bash shfiles/motion_capture_receive_publish.sh"

    tmux new-window -t "$SESSION" -n "02-mavros"
    prefill "$SESSION:02-mavros" "roslaunch mavros px4.launch"

    tmux new-window -t "$SESSION" -n "03-imu-filter"
    prefill "$SESSION:03-imu-filter" "roslaunch imu_filter imu_filter.launch"

    tmux new-window -t "$SESSION" -n "04-kf-fusion"
    prefill "$SESSION:04-kf-fusion" "roslaunch kf_fusion kf_fusion.launch"

    tmux new-window -t "$SESSION" -n "05-px4ctrl"
    prefill "$SESSION:05-px4ctrl" "roslaunch px4ctrl run_ctrl.launch"

    tmux new-window -t "$SESSION" -n "06-topic-hz"
    prefill "$SESSION:06-topic-hz" "rostopic hz /motion_capture/motion_capture_odom"
    second_pane="$(tmux split-window -v -t "$SESSION:06-topic-hz" -P -F '#{pane_id}')"
    prefill "$second_pane" "rostopic hz /kf_fusion/kf_imu_odom"
    tmux select-layout -t "$SESSION:06-topic-hz" even-vertical >/dev/null
    tmux select-window -t "$SESSION:01-mocap"
fi

cat <<INFO
Session: $SESSION
Each command is prefilled and waits for your Enter key.
Suggested order: 01 mocap, 02 MAVROS, 03 IMU filter, 04 KF fusion,
then 06 topic rates; start 05 PX4Ctrl only after checking the inputs.
Change window: Ctrl-b then 0-5 or n/p. Detach: Ctrl-b then d.
Stop after testing: tmux kill-session -t $SESSION
Do not run duplicate MAVROS or PX4Ctrl processes.
INFO

if [[ -t 0 && -t 1 ]]; then
    exec tmux attach-session -t "$SESSION"
fi
