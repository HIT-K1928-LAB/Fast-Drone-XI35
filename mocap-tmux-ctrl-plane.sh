#!/usr/bin/env bash
set -e

# Independent mocap-only session. All module commands wait for manual Enter.
WORKSPACE=/root/Fast-Drone-XI35
YOPO_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
YOPO_WS="$YOPO_ROOT/build_yopo_minco"
SESH=fd_mocap_only_minco
ROS_MASTER_URI=http://192.168.31.28:11311
ROS_IP=192.168.31.28
ODOM_TOPIC=/motion_capture/motion_capture_odom

for file in "$WORKSPACE/devel/setup.bash" "$YOPO_WS/devel/setup.bash" \
    "$WORKSPACE/shfiles/motion_capture_receive_publish.sh" \
    "$WORKSPACE/src/realflight_modules/px4ctrl/launch/run_ctrl_mocap_only.launch" \
    "$YOPO_ROOT/src/planning/yopo_minco_planner/launch/yopo_minco_mocap_only.launch"; do
    [[ -f "$file" ]] || { echo "Missing: $file" >&2; exit 1; }
done
command -v tmux >/dev/null

if tmux has-session -t "=$SESH" 2>/dev/null; then
    exec tmux attach-session -t "=$SESH"
fi

echo "Mocap-only odom: $ODOM_TOPIC"
echo "Commands are prefilled only. Press Enter in each pane to run."
echo "Use minco_test OR minco; never run both together."
echo "Do not run this stack alongside the existing fusion/VINS stack."

printf -v ENV_COMMAND 'export ROS_MASTER_URI=%q ROS_IP=%q; source %q && source %q && cd %q' \
    "$ROS_MASTER_URI" "$ROS_IP" "$WORKSPACE/devel/setup.bash" \
    "$YOPO_WS/devel/setup.bash" "$WORKSPACE"

prefill() {
    local pane=$1 command=$2
    tmux send-keys -t "$pane" -l -- "$ENV_COMMAND"
    tmux send-keys -t "$pane" C-m
    tmux send-keys -t "$pane" -l -- "$command"
}

FIRST=1
window() {
    local name=$1 command=$2
    if [[ $FIRST == 1 ]]; then
        PANE=$(tmux new-session -d -s "$SESH" -n "$name" -P -F '#{pane_id}')
        FIRST=0
    else
        PANE=$(tmux new-window -t "$SESH:" -n "$name" -P -F '#{pane_id}')
    fi
    prefill "$PANE" "$command"
}
split() {
    local command=$1 pane
    pane=$(tmux split-window -h -t "$PANE" -P -F '#{pane_id}')
    prefill "$pane" "$command"
}

window motioncapture "bash $WORKSPACE/shfiles/motion_capture_receive_publish.sh"
# Start these directly: rs_mavros_imu.sh also starts IMU filter and KF.
window sensors "roslaunch realsense2_camera rs_camera.launch"
split "roslaunch mavros px4.launch"
window px4ctrl "roslaunch px4ctrl run_ctrl_mocap_only.launch odom_topic:=$ODOM_TOPIC"
window minco_test "roslaunch yopo_minco_planner yopo_minco_mocap_only.launch odom_topic:=$ODOM_TOPIC execute:=false engine_file:=$YOPO_ROOT/yopo_minco_sim/deployment/models/yopo_minco_fp16.engine"
window minco "roslaunch yopo_minco_planner yopo_minco_mocap_only.launch odom_topic:=$ODOM_TOPIC execute:=true auto_resume:=true engine_file:=$YOPO_ROOT/yopo_minco_sim/deployment/models/yopo_minco_fp16.engine"
window minco_goal "rosrun yopo_minco_planner minco_goal.py"
window takeoff_land "bash $WORKSPACE/shfiles/takeoff.sh"
split "bash $WORKSPACE/shfiles/land.sh"
# Existing recorder already includes mocap odom, MAVROS IMU and MINCO outputs.
window record "bash $YOPO_ROOT/record_mocap_ground.sh"
window lazer "rosrun dynamic_reconfigure dynparam set /camera/stereo_module emitter_enabled 1"
tmux select-window -t "$SESH:motioncapture"
exec tmux attach-session -t "=$SESH"
