#!/usr/bin/env bash

WORKSPACE="$HOME/Fast-Drone-XI35"
ROS_MASTER_URI="http://192.168.31.97:11311"
ROS_IP="192.168.31.97"

select_mode() {
    while true; do
        echo "Select ctrl plane mode:"
        echo "  1) Visual odometry / VINS"
        echo "  2) Motion capture"
        read -r -p "Enter mode number [1-2]: " choice

        case "$choice" in
            1)
                MODE="vins"
                SESH="fd_vins"
                FIRST_WINDOW="rspx4"
                return
                ;;
            2)
                MODE="motion_capture"
                SESH="fast_drone_xi35_motion_capture_tmux_session"
                FIRST_WINDOW="motioncapture"
                return
                ;;
            *)
                echo "Invalid mode number: $choice"
                ;;
        esac
    done
}

ros_env_setup_command() {
    echo "export ROS_MASTER_URI=$ROS_MASTER_URI; export ROS_IP=$ROS_IP; source $WORKSPACE/devel/setup.bash; cd $WORKSPACE"
}

send_target_command() {
    local target="$1"
    local command="$2"
    local run_mode="${3:-auto}"

    tmux send-keys -t "$target" "$(ros_env_setup_command)" C-m

    if [ "$run_mode" = "manual" ]; then
        tmux send-keys -t "$target" "$command"
    else
        tmux send-keys -t "$target" "$command" C-m
    fi
}

send_window_command() {
    local window_name="$1"
    local command="$2"
    local run_mode="${3:-auto}"

    send_target_command "$SESH:$window_name" "$command" "$run_mode"
}

create_window() {
    local window_name="$1"
    local command="$2"
    local run_mode="${3:-auto}"

    if [ "$WINDOW_CREATED" = 0 ]; then
        tmux new-session -d -s "$SESH" -n "$window_name"
        WINDOW_CREATED=1
    else
        tmux new-window -t "$SESH" -n "$window_name"
    fi

    send_window_command "$window_name" "$command" "$run_mode"
}

create_horizontal_pane() {
    local pane_name="$1"
    local command="$2"
    local run_mode="${3:-auto}"
    local pane_id

    pane_id=$(tmux split-window -h -t "$SESH:" -P -F "#{pane_id}")
    tmux select-pane -t "$pane_id" -T "$pane_name"
    send_target_command "$pane_id" "$command" "$run_mode"
    tmux select-layout -t "$SESH:" even-horizontal >/dev/null
}

create_vertical_pane() {
    local pane_name="$1"
    local command="$2"
    local run_mode="${3:-auto}"
    local pane_id

    pane_id=$(tmux split-window -v -t "$SESH:" -P -F "#{pane_id}")
    tmux select-pane -t "$pane_id" -T "$pane_name"
    send_target_command "$pane_id" "$command" "$run_mode"
    tmux select-layout -t "$SESH:" even-vertical >/dev/null
}

create_yopo_planner_window() {
    local top_left_pane_id
    local top_right_pane_id
    local bottom_pane_id

    create_window "yopoplaner" "roslaunch yopo_planner yopo_planner.launch" "manual"
    top_left_pane_id=$(tmux display-message -p -t "$SESH:yopoplaner" "#{pane_id}")

    bottom_pane_id=$(tmux split-window -v -t "$top_left_pane_id" -P -F "#{pane_id}")
    tmux select-pane -t "$bottom_pane_id" -T "yolo_position"
    send_target_command "$bottom_pane_id" "roslaunch yolo_trt_detector yolo_trt_detector.launch" "manual"

    top_right_pane_id=$(tmux split-window -h -t "$top_left_pane_id" -P -F "#{pane_id}")
    tmux select-pane -t "$top_right_pane_id" -T "pubgoal"
    send_target_command "$top_right_pane_id" "python3 src/planner/yopo_planner/scripts/publish_nav_goal.py" "manual"

    tmux select-pane -t "$top_left_pane_id"
}

start_mode_windows() {
    case "$MODE" in
        vins)
            create_window "rspx4" "bash $WORKSPACE/shfiles/rspx4_xp.sh" "manual"
            create_horizontal_pane "kffusion" "bash $WORKSPACE/shfiles/kf_fusion.sh" "manual"
            ;;
        motion_capture)
            create_window "motioncapture" "bash $WORKSPACE/shfiles/motion_capture_receive_publish.sh" "manual"
            create_horizontal_pane "rs_mavros_imu_kf-fusion" "bash $WORKSPACE/shfiles/rs_mavros_imu.sh" "manual"
            ;;
        *)
            echo "Unknown mode: $MODE"
            exit 1
            ;;
    esac
}

start_common_windows() {
    create_window "px4ctrl" "roslaunch px4ctrl run_ctrl.launch" "manual"
    create_window "egoplanner" "roslaunch ego_planner single_run_in_exp.launch" "manual"
    create_horizontal_pane "searchplan" "roslaunch search_plan search_plan.launch" "manual"
    create_window "takeoff" "bash $WORKSPACE/shfiles/takeoff.sh" "manual"
    create_horizontal_pane "land" "bash $WORKSPACE/shfiles/land.sh" "manual"
    create_yopo_planner_window
}

# 程序启动 #

# 选择使用Vins还是Mocap
select_mode
# 检查会话是否已存在
tmux has-session -t "$SESH" 2>/dev/null

if [ $? != 0 ]; then
    WINDOW_CREATED=0
    # 启动Vins或Mocap窗口
    start_mode_windows
    # 启动公共指令窗口
    start_common_windows
    tmux select-window -t "$SESH:$FIRST_WINDOW"
fi
# 进入tmux会话
tmux attach-session -t "$SESH"
