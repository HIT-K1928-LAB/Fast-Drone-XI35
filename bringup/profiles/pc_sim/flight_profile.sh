#!/usr/bin/env bash

prepend_env_path() {
    local variable_name="$1"
    local new_path="$2"
    local current_value="${!variable_name:-}"

    case ":$current_value:" in
        *":$new_path:"*)
            return
            ;;
    esac

    printf -v "$variable_name" '%s%s' \
        "$new_path" \
        "${current_value:+:$current_value}"
    export "$variable_name"
}

setup_profile_environment() {
    export FAST_DRONE_ROOT
    export PX4_DIR
    export PX4_BUILD_DIR
    export SITL_GAZEBO

    # 等价于 PX4 的 setup_gazebo.bash，并避免主控/pane 重复追加。
    prepend_env_path \
        GAZEBO_PLUGIN_PATH \
        "$PX4_BUILD_DIR/build_gazebo"

    prepend_env_path \
        LD_LIBRARY_PATH \
        "$PX4_BUILD_DIR/build_gazebo"

    # 反向 prepend，得到 SITL models : Fast-Drone models : 原有路径。
    prepend_env_path \
        GAZEBO_MODEL_PATH \
        "$FAST_DRONE_ROOT/src/simulation/gazebo_models"

    prepend_env_path \
        GAZEBO_MODEL_PATH \
        "$SITL_GAZEBO/models"

    # 最终顺序为 PX4 : sitl_gazebo : 原有 ROS_PACKAGE_PATH。
    prepend_env_path \
        ROS_PACKAGE_PATH \
        "$SITL_GAZEBO"

    prepend_env_path \
        ROS_PACKAGE_PATH \
        "$PX4_DIR"
}

# 以下函数描述 EuRoC 专用的窗口和 pane 命令。
# new_window/add_pane 是 flight.sh 提供的公共 helper。
start_simulation() {
    new_window "simulator" \
        "vglrun -d :0.0 roslaunch px4 indoor1.launch" \
        manual
}

start_location() {
    case "$LOCALIZATION" in
        vins_stereo)
            new_window "vins location" \
                "roslaunch superpoint superpoint_frontend.launch config_file:='$VINS_CONFIG_FILE' imu_topic:='$IMU_TOPIC'" \
                manual

            add_pane "vins location" "vins" \
                "roslaunch vins fast_drone_250.launch config_file:='$VINS_CONFIG_FILE'" \
                manual

            ;;

        fastlivo2)
            new_window "fastlivo2 location" \
                "bash '$WORKSPACE/bringup/scripts/start_fastlivo2.sh'" \
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
        "roslaunch px4ctrl run_ctrl.launch config_file:='$PX4CTRL_CONFIG_FILE' odom_topic:=/vins_fusion/imu_propagate" \
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
    # add_pane "services" "foxglove" \
    #     "sleep 5" auto \
    #     "roslaunch foxglove_bridge foxglove_bridge.launch" auto
}

# 布局 hook：只由顶层进程在 tmux 初始化后调用。
# 保持为文件最后一个函数，便于快速查看 EuRoC 的完整布局。
build_user_layout() {
    start_simulation
    start_location
    start_control
    start_services
}
