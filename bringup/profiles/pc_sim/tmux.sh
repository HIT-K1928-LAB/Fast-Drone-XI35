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

# ==============================================================================
# 用户二次开发区域：pc_sim
#
# 可在这里修改：
#   1. setup_profile_environment：补充仿真所需的路径环境变量。
#   2. build_profile_layout：增删 tmux window/pane，调整启动顺序。
#
# ROS topic、remap 和节点参数应放在 launch/；算法参数和内外参应放在
# config/；定位方式应在 profile.env 中选择，不要在这里追加 ROS 参数。
# ==============================================================================

# 仿真路径环境初始化。公共运行时会在创建 session 前调用一次，并在每个
# pane 启动时再次调用，使该 pane 中的 roslaunch 继承相同环境。
setup_profile_environment() {
    export FAST_DRONE_ROOT PX4_DIR PX4_BUILD_DIR SITL_GAZEBO
    prepend_env_path GAZEBO_PLUGIN_PATH "$PX4_BUILD_DIR/build_gazebo"
    prepend_env_path LD_LIBRARY_PATH "$PX4_BUILD_DIR/build_gazebo"
    prepend_env_path GAZEBO_MODEL_PATH "$FAST_DRONE_ROOT/src/simulation/gazebo_models"
    prepend_env_path GAZEBO_MODEL_PATH "$SITL_GAZEBO/models"
    prepend_env_path GAZEBO_MODEL_PATH "$FAST_DRONE_ROOT/src/simulation/uav_simulator/simulation_bridges/models"
    prepend_env_path ROS_PACKAGE_PATH "$SITL_GAZEBO"
    prepend_env_path ROS_PACKAGE_PATH "$PX4_DIR"
}

# tmux 布局入口。new_window 创建窗口，add_pane 向已有窗口增加 pane；
# 最后的 manual/auto 表示命令需要确认后启动，或创建 pane 后自动启动。
build_profile_layout() {

    # 启动仿真环境
    new_window "simulator" "roslaunch '$PROFILE_DIR/launch/simulator.launch'" manual

    # 启动定位算法
    case "$PROFILE_MODE" in
        ground_truth)
            new_window "ground truth" "roslaunch '$PROFILE_DIR/launch/ground_truth/localization.launch'" manual
            ;;
        vins)
            new_window "vins location" "roslaunch '$PROFILE_DIR/launch/vins/frontend.launch'" manual
            add_pane "vins location" "vins" "roslaunch '$PROFILE_DIR/launch/vins/estimator.launch'" manual
            ;;
        lidar)
            add_pane "simulator" "lidar format" "roslaunch '$PROFILE_DIR/launch/lidar_bridge.launch'" manual
            new_window "fastlivo2 location" "roslaunch '$PROFILE_DIR/launch/fastlivo2.launch'" manual
            ;;
    esac

    # 启动控制算法
    new_window "px4ctrl" "roslaunch '$PROFILE_DIR/launch/$PROFILE_MODE/px4ctrl.launch'" manual
    add_pane "px4ctrl" "land" "bash '$BRINGUP_ROOT/commands/land.sh'" manual
    add_pane "px4ctrl" "takeoff" "bash '$BRINGUP_ROOT/commands/takeoff.sh'" manual

    # 启动规划算法
    new_window "planner" "roslaunch '$PROFILE_DIR/launch/$PROFILE_MODE/ego_planner.launch'" manual
    add_pane "planner" "search_plan" "roslaunch '$PROFILE_DIR/launch/$PROFILE_MODE/search_plan.launch'" manual
    add_pane "planner" "yopo" "roslaunch '$PROFILE_DIR/launch/$PROFILE_MODE/yopo.launch'" manual

    # 启动日志记录服务
    new_window "services" "roslaunch '$PROFILE_DIR/launch/services.launch'" auto
}

# ==============================================================================
# 用户二次开发区域结束
# ==============================================================================

# 公共运行时入口，请勿修改。
run_tmux_profile "$@"
