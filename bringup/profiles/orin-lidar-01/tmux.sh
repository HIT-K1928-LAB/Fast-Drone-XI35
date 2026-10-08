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
    mocap|lidar) ;;
    *)
        echo "Unsupported LOCALIZATION_DEFAULT: $PROFILE_MODE" >&2
        exit 64
        ;;
esac


# shellcheck source=/dev/null
source "$BRINGUP_ROOT/lib/tmux_runtime.sh"

# ==============================================================================
# 用户二次开发区域: 真机
#
# 可在这里修改：
#   1. build_profile_layout：增删 tmux window/pane，调整启动顺序。
#
# ROS topic、remap 和节点参数应放在 launch/；算法参数和内外参应放在
# config/；定位方式应在 profile.env 中选择，不要在这里追加 ROS 参数。
# ==============================================================================

# tmux 布局入口。new_window 创建窗口，add_pane 向已有窗口增加 pane；
# 最后的 manual/auto 表示命令需要确认后启动，或创建 pane 后自动启动。
build_profile_layout() {

    # 定位
    case "$PROFILE_MODE" in
        mocap)
            new_window "location" \
                "roslaunch '$PROFILE_DIR/launch/mocap/motion_capture.launch'" manual
            ;;
        lidar)
            new_window "location" \
                "roslaunch '$PROFILE_DIR/launch/lidar/localization.launch'" manual
            ;;
    esac

    # 启动 OFFBOARD 状态机。lidar 模式的 MAVROS 已由 FAST-LIVO2 launch
    # 管理，不可重复启动；mocap 模式仍需先单独启动 MAVROS。
    if [[ "$PROFILE_MODE" == "lidar" ]]; then
        new_window "offboard_fsm" \
            "roslaunch '$PROFILE_DIR/launch/lidar/offboard_fsm.launch'" manual
    else
        new_window "offboard_fsm" \
            "roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600" manual
        add_pane "offboard_fsm" "offboard_fsm" \
            "roslaunch '$PROFILE_DIR/launch/mocap/offboard_fsm.launch'" manual
    fi
    add_pane "offboard_fsm" "takeoff" "bash '$BRINGUP_ROOT/commands/takeoff.sh'" manual
    add_pane "offboard_fsm" "land" "bash '$BRINGUP_ROOT/commands/land.sh'" manual

    # 启动规划算法
    new_window "planner" \
        "roslaunch '$PROFILE_DIR/launch/$PROFILE_MODE/yopo.launch'" manual
    add_pane "planner" "search_plan" \
        "roslaunch '$PROFILE_DIR/launch/$PROFILE_MODE/search_plan.launch'" manual
    add_pane "planner" "yopo goal" \
        "python3 /root/Fast-Drone-XI35/src/planning/yopo_planner/scripts/publish_nav_goal.py" manual
    add_pane "planner" "yolo detector" \
        "roslaunch '$PROFILE_DIR/launch/lidar/yolo_detector.launch'" manual

    # 启动日志记录服务
    new_window "services" "roslaunch '$PROFILE_DIR/launch/common/services.launch'" auto

    # rosbag
    new_window "rosbag" "rosbag record -O odom_stale_$(date +%Y%m%d_%H%M%S).bag /mavlink/from /mavros/state /mavros/local_position/odom /mavros/setpoint_raw/local /LIVO2/imu_propagate /position_cmd /offboard_fsm/status /diagnostics /aft_mapped_to_init" manual
    add_pane "rosbag" "fastlivo2" "rosbag record --lz4 -O /root/Fast-Drone-XI35/bags/fastlivo_$(date +%Y%m%d_%H%M%S).bag  /LIVO2/imu_propagate /aft_mapped_to_init /aft_mapped_lidar_to_init /tf /tf_static" manual

    # YOLO + YOPO 全链路记录。图像使用 image_transport 压缩话题，兼顾
    # 原始传感器分析与包体积；需要重放检测器时可 republish 为 raw Image。
    add_pane "rosbag" "yolo+yopo" "rosbag record --lz4 -O /root/Fast-Drone-XI35/bags/yopo_$(date +%Y%m%d_%H%M%S).bag /tf /tf_static /LIVO2/imu_propagate /aft_mapped_to_init /aft_mapped_lidar_to_init /path /cloud_registered /camera/color/image_raw/compressed /camera/aligned_depth_to_color/image_raw/compressedDepth /camera/color/camera_info /camera/aligned_depth_to_color/camera_info /mavros/local_position/odom /mavros/vision_pose/pose /mavros/imu/data /mavros/state /mavros/extended_state /mavros/battery /position_cmd /offboard_fsm/reference /mavros/setpoint_raw/local /mavros/setpoint_raw/target_local /offboard_fsm/status /diagnostics /planning/goal /yolo_trt/annotated_image/compressed /yolo_trt/target_point_raw /yolo_trt/target_point /yolo_trt/tracked_target /yolo_trt/target_status /yolo_trt/markers /yopo_log/log /yopo_net/best_traj_visual /yopo_net/trajs_visual /yopo_net/lattice_trajs_visual /yopo_net/preprocessed_depth /yopo_net/preprocessed_depth_float /rosout" manual
}

# ==============================================================================
# 用户二次开发区域结束
# ==============================================================================

# 公共运行时入口，请勿修改。
run_tmux_profile "$@"
