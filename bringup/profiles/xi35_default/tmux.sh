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
    vins_stereo|mocap|motion_capture) ;;
    *)
        echo "Unsupported localization mode: $PROFILE_MODE" >&2
        exit 64
        ;;
esac

# shellcheck source=/dev/null
source "$BRINGUP_ROOT/lib/tmux_runtime.sh"

# ==============================================================================
# 用户二次开发区域：xi35_default
#
# 可在 build_profile_layout 中增删 tmux window/pane，调整启动顺序。
# ROS topic 和 remap 应放在 launch/；算法参数和内外参应放在 config/；
# 定位方式应在 profile.env 中选择，不要在这里追加 ROS 参数。
# ==============================================================================

# tmux 布局入口。new_window 创建窗口，add_pane 向已有窗口增加 pane；
# 最后的 manual/auto 表示命令需要确认后启动，或创建 pane 后自动启动。
build_profile_layout() {
    # 启动定位算法
    case "$PROFILE_MODE" in
        vins_stereo)
            new_window "vins location" \
                "roslaunch '$PROFILE_DIR/launch/vins.launch'" manual
            add_pane "vins location" "kf_fusion" \
                "bash '$BRINGUP_ROOT/scripts/start_kf_fusion.sh'" manual
            ;;
        mocap|motion_capture)
            new_window "mocap location" \
                "bash '$BRINGUP_ROOT/scripts/start_mocap.sh'" manual
            ;;
    esac

    # 启动控制算法
    new_window "px4ctrl" \
        "roslaunch '$PROFILE_DIR/launch/px4ctrl.launch'" manual
    add_pane "px4ctrl" "land" \
        "bash '$BRINGUP_ROOT/commands/land.sh'" manual
    add_pane "px4ctrl" "takeoff" \
        "bash '$BRINGUP_ROOT/commands/takeoff.sh'" manual

    # 启动规划算法
    new_window "planner" \
        "roslaunch '$PROFILE_DIR/launch/ego_planner.launch'" manual
    add_pane "planner" "search_plan" \
        "roslaunch '$PROFILE_DIR/launch/search_plan.launch'" manual
    add_pane "planner" "yopo" \
        "roslaunch '$PROFILE_DIR/launch/yopo.launch'" manual

    # 启动日志记录服务
    new_window "services" \
        "roslaunch '$PROFILE_DIR/launch/services.launch'" auto
}

# ==============================================================================
# 用户二次开发区域结束
# ==============================================================================

# 公共运行时入口，请勿修改。
run_tmux_profile "$@"
