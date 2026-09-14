#!/usr/bin/env bash

SESSION="yopominco"

# 如果 session 已存在，不重复创建，直接进入
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "tmux session '$SESSION' already exists."
    tmux attach-session -t "$SESSION"
    exit 0
fi

# ---------------------------------------------------------
# 各窗口要预先输入的命令
# 注意：这里只输入，不发送 Enter
# ---------------------------------------------------------


CMD_SIM='source /root/Fast-Drone-XI35/Fast-Drone-XI35/setup_yopominco.sh && roslaunch simulation_bridges pc_sim.launch world:="$FD_WS/third_party/px4_sitl/Tools/sitl_gazebo/worlds/empty.world"  sdf:=iris_3d_gpu_lidar_yopo model_root:="$FD_WS/yopo_minco_sim/models" gui:=true'

CMD_RVIZ='source /root/Fast-Drone-XI35/Fast-Drone-XI35/setup_yopominco.sh && rviz -d /root/Fast-Drone-XI35/Fast-Drone-XI35/third_party/YOPO/YOPO/yopo.rviz'

CMD_ODOM='source /root/Fast-Drone-XI35/Fast-Drone-XI35/setup_yopominco.sh && roslaunch "$FD_WS/bringup/profiles/pc_sim/launch/ground_truth/localization.launch"'

# 可选：只预览 YOPO 轨迹、不执行飞行
# YOPO + PX4Ctrl 执行接口
CMD_YOPO='source /root/Fast-Drone-XI35/Fast-Drone-XI35/setup_yopominco.sh && source /root/miniconda3/etc/profile.d/conda.sh && conda activate yopo-minco && python "$FD_WS/yopo_minco_sim/run_yopo_px4ctrl.py" --execute'

# 可选：preview_yopo 使用 yopo_preview 时需要
CMD_TF='source /opt/ros/noetic/setup.bash && rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 world yopo_preview'

CMD_PX4CTRL='source /root/Fast-Drone-XI35/Fast-Drone-XI35/setup_yopominco.sh && PX4_BIN="$FD_WS/third_party/px4_sitl/build/px4_sitl_default/bin" && "$PX4_BIN/px4-param" set COM_RC_IN_MODE 4 && "$PX4_BIN/px4-param" set COM_RCL_EXCEPT 4 && roslaunch "$FD_WS/bringup/profiles/pc_sim/launch/ground_truth/px4ctrl.launch"'

CMD_TAKEOFF='source /root/Fast-Drone-XI35/Fast-Drone-XI35/setup_yopominco.sh && bash "$FD_WS/bringup/commands/takeoff.sh"'


# ---------------------------------------------------------
# 创建 tmux session
# ---------------------------------------------------------

tmux new-session -d -s "$SESSION" -n "0-sim"

# 设置窗口编号从 1 开始
tmux set-option -t "$SESSION" base-index 0
tmux set-option -t "$SESSION" renumber-windows on

# 创建其他窗口
tmux new-window -t "$SESSION" -n "1-rviz"
tmux new-window -t "$SESSION" -n "2-odom"
tmux new-window -t "$SESSION" -n "3-preview"
tmux new-window -t "$SESSION" -n "4-tf"
tmux new-window -t "$SESSION" -n "5-px4ctrl"
tmux new-window -t "$SESSION" -n "6-takeoff"

# ---------------------------------------------------------
# 把命令输入到对应窗口
#
# 关键：
# 没有在 send-keys 后面加 C-m
# 所以命令只会显示在终端中，不会执行
# ---------------------------------------------------------

tmux send-keys -t "$SESSION:0-sim" "$CMD_SIM"
tmux send-keys -t "$SESSION:1-rviz" "$CMD_RVIZ"
tmux send-keys -t "$SESSION:2-odom" "$CMD_ODOM"
tmux send-keys -t "$SESSION:3-preview" "$CMD_PREVIEW"
tmux send-keys -t "$SESSION:4-tf" "$CMD_TF"
tmux send-keys -t "$SESSION:5-px4ctrl" "$CMD_PX4CTRL"
tmux send-keys -t "$SESSION:6-takeoff" "$CMD_TAKEOFF"

# 回到第一个窗口
tmux select-window -t "$SESSION:1-sim"

# 进入 tmux
tmux attach-session -t "$SESSION"
