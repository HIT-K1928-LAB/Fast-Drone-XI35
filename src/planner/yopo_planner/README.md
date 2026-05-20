
订阅

/odom_topic 参数，默认 /sim/odom
类型：nav_msgs/Odometry
用途：无人机当前位置、速度、姿态状态

/depth_topic 参数，默认 /depth_image
类型：sensor_msgs/Image
支持编码：32FC1 或 16UC1
用途：深度图输入，预处理成 TensorRT 输入 depth[1,1,96,160]

/move_base_simple/goal
类型：geometry_msgs/PoseStamped
用途：RViz 里 2D Nav Goal / goal point，更新 YOPO 目标点
注意：当前代码只用 msg 的 x/y，z 固定写成 2.0

发布

/so3_control/pos_cmd
类型：quadrotor_msgs/PositionCommand
用途：给 network_control_node / so3_control 用的控制指令

/yopo_net/best_traj_visual
类型：sensor_msgs/PointCloud2
用途：最佳轨迹可视化

/yopo_net/trajs_visual
类型：sensor_msgs/PointCloud2
用途：所有网络候选轨迹可视化


