
配置

YOPO 节点参数集中放在 config 目录：

- config/yopo_planner_real.yaml：实机默认配置
- config/yopo_planner_sim.yaml：仿真默认配置

常用实机参数：

- velocity：规划速度，第一次实机建议 0.5 到 1.0
- ctrl_dt：控制指令发布步长
- goal_x / goal_y / goal_z：默认目标点，也可以用 RViz 2D Nav Goal 在线更新
- min_depth / max_depth：YOPO 深度输入裁剪范围
- pitch_angle_deg：相机俯仰安装角补偿
- plan_from_reference：是否从上一条参考轨迹继续规划

启动实机：

```bash
source devel/setup.bash
roslaunch yopo_planner yopo_planner.launch
```

启动仿真：

```bash
source devel/setup.bash
roslaunch yopo_planner yopo_sim.launch rviz:=true
```

如果需要临时使用另一份配置：

```bash
roslaunch yopo_planner yopo_planner.launch config_file:=/path/to/yopo_planner_real.yaml
```

不用 RViz，直接模拟发布 2D Nav Goal：

```bash
rosrun yopo_planner publish_nav_goal.py
```

启动后反复输入目标点：

```text
goal> 5.0 0.0 2.0
goal> 8.0 1.0 2.0
```

也可以一次性发布：

```bash
rosrun yopo_planner publish_nav_goal.py 5.0 0.0 2.0
```

指定 yaw 和 frame：

```bash
rosrun yopo_planner publish_nav_goal.py 5.0 1.0 2.0 --yaw 30 --frame-id world
```

订阅

/odom_topic 参数，实机默认 /vins_fusion/imu_propagate，仿真默认 /sim/odom
类型：nav_msgs/Odometry
用途：无人机当前位置、速度、姿态状态

/depth_topic 参数，实机默认 /camera/depth/image_rect_raw，仿真默认 /pcl_render_node/depth
类型：sensor_msgs/Image
支持编码：32FC1 或 16UC1
用途：深度图输入，预处理成 TensorRT 输入 depth[1,1,96,160]

/move_base_simple/goal
类型：geometry_msgs/PoseStamped
用途：RViz 里 2D Nav Goal / goal point，或脚本发布三维目标点，更新 YOPO 目标点
注意：YOPO 会读取 msg 的 x/y/z；RViz 2D Nav Goal 通常 z 为 0，实机建议用 publish_nav_goal.py 输入明确高度

发布

/position_cmd
类型：quadrotor_msgs/PositionCommand
用途：实机默认给 px4ctrl 用的控制指令；仿真中由 yopo_sim.launch 覆盖为 /so3_control/pos_cmd

/yopo_net/best_traj_visual
类型：sensor_msgs/PointCloud2
用途：最佳轨迹可视化

/yopo_net/trajs_visual
类型：sensor_msgs/PointCloud2
用途：所有网络候选轨迹可视化
