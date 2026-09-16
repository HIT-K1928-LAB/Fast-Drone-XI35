# YOPO-MINCO Gazebo 制动后恢复

2026-09-14。新增 `run_yopo_px4ctrl_recovery.py`，依赖同目录原 `run_yopo_px4ctrl.py`。
以容器 `liuyiming_fastdro` 当前脚本为基线；本地旧版 base 文件并非远程最新版本。
原适配器、模型权重、走廊阈值和控制器没有修改。未自动启动节点或飞行。

## 行为

`IDLE → TRACK → BRAKE → HOVER_REPLAN → TRACK`。

- 仅 `upstream corridor admission requested braking` 允许恢复。
- BRAKE 停止发位置指令，利用 PX4Ctrl 指令超时回到悬停，不是主动生成制动轨迹。
- 读取 `/px4ctrl/msg_timeout/cmd`，等待该值加 0.5 秒仿真时间，再确认速度 ≤0.15 m/s、位置在 0.10 m 范围内持续稳定至少 1 秒；高度需 ≥0.6 m。
- 稳定悬停判断来自指令间隔和里程计，**没有直接读取 PX4Ctrl 的 FSM 状态**。验收时同时查看控制器 AUTO_HOVER 日志。
- 悬停重规划不发控制指令；每次从最新实际 P/V 和零 A 初始化，清除旧参考及旧轨迹时间。
- 最多以 10 Hz 处理不同时间戳的深度帧；连续至少 5 帧、覆盖至少 0.4 秒通过筛选才有资格恢复。
- 筛选保留原走廊判定；检查所选轨迹 P/V/A 有限，并用 101 个时间点检查高度 ≥0.6 m、起点偏差 ≤0.15 m。这不是连续几何碰撞证明，也不会从不合格的最优候选自动改选其他候选。
- `--auto-resume` 才允许通过后续飞；不带该参数时只显示 `READY_PREVIEW`。
- 每个目标最多进入恢复 3 次；每次恢复最长 30 秒墙钟时间。失败后 STOPPED，需人工检查并重新给目标。
- 手动停止、到达、传感器过期、PX4 状态异常、跟踪误差和异常数据均为终止状态，不会自动恢复。
- 保留原目标范围 40 m、到达距离 5 m、目标高度取当前高度的行为。没有解决终端精确到点问题。

## 首次试验

1. 在旧 YOPO 终端按 Ctrl+C，确认停止；保留 Gazebo、里程计、PX4Ctrl 和 RViz。不要同时运行 goto_point 或其他位置命令节点。
2. 将飞机回到相同测试起点并稳定悬停。若用 goto_point 复位，执行完成后退出该节点。
3. 新终端进入容器后：

```bash
cd /root/Fast-Drone-XI35/Fast-Drone-XI35
source ./setup_yopominco.sh
source /root/miniconda3/etc/profile.d/conda.sh
conda activate yopo-minco
python "$FD_WS/yopo_minco_sim/run_yopo_px4ctrl_recovery.py" --execute
```

**这个命令仍会执行收到目标后的初始飞行，只是不自动恢复制动后的飞行。**

另一个已 source ROS 的终端：

```bash
rostopic echo /yopo_minco/recovery_state
```

照常用 RViz 给原来的墙后目标。关注 TRACK → BRAKE → HOVER_REPLAN → READY_PREVIEW；BRAKE/HOVER_REPLAN 阶段 `/position_cmd` 不应有新消息。控制器应进入 AUTO_HOVER，机体稳定。30 秒后停止观察并进入 STOPPED 是预设超时。

## 自动恢复试验

预览符合预期后 Ctrl+C 退出，重新复位、稳定悬停。启动：

```bash
python "$FD_WS/yopo_minco_sim/run_yopo_px4ctrl_recovery.py" --execute --auto-resume
```

重新给相同目标。验收日志为 TRACK → BRAKE → HOVER_REPLAN → `RESUME`/TRACK；恢复后才重新产生位置命令。即使成功恢复，仍需在 Gazebo 检查真实避障与高度，不把网络走廊通过当成实际安全证明。

若 HOVER_REPLAN 长时间不恢复，看 `good_frames` 和停止原因：原走廊失败、所选轨迹采样高度低于 0.6 m 或非有限数据都会清零。不要直接减小安全阈值。

随时停止当前任务并阻止自动恢复：

```bash
rostopic pub -1 /yopo_minco/stop std_msgs/Empty '{}'
```

## 回退和离线验证

退出新脚本后恢复原启动命令：

```bash
python "$FD_WS/yopo_minco_sim/run_yopo_px4ctrl.py" --execute
```

离线状态机测试：

```bash
cd "$FD_WS/yopo_minco_sim"
python -m unittest test_yopo_recovery -v
```

测试替换网络推理和时钟，不初始化 ROS 节点、不发布命令；用于检查状态转移、指令隔离、参考重置和失效处理，不能替代真实网络闭环飞行验收。
