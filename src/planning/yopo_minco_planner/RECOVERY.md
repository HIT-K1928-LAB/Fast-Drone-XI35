# 实机 MINCO 有界恢复与合并目标入口

本次仅修改源码、脚本并在 /tmp 独立构建验证；用户需自行构建运行版本。没有启动飞行节点或向飞行 ROS master 发命令。

## 行为

仅 `no candidate passes predicted corridor threshold` 进入恢复：
`TRACK -> WAIT_HOVER -> HOVER_REPLAN -> READY -> RESUME`。
其他规划错误、传感器过期、定位故障、时间回跳、遥控接管等进入 STOP。

- 停止发布轨迹命令，等待 PX4Ctrl 的 cmd 超时加 0.5 s（最少 1 s）。这依赖现有 PX4Ctrl 超时悬停机制，不是规划器主动生成的制动轨迹。
- 速度不超过 0.10 m/s、位置相对稳定锚点不超过 0.10 m，连续稳定 1 s 后恢复规划。
- 保留原目标和原测试边界；从当前实测位置、速度及零参考加速度重新规划，不复用失败前的轨迹。
- 至少 5 个不同的新深度帧通过检查，跨度至少 0.4 s，帧间隔不超过 plan_timeout。
- 默认 `auto_resume:=false`：READY 后在目标终端输入 `resume`。确认只有效 1 s，之后仍须下一帧新规划通过。失败、漂移或过长帧间隔撤销确认。
- 可选 `auto_resume:=true`：满足同样检查后自动继续。最多 3 次恢复；恢复阶段检查从首次失败开始累计 30 s 的期限；重复恢复中连续两次未改善目标距离至少 0.05 m 则停止。
- STOP 清除目标，单独发送旧触发不会复活任务。需重新输入目标。
- 正值 kf_fail/vins_fail、IMU 或深度时间回跳超过 1 ms 锁存故障，排查后重启规划器才能解除。
- execute=true 额外要求新鲜 RC（前四通道中位 ±125，通道 5、6 大于 1750）、IN_AIR、已解锁且 OFFBOARD。映射对应当前 PX4Ctrl 实现。

PX4Ctrl 当前没有可直接读取的 FSM 状态话题。稳定判断来自超时、速度和位置，不能当作已直接确认 AUTO_HOVER；实机需核对 PX4Ctrl 日志。该功能不会主动旋转寻找通路，也不会降低走廊阈值。如果每帧持续失败，最终仍会 STOP。没有连续碰撞或实际净空保证。

## 构建（用户手动执行，容器内）

```bash
bash /root/data/yopo-minco-flight-shallow/src/planning/yopo_minco_planner/scripts/build_standalone.sh
source /root/data/yopo-minco-flight-shallow/build_yopo_minco/devel/setup.bash
```

构建后再重启 MINCO 节点以加载新版本。不要同时运行 minco_test 和 minco。旧 tmux 会话不会被启动脚本改写，也不需要为此杀掉正在使用的会话；已有窗口可手动运行：

```bash
source /root/data/yopo-minco-flight-shallow/build_yopo_minco/devel/setup.bash
rosrun yopo_minco_planner minco_goal.py
```

输入 `x y z`（world 绝对坐标）即检查目标并启动，不再需要 minco_start。过近/过远、数据过期或任务忙会被拒绝并显示原因。输入 `resume` 确认恢复，`stop` 取消任务。`q` 或 Ctrl-C 仅退出输入终端，不取消已经运行的任务。

使用 `/root/data/yopo-minco-flight-shallow/tmux-ctrl-plane.sh`（不是旧工作区 shfiles 下的同名文件）。新 tmux 会话用 minco_goal 替代 pub_goal 和 minco_start，窗口命令仍需手动按回车。原 pub_goal.sh 与目标/触发话题保留兼容。

若要选择自动恢复，在原 MINCO 启动命令末尾显式加 `auto_resume:=true`。默认先人工确认恢复，便于检查控制器交接；源代码更新不等于实机放行。

## 录包

record_mocap_ground.sh 新增 `/yopo_minco/accepted_goal` 和 `/yopo_minco/recovery_state`。原目标话题无法记录服务请求接受的目标，所以增加 accepted_goal；拒绝原因保留在服务响应及日志中。

## 离线验证范围

- C++ RecoveryGate：19 项检查（延时、稳定、连续帧、重复帧/帧间隔、漂移、期限、次数和无进展上限等）。
- 隔离 localhost:11319/11320 ROS master，真实 FP16 引擎，execute=false。
- 0.05 m 合成深度触发走廊失败，20 m 合成深度用于通过案例。走廊 safe_radius=0.20 未调低；测试专用运动边界放宽以隔离状态机验证，不是 real.yaml 的完整飞行参数验证。
- 覆盖原接口、原目标不可复活、服务目标拒绝、手动/自动恢复、READY 撤销、故障锁存及数据过期停止。
- 未验证实机动力学、真实环境避障效果或 execute=true 的 PX4Ctrl 交接。没有改变现有航向算法、模型、走廊阈值或飞行速度参数。

复现时先用 YOPO_MINCO_BUILD_DIR 指定新的临时构建目录，再 source 该目录 devel/setup.bash。运行 tests/verify_recovery_ros.py，参数 --node、--package 指向临时副本，--engine、--extrinsic 指向现有只读模型与标定；--auto 测自动恢复，--fault imu/depth 测时间回跳。测试程序拒绝 11311 和已有 master。
