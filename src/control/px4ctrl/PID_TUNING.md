# PX4Ctrl PID 激励脚本

`pid_setpoint_excitation.py` 是一个一次性 `PositionCommand` 发布器。它只等待一条
odom、记录当前位置和 yaw，然后发布测试指令；不判断飞机是否起飞、解锁或处于
OFFBOARD。

脚本发布：

- 控制指令：`/position_cmd`（可通过 `--cmd-topic` 修改）
- 标量输入：`/px4ctrl_tune/excitation/input`
- 测试阶段：`/px4ctrl_tune/excitation/phase`

标量输入在位置轴上的单位是 m，在 yaw 轴上的单位是 rad。阶段值含义：

```text
0 IDLE                  空闲
1 SETTLE                稳定等待阶段
2 POSITIVE/SINE/CHIRP   主要激励阶段
3 NEGATIVE              负向激励阶段
4 RECOVER               恢复阶段
```

## 需要的四类激励

### 1. 小阶跃：初调 Kp、Kd

先调 XY，再调 Z。初始幅值建议 XY 为 0.10～0.15 m，Z 为 0.05～0.10 m。

```bash
rosrun px4ctrl pid_setpoint_excitation.py \
  --mode step --axis forward --amplitude 0.15 --hold 3

rosrun px4ctrl pid_setpoint_excitation.py \
  --mode step --axis lateral --amplitude 0.15 --hold 3

rosrun px4ctrl pid_setpoint_excitation.py \
  --mode step --axis z --amplitude 0.08 --hold 3
```

响应慢且不过冲时逐步增加 Kp；过冲或来回摆动时优先检查/增加 Kd；高频抖动时
检查 Kd 和速度滤波。

### 2. 正反双脉冲：检查对称性、阻尼和状态残留

指令顺序为 `0 → +A → -A → 0`。

```bash
rosrun px4ctrl pid_setpoint_excitation.py \
  --mode doublet --axis forward --amplitude 0.15 --hold 2

rosrun px4ctrl pid_setpoint_excitation.py \
  --mode doublet --axis lateral --amplitude 0.15 --hold 2
```

该测试适合检查前后/左右响应是否对称，以及反向时是否存在积分或滤波历史造成的
明显冲击。

### 3. 单频正弦：检查持续跟踪、相位滞后和噪声

默认只发布变化的位置，期望速度、加速度和 jerk 保持零，用来单独观察反馈 PID。

```bash
rosrun px4ctrl pid_setpoint_excitation.py \
  --mode sine --axis forward --amplitude 0.10 \
  --frequency 0.3 --duration 15
```

建议从 0.2～0.3 Hz 开始。不要在尚未稳定的 PID 上直接使用高频输入。
如果需要测试规划器式的完整轨迹跟踪，可以增加 `--derivatives consistent`，此时脚本
会发布与正弦位置一致的速度、加速度和 jerk。

### 4. Chirp 扫频：检查闭环可用带宽

```bash
rosrun px4ctrl pid_setpoint_excitation.py \
  --mode chirp --axis forward --amplitude 0.08 \
  --start-frequency 0.1 --end-frequency 1.0 --duration 20
```

扫频用于 PID 基本稳定后的验证，不建议用它作为第一次调参输入。

## Yaw 测试

Yaw 幅值通过角度指定，但输入 topic 发布的是 rad：

```bash
rosrun px4ctrl pid_setpoint_excitation.py \
  --mode doublet --axis yaw --amplitude 8 --hold 2
```

Yaw 响应主要由 PX4 姿态/角速度内环决定，不应使用它调整 px4ctrl 的位置 Kp/Kd。

## 不同 pc_sim 定位源

默认 odom 是 ground truth。FAST-LIVO2 和 VINS 分别指定：

```bash
# FAST-LIVO2
rosrun px4ctrl pid_setpoint_excitation.py \
  --odom-topic /LIVO2/imu_propagate \
  --mode doublet --axis forward --amplitude 0.15

# VINS
rosrun px4ctrl pid_setpoint_excitation.py \
  --odom-topic /vins_fusion/imu_propagate \
  --mode doublet --axis forward --amplitude 0.15
```

## 查看输入激励

```bash
rostopic echo /px4ctrl_tune/excitation/input
rostopic echo /px4ctrl_tune/excitation/phase
```

## 控制器调参快照

`pc_sim` 配置默认开启 `/px4ctrl/tune_debug`。需要关闭或改名时修改控制器 YAML：

```yaml
tuning_debug:
    enable: false
    topic: "/px4ctrl/tune_debug"
```

当 `enable: false` 或配置中没有 `tuning_debug` 时，不创建该 publisher，也不填充
高频调参消息。常用曲线组合：

- 位置环：`position_des`、`position_meas`、`position_error`
- 速度反馈：`velocity_des`、`velocity_filtered`、`velocity_error`
- PID 输出：`acceleration_proportional`、`acceleration_derivative`、
  `acceleration_integral`、`acceleration_command`
- 冲击响应：`specific_force_body_meas`、`specific_force_body_filtered`、`body_rate_meas`
- 姿态响应：`attitude_des_rpy`、`attitude_meas_rpy`、`attitude_error_rpy`
- 推力与约束：`thrust_command`、`tilt_command`、两个 `*_limit_exceeded` 标志
- 检查试验有效性：`fsm_state`、`control_dt`、`kp/ki/kd`

可先用下面命令确认消息存在：

```bash
rostopic hz /px4ctrl/tune_debug
rostopic echo -n 1 /px4ctrl/tune_debug
```

## 使用约束

- 脚本不会检查飞行阶段，运行时机由操作者决定。
- 运行前应确保没有规划器同时向 `/position_cmd` 发布。
- 脚本结束后停止发布，px4ctrl 会在 command timeout 后回到 AUTO_HOVER。
- `step` 是设定值阶跃，不是外部扰动。验证抗恒定风和抗冲击能力需要另行向
  Gazebo 机体施加外力。
- 建议先关闭 Ki 和在线推力估计，按 XY、Z、正弦、扫频的顺序测试。

# 在线调节PID参数

```bash
rosrun rqt_reconfigure rqt_reconfigure
```