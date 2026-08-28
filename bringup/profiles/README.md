# Profiles 使用说明

一个 profile 表示一台具体无人机或一套完整仿真平台。每个可用
profile 必须包含 `profile.env`，ROS 节点连接和算法参数分别放在
`launch/` 与 `config/` 中。

## 目录结构

```text
profiles/<profile_name>/
├── profile.env                 # 网络、运行环境和模式选择
├── tmux.sh                     # 该机型专用 tmux 布局和环境 hook
├── launch/                     # 节点组合、topic、remap 和模型选择
│   ├── vins.launch
│   ├── px4ctrl.launch
│   ├── ego_planner.launch
│   └── services.launch
└── config/                     # 算法数值参数
    ├── vins/
    │   ├── fast_drone_250.yaml
    │   ├── left.yaml
    │   └── right.yaml
    └── px4ctrl/
        └── ctrl_param_fpv.yaml
```

职责边界：

- `profile.env`：ROS 网络、GPU/PX4 环境和定位模式选择。
- `tmux.sh`：tmux window/pane、启动顺序、选择哪个 profile launch。
- `launch/`：ROS 图连接，包括 topic、remap、模型名和扫描周期。
- `config/`：内外参、PID、噪声、算法开关等数值参数。

tmux 命令只启动 profile 内的 launch，不在命令末尾追加运行参数：

```bash
roslaunch "$PROFILE_DIR/launch/px4ctrl.launch"
```

需要修改里程计 topic 时，修改对应机型的 `launch/px4ctrl.launch` 和规划
launch；当前项目保留真值、VINS 和 FAST-LIVO2 原有的定位输出 topic，
没有强制统一接口。

## 配置文件的相对路径

同一算法的主配置和子配置应放在同一目录。例如：

```yaml
cam0_calib: "left.yaml"
cam1_calib: "right.yaml"
```

主配置位于：

```text
profiles/xi35_10/config/vins/fast_drone_250.yaml
```

VINS 会加载同目录下的 `left.yaml` 和 `right.yaml`。profile launch 使用
`$(dirname)` 引用本 profile 的配置，因此可以直接通过绝对路径启动，
也不依赖 tmux 注入算法配置变量。

## profile.env

硬件 profile 示例：

```bash
PROFILE_NAME="xi35_10"

ROS_MASTER_URI_DEFAULT="http://192.168.31.97:11311"
ROS_IP_DEFAULT="192.168.31.97"

CAMERA_TYPE="realsense"
LOCALIZATION_DEFAULT="vins_stereo"
```

字段说明：

| 字段 | 说明 |
|---|---|
| `PROFILE_NAME` | profile 名称，建议与目录名一致 |
| `ROS_MASTER_URI_DEFAULT` | 该机型强制使用的 ROS master 地址 |
| `ROS_IP_DEFAULT` | 该机型强制使用的本机 ROS IP |
| `CAMERA_TYPE` | status 窗口显示的相机类型 |
| `LOCALIZATION_DEFAULT` | 硬件 profile 使用的定位方式，只能通过配置修改 |
| `ODOMETRY_SOURCE` | `pc_sim` 使用的定位方式：真值、VINS 或激光 |

不要在 `profile.env` 中维护 `odom_topic`、`imu_topic`、扫描周期或算法
YAML 路径；这些内容分别属于 `launch/` 和 `config/`。

## tmux 布局

每个 profile 都直接维护自己的 `tmux.sh`。脚本定义
`build_profile_layout()`，并可选定义可重复执行的
`setup_profile_environment()`：

```bash
#!/usr/bin/env bash

setup_profile_environment() {
    export PX4_SIM_MODEL="iris"
    export GAZEBO_MODEL_PATH="$PROFILE_DIR/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
}

build_profile_layout() {
    new_window "simulator" \
        "roslaunch '$PROFILE_DIR/launch/simulator.launch'" \
        manual
}

run_tmux_profile "$@"
```

`tmux.sh` 先加载 `profile.env` 和 `bringup/lib/tmux_runtime.sh`，再调用
`run_tmux_profile`。需要 ROS、Gazebo 或 PX4 子进程继承的变量必须在环境
hook 中显式 `export`。

## 新增机型

```bash
cp -a bringup/profiles/xi35_default bringup/profiles/xi35_11
```

然后：

1. 修改 `profile.env` 中的 `PROFILE_NAME` 和 ROS 网络参数。
2. 修改 `launch/` 中的设备、topic 和 remap。
3. 替换 `config/vins/` 中的相机内参和相机—IMU 外参。
4. 修改 `config/px4ctrl/` 中的控制参数。
5. 检查图像分辨率、IMU 频率和定位输出 topic。
6. 运行 `bash bringup/flight.sh --list-profiles` 确认已识别。

只有存在 `profile.env` 的目录才会被列出。

## Git 管理建议

应该提交 `tmux.sh`、`profile.env`、`launch/` 和 `config/`。不要提交 TensorRT
`.engine`、ROS 日志、标定 bag、临时优化输出、密码或私钥。
