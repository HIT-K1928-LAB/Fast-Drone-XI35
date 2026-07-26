# Profiles 使用说明

一个 profile 表示一台具体无人机或一套完整仿真平台的配置。每个可用 profile 必须包含 `profile.env`。

## 目录结构

推荐结构：

```text
profiles/<profile_name>/
├── profile.env
├── flight_profile.sh         # 可选：扩展环境并覆写 tmux 布局
├── px4ctrl/
│   └── ctrl_param_fpv.yaml
└── vins/
    ├── fast_drone_250.yaml
    ├── left.yaml
    ├── right.yaml
    ├── mono_downward.yaml
    └── realsense_bind_exposure.json
```

主配置中使用的相对文件应放在同一个 profile 内。例如：

```yaml
cam0_calib: "left.yaml"
cam1_calib: "right.yaml"
```

当主配置为：

```text
profiles/xi35_10/vins/fast_drone_250.yaml
```

VINS会加载：

```text
profiles/xi35_10/vins/left.yaml
profiles/xi35_10/vins/right.yaml
```

## profile.env

示例：

```bash
PROFILE_NAME="xi35_10"

VINS_CONFIG="vins/fast_drone_250.yaml"
REALSENSE_CONFIG="vins/realsense_bind_exposure.json"
PX4CTRL_CONFIG="px4ctrl/ctrl_param_fpv.yaml"

ROS_MASTER_URI_DEFAULT="http://192.168.31.97:11311"
ROS_IP_DEFAULT="192.168.31.97"

CAMERA_TYPE="realsense"
LOCALIZATION_DEFAULT="vins_stereo"

# 可选；相对路径从 Fast-Drone-XI35 仓库根目录解析
# FLIGHT_PROFILE_SCRIPT="bringup/profiles/xi35_10/flight_profile.sh"
```

字段说明：

| 字段 | 说明 |
|---|---|
| `PROFILE_NAME` | profile名称，建议与目录名一致 |
| `VINS_CONFIG` | 相对 profile目录的 VINS主配置路径 |
| `PX4CTRL_CONFIG` | 相对 profile目录的 PX4Ctrl配置路径 |
| `REALSENSE_CONFIG` | RealSense高级模式 JSON路径；需要相机启动命令显式传入时使用 |
| `ROS_MASTER_URI_DEFAULT` | 该机型启动时强制使用的 ROS master地址 |
| `ROS_IP_DEFAULT` | 该机型启动时强制使用的本机 ROS IP |
| `CAMERA_TYPE` | 相机类型说明，会显示在 status窗口中 |
| `LOCALIZATION_DEFAULT` | 未传第二个命令行参数时使用的定位方式 |
| `FLIGHT_PROFILE_SCRIPT` | 可选的环境和 tmux布局覆写脚本；相对路径从仓库根目录解析 |

使用内置布局时，`flight.sh` 会自动校验并解析 `VINS_CONFIG` 和 `PX4CTRL_CONFIG`。配置 `FLIGHT_PROFILE_SCRIPT` 后，这两个字段变为可选，外部脚本应自行校验它实际需要的配置。新增其他模块配置时，可在外部脚本中使用 `$PROFILE_DIR` 将 profile相对路径解析为绝对路径。

## 扩展环境并覆写 tmux布局

如果某个 profile需要额外环境变量或不使用默认完整布局，可在该 profile目录中新建 `flight_profile.sh`：

```bash
#!/usr/bin/env bash

setup_profile_environment() {
    # profile.env 中的变量可在这里直接读取。需要由 ROS、Gazebo、
    # PX4 等子进程继承的变量必须显式 export。
    export PX4_SIM_MODEL="$PX4_MODEL"
    export GAZEBO_MODEL_PATH="$PROFILE_DIR/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
}

build_user_layout() {
    start_localization
}
```

然后在对应的 `profile.env` 中配置仓库根目录相对路径：

```bash
FLIGHT_PROFILE_SCRIPT="bringup/profiles/xi35_10/flight_profile.sh"
```

`flight.sh` 会先加载 `profile.env`，再使用 `source` 加载该文件。因此，`flight_profile.sh` 可以直接读取 `profile.env` 中的全部 shell变量。可选的 `setup_profile_environment()` 会在顶层启动进程和每个 tmux pane 中各执行一次，应该只设置环境且保持可重复执行；不要在其中启动程序。

外部脚本必须定义 `build_user_layout()`。配置了脚本但文件不存在、不可读或没有定义该函数时，启动会直接报错，不会回退到默认布局。未定义 `FLIGHT_PROFILE_SCRIPT` 或值为空时，不会加载任何外部脚本，仍使用内置的 `start_localization + start_control + start_services`。

`profile.env` 中的普通赋值是 shell变量，不会自动导出给 ROS等子进程。需要子进程继承的变量，应在 `setup_profile_environment()` 中显式 `export`。外部脚本应只定义函数，不要在文件顶层直接启动程序；`build_user_layout()` 可以调用 `flight.sh` 提供的 `new_window`、`add_pane`、`start_localization`、`start_control` 和 `start_services`。该文件会作为 shell代码执行，只应指向仓库中的可信脚本。

## 新增机型

以默认配置为模板：

```bash
cp -a bringup/profiles/xi35_default bringup/profiles/xi35_11
```

然后：

1. 将 `profile.env` 中的 `PROFILE_NAME` 改为 `xi35_11`。
2. 修改该机型的 ROS网络参数。
3. 替换相机内参和相机—IMU外参。
4. 修改 PX4Ctrl控制参数。
5. 检查 YAML 中的话题名称和图像分辨率。
6. 将新 profile提交到 Git。

确认脚本已经识别：

```bash
./bringup/tmux/flight.sh --list-profiles
```

只有存在 `profile.env` 的目录才会被列出。

## Git管理建议

应该提交：

- `profile.env`
- VINS主配置
- 相机内参和外参
- PX4Ctrl参数
- RealSense配置

不建议提交：

- TensorRT `.engine` 文件
- ROS日志和运行轨迹
- 标定 bag大文件
- 临时优化输出
- 机器密码和私钥
