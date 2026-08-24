# Fast-Drone Bringup

`bringup` 使用一个公开入口选择机型，每个机型独立维护 tmux 布局、ROS
launch 和算法配置。

```text
bringup/
├── flight.sh                  # 唯一公开启动入口
├── lib/
│   └── tmux_runtime.sh        # 公共 tmux session/pane 运行框架
└── profiles/<profile>/
    ├── tmux.sh                # 该机型全部 window 和 pane
    ├── profile.env            # 网络、GPU 和定位方式
    ├── launch/                # ROS 节点连接、topic 和 remap
    └── config/                # 内外参、PID 和算法参数
```

## 启动

总入口只接受一个机型参数：

```bash
bash bringup/flight.sh pc_sim
bash bringup/flight.sh xi35_default
bash bringup/flight.sh xi35_10
bash bringup/flight.sh xi35_euroc
```

不再支持从命令行传入定位方式或视觉前端。修改定位方式后，需要关闭旧
session 并重新启动。

仿真在 `bringup/profiles/pc_sim/profile.env` 中修改：

```bash
ODOMETRY_SOURCE="lidar"       # ground_truth / vins / lidar
```

硬件机型在自己的 `profile.env` 中修改：

```bash
LOCALIZATION_DEFAULT="vins_stereo"  # vins_stereo / mocap
```

辅助命令：

```bash
# 脚本帮助
bash bringup/flight.sh --help
# 列出支持的所有机型
bash bringup/flight.sh --list-profiles
# 配置脚本自动补全
source <(bash bringup/flight.sh completion)
```

session 名称为 `fd_<profile>_<配置模式>`。如果同名 session 已存在，入口
会直接 attach。配置变化后可执行：

```bash
tmux kill-session -t fd_pc_sim_lidar
bash bringup/flight.sh pc_sim
```

## tmux 鼠标与复制

- 滚轮进入 tmux 历史并上下翻页。
- tmux 内部复制粘贴：左键拖拽并松开，然后按鼠标中键或右键粘贴。
- 系统剪贴板复制粘贴：按住 `Shift` 再用左键拖拽，按
  `Ctrl + Shift + C` 复制，使用 `Ctrl + Shift + V` 粘贴。
- 普通拖拽写入的是 tmux buffer，不要再用 `Ctrl + Shift + V` 粘贴，
  否则得到的是外层终端剪贴板中的旧内容。
- pane 中按 `Ctrl-C` 后先给节点 2 秒优雅退出时间；仍未退出时再依次
  使用 `SIGTERM` 和 `SIGKILL`，最长约 3 秒。清理范围包含 roslaunch
  创建的独立进程组，例如 Gazebo、PX4 和 ROS 节点。
- `q` 或 `Esc` 退出复制/翻页模式。
- 右下角 `EXIT SESSION` 关闭当前整个 session 的所有 window、pane 及其
  子进程组，不影响其他 tmux session。

Ptyxis/VTE 不接受 OSC52 写剪贴板，因此 tmux buffer 和系统剪贴板使用
两套明确的操作方式。

## 二次开发边界

- 增删 window/pane：修改对应机型的 `tmux.sh`。
- 修改 topic、remap、模型或扫描周期：修改对应机型的 `launch/`。
- 修改内外参、PID 或算法开关：修改对应机型的 `config/`。
- 修改 tmux 同步、状态栏或鼠标行为：修改 `lib/tmux_runtime.sh`。
- 不要在 tmux 命令末尾追加 `topic:=...` 等 ROS 参数。

详细机型目录规范见 [profiles 使用说明](profiles/README.md)。
