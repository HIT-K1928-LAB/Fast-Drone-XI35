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

## tmux 支持操作

- 滚轮进入 tmux 历史并上下翻页。
- tmux 内部复制粘贴：左键拖拽并松开，然后按鼠标中键或右键粘贴。
- 系统剪贴板复制粘贴：按住 `Shift` 再用左键拖拽，按
  `Ctrl + Shift + C` 复制，使用 `Ctrl + Shift + V` 粘贴。
- 右下角 `EXIT SESSION` 关闭当前整个 session 的所有 window、pane 及其
  子进程组，不影响其他 tmux session。
- 可以鼠标点击下面标签页实现快速切换窗口
- 可以鼠标点击分栏的窗口实现快速切换光标位置

## 日志目录

每次新建 tmux session 时，公共运行时会按本机时间创建一个日志目录：

```text
log/2026-08-25_15-42-18_pc_sim_lidar/
├── latest -> <ROS run_id>
└── <ROS run_id>/
    ├── master.log
    ├── roslaunch-*.log
    ├── *-stdout.log
    └── nodes/
        ├── px4ctrl.log
        └── fast_livo2.log
```

同一 session 的所有 window 和 pane 共用该目录。项目根目录下的
`log/latest` 使用相对符号链接指向最近创建的 session 日志目录，因此在
宿主机和容器中都可以直接打开。如果同一秒创建同名 session，目录名会依次
追加 `_02`、`_03`，不会覆盖已有日志。

## 二次开发

- 增删 window/pane：修改对应机型的 `tmux.sh`。
- 修改 topic、remap、模型或扫描周期：修改对应机型的 `launch/`。
- 修改内外参、PID 或算法开关：修改对应机型的 `config/`。
- 修改 tmux 同步、状态栏或鼠标行为：修改 `lib/tmux_runtime.sh`。
- 不要在 tmux 命令末尾追加 `topic:=...` 等 ROS 参数。

详细机型目录规范见 [profiles 使用说明](profiles/README.md)。
