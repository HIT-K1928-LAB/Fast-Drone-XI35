# tmux 飞行启动说明

统一入口：

```text
bringup/tmux/flight.sh
```

## 启动

```bash
# 默认：xi35_default + profile默认定位方式 + xfeat
./bringup/tmux/flight.sh

# 指定机型
./bringup/tmux/flight.sh xi35_10

# 指定定位方式
./bringup/tmux/flight.sh xi35_10 vins_stereo

# 指定视觉前端
./bringup/tmux/flight.sh xi35_10 vins_stereo xfeat
```

目前 `vins_stereo` 分支实际启动 `bringup/scripts/start_vins.sh`，因此已接通的前端是 `xfeat`。第三个参数会显示在 status窗口中；若要支持其他前端，还需要在 `start_localization()` 中根据 `$FRONTEND` 选择对应启动命令。

帮助：

```bash
./bringup/tmux/flight.sh --help
./bringup/tmux/flight.sh --list-profiles
```

## Tab补全

当前 shell中启用 Bash补全：

```bash
source <(./bringup/tmux/flight.sh completion)
```

之后输入以下内容并按 `Tab`，会动态读取 `bringup/profiles/*/profile.env`：

```bash
./bringup/tmux/flight.sh xi<Tab>
```

永久启用时，将补全命令使用的脚本路径展开为绝对路径后写入 `~/.bashrc`：

```bash
flight_script="$(realpath ./bringup/tmux/flight.sh)"
printf 'source <(%q completion)\n' "$flight_script" >> ~/.bashrc
source ~/.bashrc
```

补全还会提示第二个参数的定位方式和第三个参数的视觉前端。

session名称格式：

```text
fd_<profile>_<localization>
```

例如：

```text
fd_xi35_10_vins_stereo
```

如果 session已经存在，脚本会直接 attach，不会重新加载配置。配置改变后需要重建：

```bash
tmux kill-session -t fd_xi35_10_vins_stereo
./bringup/tmux/flight.sh xi35_10 vins_stereo
```

## 界面操作

脚本默认开启 tmux鼠标支持：

- 点击选择 pane。
- 点击底部状态栏切换 window。
- 拖动边框调整 pane大小。
- 滚轮查看当前 pane的历史输出。
- 按 `q` 或 `Esc` 退出复制/翻页模式。
- 需要选择终端文字时，通常按住 `Shift` 再拖动鼠标。

底部右侧的 `EXIT SESSION` 会立即关闭整个 session及其所有程序，没有二次确认。

`Ctrl+C` 行为与普通终端一致：

- 命令尚未回车时，取消预填命令并进入空终端。
- 程序运行时，终止当前程序但保留 pane。

## 修改窗口布局

所有 profile共用的默认布局可以修改 `flight.sh` 顶部的：

```text
USER MAINTENANCE AREA
```

底层环境加载、布局屏障和终端信号处理不需要修改。

### 为单个 profile覆写布局

在 `profile.env` 中配置：

```bash
FLIGHT_LAYOUT_SCRIPT="bringup/profiles/xi35_10/flight_layout.sh"
```

路径相对于仓库根目录。对应脚本必须定义 `build_user_layout()`，例如只启动定位模块：

```bash
#!/usr/bin/env bash

build_user_layout() {
    start_localization
}
```

脚本由 `flight.sh` 使用 `source` 加载，而不是作为子进程执行，因此新定义的 `build_user_layout()` 会取代内置默认布局。未配置 `FLIGHT_LAYOUT_SCRIPT` 时行为不变。布局脚本应只定义函数，不要在顶层执行启动命令。

### 添加所有 pane共用的环境变量

在用户维护区的 `setup_common_environment()` 中添加。例如 ROS日志统一保存到仓库的 `log/`：

```bash
setup_common_environment() {
    export ROS_LOG_DIR="$WORKSPACE/log"
    mkdir -p "$ROS_LOG_DIR"
}
```

这里使用 `$WORKSPACE/log`，配置上相对于仓库根目录，传给 ROS 的实际值则是稳定的绝对路径。

### 新建 window

等待手动回车：

```bash
new_window "px4ctrl" \
    "roslaunch px4ctrl run_ctrl.launch config_file:='$PX4CTRL_CONFIG_FILE'" \
    manual
```

立即执行：

```bash
new_window "logger" \
    "roslaunch uav_utils rosout_file_logger.launch" \
    auto
```

### 向已有 window添加 pane

```bash
add_pane "planner" "search_plan" \
    "roslaunch search_plan search_plan.launch" \
    manual
```

第一个参数必须是已经由 `new_window` 创建的 window名称。pane数量变化后会自动使用 `tiled` 布局。

### 一个 pane中依次执行多条命令

先自动执行，再预填下一条命令等待回车：

```bash
add_pane "flight" "land" \
    "rosparam set /flight_mode landing" auto \
    "bash '$WORKSPACE/bringup/commands/land.sh'" manual
```

命令按从左到右的顺序执行。若前一条是长期运行的前台程序，只有它结束后才会处理下一条；需要后台执行时显式添加 `&`。

### 增加定位方式

在 `start_localization()` 中添加分支：

```bash
fastlio)
    new_window "fastlio" \
        "roslaunch fast_lio mapping.launch" \
        manual
    ;;
```

然后可通过以下命令选择：

```bash
./bringup/tmux/flight.sh xi35_10 fastlio
```

### 增加模块分组

推荐将相关窗口封装成独立函数：

```bash
start_detection() {
    new_window "detector" \
        "roslaunch detector detector.launch" \
        manual
}
```

再加入：

```bash
build_user_layout() {
    start_localization
    start_control
    start_detection
}
```

## 常用 tmux命令

```bash
# 查看session
tmux ls

# 进入session
tmux attach -t fd_xi35_10_vins_stereo

# 暂时离开但不结束程序
# 在tmux内按 Ctrl+b，然后按 d

# 关闭session及全部程序
tmux kill-session -t fd_xi35_10_vins_stereo
```
